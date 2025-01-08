from fastapi import FastAPI, WebSocket, Depends
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import serial
import asyncio
import json
from typing import Dict, Set
import uvicorn
from enum import Enum
import multiprocessing
import sys
import os
import queue
import threading

# Add the parent folder 'a' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from simulation.simulation import run_simulation

IS_SIM = True

class SimulationLogger:
    def __init__(self):
        self.log_queue = queue.Queue()
        self._stop_event = threading.Event()

    async def broadcast_logs(self, websockets):
        while not self._stop_event.is_set():
            try:
                log_message = self.log_queue.get_nowait()
                disconnected_sockets = set()
                for websocket in websockets:
                    try:
                        await websocket.send_json({
                            'type': 'simulation_log',
                            'message': log_message
                        })
                    except Exception as e:
                        print(f"Error broadcasting log: {e}")
                        disconnected_sockets.add(websocket)
                
                websockets -= disconnected_sockets
            except queue.Empty:
                await asyncio.sleep(0.1)

    def stop(self):
        self._stop_event.set()

class ControlMode(Enum):
    HARDWARE = "hardware"
    SIMULATION = "simulation"

class MachineController:
    def __init__(self, mode=ControlMode.SIMULATION, port='/dev/ttyACM0', baud_rate=115200):
        self.mode = mode
        self.active_websockets: Set[WebSocket] = set()
        self.running = True
        
        if self.mode == ControlMode.HARDWARE:
            try:
                self.serial = serial.Serial(port, baud_rate, timeout=1)
                print(f"Successfully connected to hardware on {port}")
            except serial.SerialException as e:
                print(f"Error connecting to hardware: {e}")
                self.serial = None
        
        elif self.mode == ControlMode.SIMULATION:
            # Create communication queues
            self.command_queue = multiprocessing.Queue()
            self.status_queue = multiprocessing.Queue()
            
            # Create pipe for simulation output
            self.sim_output_read, sim_output_write = multiprocessing.Pipe(duplex=False)
            
            # Initialize simulation logger
            self.sim_logger = SimulationLogger()
            
            # Start simulation process with redirected output
            self.sim_process = multiprocessing.Process(
                target=self._run_simulation_with_output,
                args=(self.command_queue, self.status_queue, sim_output_write)
            )
            self.sim_process.start()
            sim_output_write.close()  # Close write end in parent process
            
            # Start log reader thread
            self.log_reader_thread = threading.Thread(
                target=self._read_simulation_output,
                args=(self.sim_output_read,)
            )
            self.log_reader_thread.daemon = True
            self.log_reader_thread.start()
            
            print("Simulation process started")

    def _run_simulation_with_output(self, command_queue, status_queue, output_pipe):
        """Run simulation with output redirection"""
        # Redirect stdout to the pipe
        sys.stdout = PipeWriter(output_pipe)
        try:
            run_simulation(command_queue, status_queue)
        finally:
            output_pipe.close()

    def _read_simulation_output(self, pipe):
        """Read simulation output from pipe and add to log queue"""
        while self.running:
            try:
                if pipe.poll(0.1):  # Check for data with timeout
                    message = pipe.recv()
                    print(message)
                    self.sim_logger.log_queue.put(message)
            except Exception as e:
                print(f"Error reading simulation output: {e}")
                break

    async def start_log_broadcasting(self):
        """Start broadcasting logs to websockets"""
        if self.mode == ControlMode.SIMULATION:
            await self.sim_logger.broadcast_logs(self.active_websockets)
    
    async def process_command(self, cmd_type: int, cmd_data: int):
        if self.mode == ControlMode.HARDWARE:
            if self.serial:
                self.serial.write(bytes([cmd_type, cmd_data]))
        
        elif self.mode == ControlMode.SIMULATION:
            try:
                self.command_queue.put_nowait({
                    'type': cmd_type,
                    'data': cmd_data
                })
            except:
                pass

    async def get_status(self):
        if self.mode == ControlMode.HARDWARE:
            if self.serial:
                if self.serial.read() == b'\xff':
                    status = ord(self.serial.read())
                    if self.serial.read() == b'\xfe':
                        return status
        
        elif self.mode == ControlMode.SIMULATION:
            try:
                return self.status_queue.get_nowait()
            except:
                pass
        
        return 0

    async def broadcast_status(self):
        status = await self.get_status()
        status_data = {
            'limit_x': bool(status & 1),
            'limit_y': bool(status & 2),
            'limit_z': bool(status & 4)
        }
        
        disconnected_sockets = set()
        for websocket in self.active_websockets:
            try:
                await websocket.send_json(status_data)
            except Exception as e:
                print(f"Error broadcasting to client: {e}")
                disconnected_sockets.add(websocket)
        
        self.active_websockets -= disconnected_sockets

    async def status_monitor(self):
        while self.running:
            await self.broadcast_status()
            await asyncio.sleep(0.01)

    def cleanup(self):
        self.running = False
        if self.mode == ControlMode.HARDWARE and self.serial:
            self.serial.close()
        elif self.mode == ControlMode.SIMULATION:
            self.sim_logger.stop()
            self.sim_process.terminate()
            self.sim_process.join()

class PipeWriter:
    """Custom writer to redirect stdout to a pipe"""
    def __init__(self, pipe):
        self.pipe = pipe
        self.line_buffering = True  # Add this to mimic reconfigure

    def write(self, text):
        try:
            if text:  # Only send non-empty text
                self.pipe.send(text)
                if self.line_buffering and text.endswith('\n'):
                    self.flush()
        except Exception as e:
            print(f"Error writing to pipe: {e}")

    def flush(self):
        pass

    def reconfigure(self, *args, **kwargs):
        # Add this method to prevent the attribute error
        if 'line_buffering' in kwargs:
            self.line_buffering = kwargs['line_buffering']
        return self
    

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

controller = MachineController()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    monitor_task = asyncio.create_task(controller.status_monitor())
    if controller.mode == ControlMode.SIMULATION:
        log_task = asyncio.create_task(controller.start_log_broadcasting())
    try:
        yield
    finally:
        # Shutdown logic
        controller.running = False
        monitor_task.cancel()
        if controller.mode == ControlMode.SIMULATION:
            log_task.cancel()

app.router.lifespan_context = lifespan

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    controller.active_websockets.add(websocket)

    try:
        while True:
            message = await websocket.receive_json()

            if message['type'] == 'joystick':
                x_step = 1 if abs(message['x']) > 0.1 else 0
                x_dir = 1 if message['x'] > 0 else 0
                y_step = 1 if abs(message['y']) > 0.1 else 0
                y_dir = 1 if message['y'] > 0 else 0
                z_step = 1 if abs(message['z']) > 0.1 else 0
                z_dir = 1 if message['z'] > 0 else 0
                print(x_step, x_dir, y_step, y_dir, z_step, z_dir)
                cmd_data = (
                    (x_step) |
                    (x_dir << 1) |
                    (y_step << 2) |
                    (y_dir << 3) |
                    (z_step << 4) |
                    (z_dir << 5)
                )
                await controller.process_command(0x01, cmd_data)

            elif message['type'] == 'enable':
                await controller.process_command(0x02, 1 if message['enabled'] else 0)

            elif message['type'] == 'home':
                await controller.process_command(0x03, 1 if message['start'] else 0)

    except:
        controller.active_websockets.remove(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)