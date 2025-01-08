import asyncio
import base64
import mujoco
import io
import numpy as np
from PIL import Image
import websockets

# MuJoCo XML model definition
xml = """
<mujoco>
  <visual>
    <global offwidth="1920" offheight="1080"/>
  </visual>
  <worldbody>
    <body name="smallbox">
      <joint type="free" damping="0.0005"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# Create MuJoCo model and data
model = mujoco.MjModel.from_xml_path("briekiebot.xml")
data = mujoco.MjData(model)

# Movement variables
cube_pos = [0, 0]  # [z, y] position of the cube
speed = 0.01  # Movement speed


async def simulate():
    """
    Simulate the model at 2 kHz (2000 steps per second).
    """
    timestep = model.opt.timestep  # Simulation timestep
    interval = timestep * 1000  # Convert to milliseconds
    while True:
        # Step the simulation
        mujoco.mj_step(model, data)

        # Wait for the next simulation step
        await asyncio.sleep(interval / 1000)  # Convert milliseconds to seconds


async def render_frames(websocket, renderer, fps=24):
    """
    Send frames to the WebSocket at 24 FPS.
    """
    frame_interval = 1 / fps
    while True:
        # Render the scene
        renderer.update_scene(data)
        rendered_image = renderer.render()

        # Convert rendered image to base64-encoded string
        buffer = io.BytesIO()
        Image.fromarray(rendered_image).save(buffer, format="JPEG")
        encoded_frame = base64.b64encode(buffer.getvalue()).decode("utf-8")

        # Send the frame
        await websocket.send(encoded_frame)

        # Wait for the next frame
        await asyncio.sleep(frame_interval)


async def handle_websocket(websocket):
    """
    Handle WebSocket commands to move the cube.
    """
    global cube_pos
    while True:
        try:
            # Wait for key input from the WebSocket
            key = await websocket.recv()
            if key == "up":
                cube_pos[1] += speed  # Move up (increase y)
            elif key == "down":
                cube_pos[1] -= speed  # Move down (decrease y)
            elif key == "left":
                cube_pos[0] -= speed  # Move left (decrease z)
            elif key == "right":
                cube_pos[0] += speed  # Move right (increase z)

            # Update cube position in simulation
            data.geom_xpos[0][1] = cube_pos[1]
            data.geom_xpos[0][2] = cube_pos[0]

        except websockets.ConnectionClosed:
            break


async def send_frames(websocket, path):
    """
    Combine simulation, rendering, and WebSocket handling.
    """
    with mujoco.Renderer(model, width=1920, height=1080) as renderer:
        # Start simulation task at 2 kHz
        simulation_task = asyncio.create_task(simulate())

        # Start rendering frames at 24 FPS
        rendering_task = asyncio.create_task(render_frames(websocket, renderer))

        # Handle WebSocket commands
        await handle_websocket(websocket)

        # Cancel tasks when WebSocket closes
        simulation_task.cancel()
        rendering_task.cancel()
        try:
            await simulation_task
            await rendering_task
        except asyncio.CancelledError:
            pass


# Start the WebSocket server
start_server = websockets.serve(send_frames, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
