import React, { useState, useEffect, useCallback } from 'react';
import { Joystick } from 'react-joystick-component';
import { Switch } from '../components/ui/switch';
import { Button } from '../components/ui/button';
import { Card, CardContent, CardHeader, CardTitle } from '../components/ui/card';
import { Alert, AlertDescription } from '../components/ui/alert';

export default function MachineControl() {
  const [websocket, setWebsocket] = useState(null);
  const [enabled, setEnabled] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const [limits, setLimits] = useState({ limit_x: false, limit_y: false, limit_z: false });
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const [joystickPosition, setJoystickPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    let ws = null;
    let reconnectTimeout = null;

    const connect = () => {
      try {
        ws = new WebSocket('ws://localhost:8000/ws');

        ws.onopen = () => {
          console.log('WebSocket Connected');
          setConnected(true);
          setWebsocket(ws);
          setError(null);
        };

        ws.onclose = (event) => {
          console.log('WebSocket Closed:', event);
          setConnected(false);
          setWebsocket(null);
          reconnectTimeout = setTimeout(connect, 2000);
        };

        ws.onerror = (error) => {
          console.error('WebSocket Error:', error);
          setError('Failed to connect to the machine controller');
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            setLimits(data);
          } catch (e) {
            console.error('Error parsing message:', e);
          }
        };
      } catch (err) {
        console.error('Connection error:', err);
        setError('Failed to connect to the machine controller');
      }
    };

    connect();

    return () => {
      if (ws) ws.close();
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
    };
  }, []);

  const sendJoystickPosition = useCallback(() => {
    if (websocket && websocket.readyState === WebSocket.OPEN && enabled && !isHoming) {
      const { x, y } = joystickPosition;
      if (x !== 0 || y !== 0) {
        console.log(x,y)
        try {
          websocket.send(
            JSON.stringify({
              type: 'joystick',
              x,
              y,
              z: 0,
            })
          );
        } catch (err) {
          console.error('Error sending joystick data:', err);
        }
      }
    }
  }, [websocket, enabled, isHoming, joystickPosition]);

  useEffect(() => {
    const interval = setInterval(() => {
      sendJoystickPosition();
    }, 100); // Sends data every 100ms
    return () => clearInterval(interval);
  }, [sendJoystickPosition]);

  const handleMove = useCallback((event) => {
    setJoystickPosition({ x: event.x, y: event.y });
  }, []);

  const handleStop = useCallback(() => {
    setJoystickPosition({ x: 0, y: 0 });
    if (websocket && websocket.readyState === WebSocket.OPEN) {
      try {
        websocket.send(
          JSON.stringify({
            type: 'joystick',
            x: 0,
            y: 0,
            z: 0,
          })
        );
      } catch (err) {
        console.error('Error sending stop command:', err);
      }
    }
  }, [websocket]);

  const toggleEnable = useCallback(() => {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
      const newState = !enabled;
      setEnabled(newState);
      try {
        websocket.send(
          JSON.stringify({
            type: 'enable',
            enabled: newState,
          })
        );
      } catch (err) {
        console.error('Error toggling enable:', err);
      }
    }
  }, [websocket, enabled]);

  const toggleHoming = useCallback(() => {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
      const newState = !isHoming;
      setIsHoming(newState);
      try {
        websocket.send(
          JSON.stringify({
            type: 'home',
            start: newState,
          })
        );
      } catch (err) {
        console.error('Error toggling homing:', err);
      }
    }
  }, [websocket, isHoming]);

  return (
    <div className="p-4 max-w-4xl mx-auto">
      <Card className="mb-4">
        <CardHeader>
          <CardTitle>Machine Control Panel</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid gap-4">
            {error && (
              <Alert variant="destructive">
                <AlertDescription>{error}</AlertDescription>
              </Alert>
            )}
            {!connected && !error && (
              <Alert>
                <AlertDescription>
                  Attempting to connect to machine controller...
                </AlertDescription>
              </Alert>
            )}
            <div className="flex items-center justify-between">
              <div className="flex items-center space-x-2">
                <Switch checked={enabled} onCheckedChange={toggleEnable} disabled={!connected} />
                <span>Enable Motors</span>
              </div>
              <Button
                onClick={toggleHoming}
                disabled={!connected || !enabled}
                variant={isHoming ? 'destructive' : 'default'}
              >
                {isHoming ? 'Stop Homing' : 'Home Machine'}
              </Button>
            </div>
            <div className="flex justify-center py-8">
              <Joystick
                size={150}
                baseColor="#e2e8f0"
                stickColor="#3b82f6"
                move={handleMove}
                stop={handleStop}
                disabled={!connected || !enabled || isHoming}
              />
            </div>
            <div className="grid grid-cols-3 gap-4 text-center">
              <div className={`p-2 rounded ${limits.limit_x ? 'bg-red-100' : 'bg-green-100'}`}>
                X Limit: {limits.limit_x ? 'Hit' : 'Clear'}
              </div>
              <div className={`p-2 rounded ${limits.limit_y ? 'bg-red-100' : 'bg-green-100'}`}>
                Y Limit: {limits.limit_y ? 'Hit' : 'Clear'}
              </div>
              <div className={`p-2 rounded ${limits.limit_z ? 'bg-red-100' : 'bg-green-100'}`}>
                Z Limit: {limits.limit_z ? 'Hit' : 'Clear'}
              </div>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
