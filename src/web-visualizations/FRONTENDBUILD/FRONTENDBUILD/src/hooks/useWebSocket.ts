// src/hooks/useWebSocket.ts
import { useState, useEffect, useCallback } from 'react';

interface DroneData {
  drones: {
    [key: string]: {
      position: number[];
      heading: number;
      conn: boolean;
      armed: boolean;
      in_air: boolean;
    };
  };
  timestamp: number;
}

export const useWebSocket = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [droneData, setDroneData] = useState<DroneData | null>(null);
  const [ws, setWs] = useState<WebSocket | null>(null);

  const connect = useCallback(() => {
    try {
     // const websocket = new WebSocket('ws://localhost:8765'); //etheret
       const websocket = new WebSocket('ws://M4001-23-01.local:8765');
      
      websocket.onopen = () => {
        console.log('WebSocket Connected');
        setIsConnected(true);
      };

      websocket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          setDroneData(data);
        //   console.log('Received data:', data);

        // can you please open the chrome here ???
        } catch (error) {
          console.error('Error parsing WebSocket data:', error);
        }
      };

      websocket.onclose = () => {
        console.log('WebSocket Disconnected - Attempting to reconnect...');
        setIsConnected(false);
        setWs(null);
        // Attempt to reconnect after 2 seconds
        setTimeout(connect, 2000);
      };

      websocket.onerror = (error) => {
        console.error('WebSocket Error:', error);
        websocket.close();
      };

      setWs(websocket);
    } catch (error) {
      console.error('Connection error:', error);
      setTimeout(connect, 2000);
    }
  }, []);

  useEffect(() => {
    connect();
    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [connect]);

  const sendMessage = useCallback((message: any) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }, [ws]);

  return { isConnected, droneData, sendMessage };
};