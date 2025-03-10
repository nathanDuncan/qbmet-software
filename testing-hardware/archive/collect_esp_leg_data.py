import asyncio
import websockets

async def handle_connection(websocket):
    print("Client connected...")
    try:
        async for message in websocket:
            print(f"Received data:\n{message}")
            with open("sensor_data.txt", "a") as f:
                f.write(message + "\n")
            print("Data saved.")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")

async def main():
    async with websockets.serve(handle_connection, "192.168.5.40", "1234"):
        print(f"WebSocket server running on port <port>...")
        await asyncio.Future() # Run forever

if __name__ == "__main__":
    asyncio.run(main())
