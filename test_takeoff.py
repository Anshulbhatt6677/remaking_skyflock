import asyncio
from mavsdk import System
async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("Connected!")
    try:
        await drone.action.takeoff()
    except Exception as e:
        print(f"Error: {e}")
asyncio.run(run())
