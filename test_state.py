import asyncio
from mavsdk import System

async def run():
    drone = System(port=50051)
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("Connected")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
            
    async for armed in drone.telemetry.armed():
        print(f"Armed: {armed}")
        break
        
    async for mode in drone.telemetry.flight_mode():
        print(f"Flight Mode: {mode}")
        break
        
    async for landed_state in drone.telemetry.landed_state():
        print(f"Landed State: {landed_state}")
        break
        
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        break

asyncio.run(run())
