import asyncio
from mavsdk import System

async def consume1(drone):
    print("Consume 1 started")
    async for health in drone.telemetry.health():
        print(f"Consume 1: {health}")
        await asyncio.sleep(0.5)

async def consume2(drone):
    print("Consume 2 started")
    async for health in drone.telemetry.health():
        print(f"Consume 2: {health}")
        await asyncio.sleep(0.5)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break
    
    # Run both consumers
    task1 = asyncio.create_task(consume1(drone))
    task2 = asyncio.create_task(consume2(drone))
    
    await asyncio.sleep(2)
    task1.cancel()
    task2.cancel()
    print("Done")

asyncio.run(run())
