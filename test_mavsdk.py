import asyncio
from mavsdk import System
async def run():
    s = System()
    print("Action methods:", [m for m in dir(s.action) if not m.startswith('_')])
asyncio.run(run())
