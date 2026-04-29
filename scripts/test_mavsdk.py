#!/usr/bin/env python3
"""Smoke test for the MAVSDK -> PX4 SITL link.

Connects, waits for the connection state, then prints a single
armed / in-air / GPS-position sample. If all four prints succeed the
link is healthy and Phase 5 can proceed.

Run while PX4 SITL is up at the pxh> prompt:
    docker exec -it px4_sitl python3 /root/3d-path-planner/scripts/test_mavsdk.py
"""

import asyncio

from mavsdk import System


async def main() -> None:
    drone = System()
    print('Connecting to PX4 SITL on udp://:14540 ...')
    await drone.connect(system_address='udp://:14540')

    async for state in drone.core.connection_state():
        if state.is_connected:
            print('Connected.')
            break

    async for is_armed in drone.telemetry.armed():
        print(f'Armed:  {is_armed}')
        break

    async for in_air in drone.telemetry.in_air():
        print(f'In air: {in_air}')
        break

    async for pos in drone.telemetry.position():
        print(
            f'Position: lat={pos.latitude_deg:.6f}  lon={pos.longitude_deg:.6f}  '
            f'alt_abs={pos.absolute_altitude_m:.2f} m  alt_rel={pos.relative_altitude_m:.2f} m'
        )
        break


if __name__ == '__main__':
    asyncio.run(main())
