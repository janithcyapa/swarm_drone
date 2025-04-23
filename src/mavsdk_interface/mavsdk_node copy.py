import asyncio
from mavsdk import System
import os
import math

async def show_all_telemetry(drone):
    os.system('clear')  # or 'cls' on Windows
    print("========= PX4 MAVSDK Telemetry =========")

    async for position in drone.telemetry.position():
        print(f"Latitude        : {position.latitude_deg}")
        print(f"Longitude       : {position.longitude_deg}")
        print(f"Rel Alt (m)     : {position.relative_altitude_m}")
        print(f"Abs Alt (m)     : {position.absolute_altitude_m}")
        break

    async for gps in drone.telemetry.gps_info():
        print(f"GPS Fix         : {gps.fix_type}")
        print(f"Satellites      : {gps.num_satellites}")
        break


    async for att in drone.telemetry.attitude_euler():
        print(f"Roll            : {att.roll_deg:.2f}°")
        print(f"Pitch           : {att.pitch_deg:.2f}°")
        print(f"Yaw             : {att.yaw_deg:.2f}°")
        break

    async for battery in drone.telemetry.battery():
        print(f"Voltage         : {battery.voltage_v:.2f} V")
        print(f"Battery         : {battery.remaining_percent * 100:.1f} %")
        break

    async for flight_mode in drone.telemetry.flight_mode():
        print(f"Flight Mode     : {flight_mode}")
        break

    async for armed in drone.telemetry.armed():
        print(f"Armed           : {armed}")
        break

    async for rc in drone.telemetry.rc_status():
        if rc.signal_strength_percent != rc.signal_strength_percent:  # NaN check
            print("RC Signal       :  No RC Signal detected.")
        else:
            print(f"RC Signal       : {rc.signal_strength_percent:.1f} %")
        break

    print("---------- Pre-Arm Health Check ---------")
    async for health in drone.telemetry.health():
        health_data = {
            "Accelerometer Calibration": health.is_accelerometer_calibration_ok,
            "Gyrometer Calibration"   : health.is_gyrometer_calibration_ok,
            "Magnetometer Calibration": health.is_magnetometer_calibration_ok,
            "Global Position OK"      : health.is_global_position_ok,
            "Home Position OK"        : health.is_home_position_ok,
            "Local Position OK"       : health.is_local_position_ok,
            "Armable"                 : health.is_armable,
        }
        for check, result in health_data.items():
            print(f"{check:<25}: {result}")
        break

    print("=========================================")


async def run():
    drone = System()
    print("Connecting to the drone on /dev/ttyACM0...")
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected!")
            break

    await show_all_telemetry(drone)


if __name__ == "__main__":
    asyncio.run(run())
