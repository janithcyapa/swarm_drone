import asyncio
from mavsdk import System
import os

telemetry_data = {
    "lat": "N/A", "lon": "N/A", "alt": "N/A", "abs_alt": "N/A",
    "speed": "N/A", "roll": "N/A", "pitch": "N/A", "yaw": "N/A",
    "voltage": "N/A", "battery": "N/A",
    "gps_fix": "N/A", "satellites": "N/A",
    "flight_mode": "N/A", "armed": "N/A",
    "rc_signal": "N/A",
    "health": {
        "Accelerometer calibration": "N/A",
        "Armable": "N/A",
        "Global position": "N/A",
        "Gyrometer calibration": "N/A",
        "Home position": "N/A",
        "Local position": "N/A",
        "Magnetometer calibration": "N/A"
    },
    "mission": {
        "exists": False,
        "current": "N/A",
        "total": "N/A",
        "finished": "N/A"
    }
}

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")
    print("Connecting to drone...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected!")
            
            break

    # Launch all telemetry fetchers
    tasks = [
        fetch_position(drone),
        fetch_attitude(drone),
        fetch_battery(drone),
        fetch_gps(drone),
        fetch_flight_mode(drone),
        fetch_armed_status(drone),
        # fetch_ground_speed(drone),
        fetch_rc_signal(drone),
        fetch_health(drone),
        fetch_mission_progress(drone),
        display_loop()
    ]
    await asyncio.gather(*[asyncio.create_task(task) for task in tasks])

async def display_loop():
    while True:
        os.system("clear")  # Use 'cls' on Windows
        print("========= PX4 MAVSDK Telemetry =========")
        print(f"GPS Fix        : {telemetry_data['gps_fix']}")
        print(f"Satellites     : {telemetry_data['satellites']}")
        print(f"Latitude       : {telemetry_data['lat']}")
        print(f"Longitude      : {telemetry_data['lon']}")
        print(f"Rel Alt (m)    : {telemetry_data['alt']}")
        print(f"Abs Alt (m)    : {telemetry_data['abs_alt']}")
        # print(f"Ground Speed   : {telemetry_data['speed']} m/s")
        print(f"Roll           : {telemetry_data['roll']}°")
        print(f"Pitch          : {telemetry_data['pitch']}°")
        print(f"Yaw            : {telemetry_data['yaw']}°")
        print(f"Voltage        : {telemetry_data['voltage']} V")
        print(f"Battery        : {telemetry_data['battery']} %")
        print(f"Flight Mode    : {telemetry_data['flight_mode']}")
        print(f"Armed          : {telemetry_data['armed']}")
        print(f"RC Signal      : {telemetry_data['rc_signal']} %")

        print("\n---------- Pre-Arm Health Check ---------")
        for key, value in telemetry_data["health"].items():
            print(f"{key:<30}: {value}")

        print("\n------------ Mission Info ---------------")
        if telemetry_data["mission"]["exists"]:
            print(f"Current Item   : {telemetry_data['mission']['current']}")
            print(f"Total Items    : {telemetry_data['mission']['total']}")
            print(f"Finished       : {'Yes' if telemetry_data['mission']['finished'] else 'No'}")
        else:
            print("No Mission Uploaded.")
        print("=========================================\n")
        await asyncio.sleep(1)

# Individual fetch functions
async def fetch_position(drone):
    async for pos in drone.telemetry.position():
        telemetry_data["lat"] = f"{pos.latitude_deg:.6f}"
        telemetry_data["lon"] = f"{pos.longitude_deg:.6f}"
        telemetry_data["alt"] = f"{pos.relative_altitude_m:.2f}"
        telemetry_data["abs_alt"] = f"{pos.absolute_altitude_m:.2f}"

async def fetch_attitude(drone):
    async for att in drone.telemetry.attitude_euler():
        telemetry_data["roll"] = f"{att.roll_deg:.2f}"
        telemetry_data["pitch"] = f"{att.pitch_deg:.2f}"
        telemetry_data["yaw"] = f"{att.yaw_deg:.2f}"

async def fetch_battery(drone):
    async for batt in drone.telemetry.battery():
        telemetry_data["voltage"] = f"{batt.voltage_v:.2f}"
        telemetry_data["battery"] = f"{batt.remaining_percent * 100:.1f}"

async def fetch_gps(drone):
    async for gps in drone.telemetry.gps_info():
        telemetry_data["gps_fix"] = str(gps.fix_type).replace("FIX_TYPE_", "")
        telemetry_data["satellites"] = f"{gps.num_satellites}"

async def fetch_flight_mode(drone):
    async for mode in drone.telemetry.flight_mode():
        telemetry_data["flight_mode"] = str(mode).replace("FLIGHT_MODE_", "")

async def fetch_armed_status(drone):
    async for armed in drone.telemetry.armed():
        telemetry_data["armed"] = "Yes" if armed else "No"

# async def fetch_ground_speed(drone):
#     async for speed in drone.telemetry.ground_speed():
#         telemetry_data["speed"] = f"{speed:.2f}"

async def fetch_rc_signal(drone):
    async for rc in drone.telemetry.rc_status():
        try:
            telemetry_data["rc_signal"] = f"{rc.signal_strength_percent:.1f}"
        except:
            telemetry_data["rc_signal"] = "N/A"

async def fetch_health(drone):
    async for health in drone.telemetry.health():
        telemetry_data["health"] = {
            "Accelerometer calibration": "OK" if health.is_accelerometer_calibration_ok else "FAIL",
            "Gyrometer calibration"   : "OK" if health.is_gyrometer_calibration_ok else "FAIL",
            "Magnetometer calibration": "OK" if health.is_magnetometer_calibration_ok else "FAIL",
            "Global position"         : "OK" if health.is_global_position_ok else "FAIL",
            "Home position"           : "OK" if health.is_home_position_ok else "FAIL",
            "Local position"          : "OK" if health.is_local_position_ok else "FAIL",
            "Armable"                 : "OK" if health.is_armable else "FAIL"
        }

async def fetch_mission_progress(drone):
    try:
        mission_items = await drone.mission.mission_items()
        telemetry_data["mission"]["total"] = len(mission_items)
        telemetry_data["mission"]["exists"] = True
    except:
        telemetry_data["mission"]["exists"] = False

    async for progress in drone.mission.mission_progress():
        telemetry_data["mission"]["current"] = progress.current
        telemetry_data["mission"]["finished"] = (progress.current == progress.total)

if __name__ == "__main__":
    asyncio.run(run())
