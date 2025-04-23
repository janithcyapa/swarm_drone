import asyncio
from mavsdk import System
from mavsdk.telemetry import Health

async def connect_to_pixhawk(connection_str="serial:///dev/ttyACM0:57600"):
    drone = System()
    await drone.connect(system_address=connection_str)

    print(f"🔌 Connecting to Pixhawk at {connection_str}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ Connected to drone! UUID: {state}")
            break
    return drone

async def pre_arm_checks(drone: System):
    print("\n🛠️ Running pre-arm checks...\n")

    async for health in drone.telemetry.health():
        print(f"🛰️  GPS Fix: {'✅' if health.is_global_position_ok else '❌'}")
        print(f"📍 Home Position Set: {'✅' if health.is_home_position_ok else '❌'}")
        print(f"🧭 Gyro Calibration: {'✅' if health.is_gyrometer_calibration_ok else '❌'}")
        print(f"🧭 Accel Calibration: {'✅' if health.is_accelerometer_calibration_ok else '❌'}")
        print(f"🧭 Mag Calibration: {'✅' if health.is_magnetometer_calibration_ok else '❌'}")
        print(f"📡 Local Position OK: {'✅' if health.is_local_position_ok else '❌'}")
        print(f"⚙️  Armable: {'✅' if health.is_armable else '❌'}")

        if health.is_armable:
            print("\n🚀 Drone is ready to arm!\n")
        else:
            print("\n⏳ Not ready to arm yet. Waiting...\n")
        break

# Example usage
async def main():
    drone = await connect_to_pixhawk()
    await pre_arm_checks(drone)

if __name__ == "__main__":
    asyncio.run(main())
