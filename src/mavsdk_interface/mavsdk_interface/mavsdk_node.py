#!/usr/bin/env python3

import asyncio
from mavsdk import System
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class MavsdkTelemetryNode(Node):
    def __init__(self):
        super().__init__('mavsdk_telemetry')
        
        # Initialize telemetry data dictionary
        self.telemetry_data = {
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
        }
        
        # Error tracking system
        self.error_list = []
        self.MAX_ERRORS_DISPLAYED = 5
        
        # ROS Publishers
        self.gps_pub = self.create_publisher(NavSatFix, 'mavsdk/gps', 10)
        self.imu_pub = self.create_publisher(Imu, 'mavsdk/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'mavsdk/battery', 10)
        self.flight_mode_pub = self.create_publisher(String, 'mavsdk/flight_mode', 10)
        self.armed_pub = self.create_publisher(Bool, 'mavsdk/armed', 10)
        self.rc_signal_pub = self.create_publisher(Float64, 'mavsdk/rc_signal', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'mavsdk/diagnostics', 10)
        self.status_pub = self.create_publisher(String, 'mavsdk/status', 10)
        
        # Start the async tasks
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.run())
    
    def add_error(self, error_msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.error_list.append(f"[{timestamp}] {error_msg}")
        if len(self.error_list) > self.MAX_ERRORS_DISPLAYED:
            self.error_list.pop(0)
    
    async def run(self):
        try:
            drone = System()
            await drone.connect(system_address="serial:///dev/ttyACM0:57600")
            self.get_logger().info("Connecting to drone...")

            async for state in drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info("✅ Drone connected!")
                    break

            # Launch all telemetry fetchers
            tasks = [
                asyncio.create_task(self.fetch_position(drone)),
                asyncio.create_task(self.fetch_attitude(drone)),
                asyncio.create_task(self.fetch_battery(drone)),
                asyncio.create_task(self.fetch_gps(drone)),
                asyncio.create_task(self.fetch_flight_mode(drone)),
                asyncio.create_task(self.fetch_armed_status(drone)),
                asyncio.create_task(self.fetch_rc_signal(drone)),
                asyncio.create_task(self.fetch_health(drone)),
                asyncio.create_task(self.publish_loop())
            ]
            
            await asyncio.gather(*tasks)
            
        except Exception as e:
            self.add_error(f"Main connection error: {str(e)}")
            await asyncio.sleep(5)  # Wait before retrying
            await self.run()  # Attempt to reconnect
    
    async def publish_loop(self):
        while True:
            try:
                # Publish GPS data
                gps_msg = NavSatFix()
                try:
                    gps_msg.latitude = float(self.telemetry_data['lat'])
                    gps_msg.longitude = float(self.telemetry_data['lon'])
                    gps_msg.altitude = float(self.telemetry_data['abs_alt'])
                except:
                    pass
                self.gps_pub.publish(gps_msg)
                
                # Publish IMU data
                imu_msg = Imu()
                try:
                    # Note: MAVSDK provides Euler angles, not raw IMU data
                    # You might want to convert these to quaternions for ROS
                    imu_msg.orientation.x = float(self.telemetry_data['roll'])
                    imu_msg.orientation.y = float(self.telemetry_data['pitch'])
                    imu_msg.orientation.z = float(self.telemetry_data['yaw'])
                except:
                    pass
                self.imu_pub.publish(imu_msg)
                
                # Publish battery data
                battery_msg = BatteryState()
                try:
                    battery_msg.voltage = float(self.telemetry_data['voltage'])
                    battery_msg.percentage = float(self.telemetry_data['battery']) / 100.0
                except:
                    pass
                self.battery_pub.publish(battery_msg)
                
                # Publish flight mode
                flight_mode_msg = String()
                flight_mode_msg.data = self.telemetry_data['flight_mode']
                self.flight_mode_pub.publish(flight_mode_msg)
                
                # Publish armed status
                armed_msg = Bool()
                armed_msg.data = self.telemetry_data['armed'] == "Yes"
                self.armed_pub.publish(armed_msg)
                
                # Publish RC signal
                rc_msg = Float64()
                try:
                    rc_msg.data = float(self.telemetry_data['rc_signal'])
                except:
                    pass
                self.rc_signal_pub.publish(rc_msg)
                
                # Publish diagnostics
                diag_msg = DiagnosticArray()
                status = DiagnosticStatus()
                status.name = "MAVSDK Telemetry"
                
                # Add health checks
                for key, value in self.telemetry_data["health"].items():
                    status.values.append(KeyValue(key=key, value=str(value)))
                
                # Add other telemetry
                for key, value in self.telemetry_data.items():
                    if key != "health":
                        status.values.append(KeyValue(key=key, value=str(value)))
                
                # Add errors if any
                if self.error_list:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f"{len(self.error_list)} errors reported"
                    for error in self.error_list:
                        status.values.append(KeyValue(key="Error", value=error))
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "All systems normal"
                
                diag_msg.status.append(status)
                self.diagnostics_pub.publish(diag_msg)
                
                # Publish status message (for debugging)
                status_msg = String()
                status_msg.data = f"GPS: {self.telemetry_data['gps_fix']}, Battery: {self.telemetry_data['battery']}%"
                self.status_pub.publish(status_msg)
                
                await asyncio.sleep(1)
            except Exception as e:
                self.add_error(f"Publish loop error: {str(e)}")
                await asyncio.sleep(1)
    
    async def fetch_position(self, drone):
        while True:
            try:
                async for pos in drone.telemetry.position():
                    self.telemetry_data["lat"] = f"{pos.latitude_deg:.6f}"
                    self.telemetry_data["lon"] = f"{pos.longitude_deg:.6f}"
                    self.telemetry_data["alt"] = f"{pos.relative_altitude_m:.2f}"
                    self.telemetry_data["abs_alt"] = f"{pos.absolute_altitude_m:.2f}"
            except Exception as e:
                self.add_error(f"Position fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_attitude(self, drone):
        while True:
            try:
                async for att in drone.telemetry.attitude_euler():
                    self.telemetry_data["roll"] = f"{att.roll_deg:.2f}"
                    self.telemetry_data["pitch"] = f"{att.pitch_deg:.2f}"
                    self.telemetry_data["yaw"] = f"{att.yaw_deg:.2f}"
            except Exception as e:
                self.add_error(f"Attitude fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_battery(self, drone):
        while True:
            try:
                async for batt in drone.telemetry.battery():
                    self.telemetry_data["voltage"] = f"{batt.voltage_v:.2f}"
                    self.telemetry_data["battery"] = f"{batt.remaining_percent * 100:.1f}"
            except Exception as e:
                self.add_error(f"Battery fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_gps(self, drone):
        while True:
            try:
                async for gps in drone.telemetry.gps_info():
                    self.telemetry_data["gps_fix"] = str(gps.fix_type).replace("FIX_TYPE_", "")
                    self.telemetry_data["satellites"] = f"{gps.num_satellites}"
            except Exception as e:
                self.add_error(f"GPS fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_flight_mode(self, drone):
        while True:
            try:
                async for mode in drone.telemetry.flight_mode():
                    self.telemetry_data["flight_mode"] = str(mode).replace("FLIGHT_MODE_", "")
            except Exception as e:
                self.add_error(f"Flight mode fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_armed_status(self, drone):
        while True:
            try:
                async for armed in drone.telemetry.armed():
                    self.telemetry_data["armed"] = "Yes" if armed else "No"
            except Exception as e:
                self.add_error(f"Armed status fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_rc_signal(self, drone):
        while True:
            try:
                async for rc in drone.telemetry.rc_status():
                    try:
                        self.telemetry_data["rc_signal"] = f"{rc.signal_strength_percent:.1f}"
                    except:
                        self.telemetry_data["rc_signal"] = "N/A"
            except Exception as e:
                self.add_error(f"RC signal fetch error: {str(e)}")
                await asyncio.sleep(1)

    async def fetch_health(self, drone):
        while True:
            try:
                async for health in drone.telemetry.health():
                    self.telemetry_data["health"] = {
                        "Accelerometer calibration": "OK" if health.is_accelerometer_calibration_ok else "FAIL",
                        "Gyrometer calibration"   : "OK" if health.is_gyrometer_calibration_ok else "FAIL",
                        "Magnetometer calibration": "OK" if health.is_magnetometer_calibration_ok else "FAIL",
                        "Global position"         : "OK" if health.is_global_position_ok else "FAIL",
                        "Home position"           : "OK" if health.is_home_position_ok else "FAIL",
                        "Local position"          : "OK" if health.is_local_position_ok else "FAIL",
                        "Armable"                 : "OK" if health.is_armable else "FAIL"
                    }
            except Exception as e:
                self.add_error(f"Health check fetch error: {str(e)}")
                await asyncio.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MavsdkTelemetryNode()

    try:
        # Run the asyncio tasks in a separate thread
        loop = asyncio.get_event_loop()
        loop.run_until_complete(node.run())
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()