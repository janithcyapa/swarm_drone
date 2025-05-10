import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String, Int32, Float32MultiArray # type: ignore
from mavsdk import System

import asyncio
import threading


class MavsdkNode(Node):
    def __init__(self, **kwargs):
        super().__init__('mavsdk_node', **kwargs)
        self.declare_parameter('system_address', 'serial:///dev/ttyACM0:57600')
        
        # Publisher for connection status
        self.connection_publisher = self.create_publisher(String, 'connection_status', 10)
        connection_msg = String()
        connection_msg.data = "Initializing mavsdk node"
        self.connection_publisher.publish(connection_msg)
        
        # Publisher for uptime
        self.uptime_publisher = self.create_publisher(Int32, 'uptime', 10)
        self.uptime_counter = 0
        
        # Publisher for battery status
        self.battery_publisher = self.create_publisher(Float32MultiArray, 'battery', 10)
        
        # Timer for publishing incrementing numbers
        self.create_timer(1.0, self.publish_uptime_counter)
        
        self.system_address = self.get_parameter('system_address').get_parameter_value().string_value
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        # Run async connection in background thread
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        self.loop.run_until_complete(self.connect_and_publish())

    async def connect_and_publish(self):
        self.get_logger().info("Starting connection process...")

        self.get_logger().info(f"Attempting to connect to drone at {self.system_address}")
        connection_msg = String()
        connection_msg.data = "Connecting to drone..."
        self.connection_publisher.publish(connection_msg)

        await self.drone.connect(system_address=self.system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected.")
                connection_msg.data = "Drone connected"
                self.connection_publisher.publish(connection_msg)
                # Start publishing battery data
                asyncio.create_task(self.publish_battery_status())
                break
            else:
                if not state.is_connected:
                    self.get_logger().info("Drone not connected.")
                    connection_msg.data = "Drone not connected."
                    self.connection_publisher.publish(connection_msg)

    async def publish_battery_status(self):
        # try:
            self.get_logger().info("Battery fetching")
            async for pos in self.drone.core.connection_state():
                self.get_logger().info("Battery publishing")
                # battery_msg = Float32MultiArray()
                # battery_msg.data = [battery.voltage_v, battery.remaining_percent * 100]
                # self.battery_publisher.publish(battery_msg)
        # except Exception as e:
        #     self.get_logger().error(f"Battery fetch error: {e}")

    def publish_uptime_counter(self):
        msg = Int32()
        msg.data = self.uptime_counter
        self.uptime_publisher.publish(msg)
        self.uptime_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MavsdkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
