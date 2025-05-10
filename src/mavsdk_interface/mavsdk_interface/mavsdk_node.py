import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String, Int32 # type: ignore
from mavsdk import System

import asyncio
import threading

class MavsdkNode(Node):
    def __init__(self):
        super().__init__('mavsdk_node')
        self.declare_parameter('system_address', 'serial:///dev/ttyACM0:57600')
        
        # Publisher for connection status
        self.connection_publisher = self.create_publisher(String, 'connection_status', 10)
        connection_msg = String()
        connection_msg.data = "Initializing mavsdk node"
        self.connection_publisher.publish(connection_msg)
        
        # Publisher for uptime
        self.uptime_publisher = self.create_publisher(Int32, 'uptime', 10)
        self.uptime_counter = 0
        
        # Timer for publishing incrementing numbers
        self.create_timer(1.0, self.publish_uptime_counter)
        
        self.system_address = self.get_parameter('system_address').get_parameter_value().string_value
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        # Run async connection in background thread
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        # asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_and_publish())

    async def connect_and_publish(self):
        print("Starting connection process...")

        print(f"Attempting to connect to drone at {self.system_address}")
        connection_msg = String()
        connection_msg.data = "Connecting to drone..."
        self.connection_publisher.publish(connection_msg)

        await self.drone.connect(system_address=self.system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected.")
                connection_msg.data = "Drone connected"
                self.connection_publisher.publish(connection_msg)
                break
            else:
                if not state.is_connected:
                    print("Drone not connected.")
                    connection_msg.data = "Drone not connected."
                    self.connection_publisher.publish(connection_msg)

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
