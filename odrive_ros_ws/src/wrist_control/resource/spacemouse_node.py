import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from .pyspacemouse import open, print_buttons, print_state, ButtonCallback
import time
import numpy as np
from transforms3d.euler import euler2quat
from platform_interfaces.srv import SwitchTopic


class DeviceNode(Node):
    def __init__(self):
        super().__init__('spacemouse_node')
        self.publisher_ = self.create_publisher(Pose, 'spacemouse_pose', 100)
        self.timer = self.create_timer(0.01, self.publish_state)  # Adjust the publish rate as needed
        self.state = None  # Initialize state here

        # Define your button callbacks
        # For simplicity, I'm using print statements instead of actual callbacks
        button_arr = [ButtonCallback(0, self.button_0),
                  ButtonCallback([1], lambda state, buttons, pressed_buttons: print("Button: 1")),
                  ButtonCallback([0, 1], self.button_0_1), ]

        # Initialize your device here
        # For now, I'm using a placeholder for 'dev'
        self.dev = open()
        self.get_logger().info(self.dev.describe_connection())
        self.old_buttons = [0,0] #Assume we start with buttons not pressed. 

        self.mux_client = self.create_client(SwitchTopic, 'switch_topic')

    def publish_state(self):
        # Continuously publish data from the state variable
        msg = self.state
        self.publisher_.publish(msg)

    def button_0(self, state, buttons, pressed_buttons):
        print("Button 0")

    def button_0_1(self, state, buttons, pressed_buttons):
        print("Button 0 and 1")

    def read_device(self):
        return self.dev.read()

    def run_device(self):
        # Continuously read data from the device and update state
        while rclpy.ok():
            tic = time.time()
            for i in range(50):
                self.state = self.read_device()
            print(self.state)
            # print("STATES?", self.state.x, self.state.y, self.state.z, self.state.roll, self.state.pitch, self.state.yaw)

            #Service the buttons (for mux switching)
            if self.state.buttons[0] == 1 and self.old_buttons[0] == 0:
                request = SwitchTopic.Request()
                self.mux_client.call_async(request)
            msg = Pose()
            
            msg.position.x = float(self.state.x)
            msg.position.y = float(self.state.y)
            msg.position.z = float(self.state.z)
            
            quat = euler2quat(self.state.roll, self.state.pitch, self.state.yaw)

            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]
            
            self.state = msg
            toc = time.time()
            self.get_logger().info("{:7.6f}".format(toc - tic))
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = DeviceNode()
    node.run_device()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
