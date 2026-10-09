import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from .pyspacemouse import open, print_buttons, print_state, ButtonCallback
from platform_interfaces.msg import Command, ServoAngles
import numpy as np
from transforms3d.euler import quat2euler

'''
CREDIT GOES HERE:

This code utilizes the following source:
https://github.com/Yeok-c/Stewart_Py
MIT Licence

'''



class DynamicsNode(Node):
    def __init__(self):
        super().__init__('rigid_dynamics_node')
        
        # self.command_sub = self.create_subscription(
            # Command, --> Message Type
            # 'servo_commands', --> Topic
            # self.command_callback, --> Callback
            # 10 --> Frequency
        # )
        
        self.publisher_ = self.create_publisher(ServoAngles, 'servo_angles', 100)
        self.subscriber = self.create_subscription(Pose, 'command_pose', self.rigid_stewart_dynamics, 100)
        
        self.timer = self.create_timer(0.01, self.publish_angles)  # Adjust the publish rate as needed
        
        
        # These are in cm
        self.platform_radius_top = 7.874
        self.platform_radius_bottom = 11.28616996
        self.top_to_bottom = 6.1341
        self.static_length = 6.814
        
        # These are in radians
        self.gamma_B = 0.3639382215786 # half angle between adjacent legs on the base
        self.gamma_P = 0.3818744117959 # half angle between adjacent legs on the platform
        self.max_rot = np.pi/5

        
        # This is in cm
        self.max_lin = 10
        
        self.servo_angles = np.zeros(6)
        
    def rotX(self, theta):
        rotx = np.array([
            [1,     0    ,    0    ],
            [0,  np.cos(theta), -np.sin(theta)],
            [0,  np.sin(theta), np.cos(theta)] ])
        return rotx
    
    
    '''
    [1 0 0 0]
    [0 1 0 0]
    [0 0 1 0]
    [0 0 0 1]
    '''

    def rotY(self, theta):    
        roty = np.array([
            [np.cos(theta), 0,  np.sin(theta) ],
            [0         , 1,     0       ],
            [-np.sin(theta), 0,  np.cos(theta) ] ])   
        return roty
        
    def rotZ(self, theta):    
        rotz = np.array([
            [ np.cos(theta),-np.sin(theta), 0 ],
            [ np.sin(theta), np.cos(theta), 0 ],
            [   0        ,     0      , 1 ] ])   
        return rotz
        
    def rigid_stewart_dynamics(self, msg):
        
        pi = np.pi

        ## Define the Geometry of the platform
        # Psi_B (Polar coordinates)
        psi_B = np.array([ 
            -self.gamma_B, 
            self.gamma_B,
            2*pi/3 - self.gamma_B, 
            2*pi/3 + self.gamma_B, 
            2*pi/3 + 2*pi/3 - self.gamma_B, 
            2*pi/3 + 2*pi/3 + self.gamma_B])

        # psi_P (Polar coordinates)
        # Direction of the points where the rod is attached to the platform.
        psi_P = np.array([ 
            pi/3 + 2*pi/3 + 2*pi/3 + self.gamma_P,
            pi/3 + -self.gamma_P, 
            pi/3 + self.gamma_P,
            pi/3 + 2*pi/3 - self.gamma_P, 
            pi/3 + 2*pi/3 + self.gamma_P, 
            pi/3 + 2*pi/3 + 2*pi/3 - self.gamma_P])

        # Coordinate of the points where servo arms 
        # are attached to the corresponding servo axis.
        B = self.platform_radius_bottom * np.array( [ 
            [ np.cos(psi_B[0]), np.sin(psi_B[0]), 0],
            [ np.cos(psi_B[1]), np.sin(psi_B[1]), 0],
            [ np.cos(psi_B[2]), np.sin(psi_B[2]), 0],
            [ np.cos(psi_B[3]), np.sin(psi_B[3]), 0],
            [ np.cos(psi_B[4]), np.sin(psi_B[4]), 0],
            [ np.cos(psi_B[5]), np.sin(psi_B[5]), 0] ])
        B = np.transpose(B)
            
        # Coordinates of the points where the rods 
        # are attached to the platform.
        P = self.platform_radius_top * np.array([ 
            [ np.cos(psi_P[0]),  np.sin(psi_P[0]), 0],
            [ np.cos(psi_P[1]),  np.sin(psi_P[1]), 0],
            [ np.cos(psi_P[2]),  np.sin(psi_P[2]), 0],
            [ np.cos(psi_P[3]),  np.sin(psi_P[3]), 0],
            [ np.cos(psi_P[4]),  np.sin(psi_P[4]), 0],
            [ np.cos(psi_P[5]),  np.sin(psi_P[5]), 0] ])
        P = np.transpose(P)

        # print('6x3 array for Base anchors \n', B)
        # print('6x1 array for psi_B \n', psi_B)
        
        # print('6x3 array for Platform anchors \n', P)
        # print('6x1 array for psi_P \n', psi_P)

        # # Definition of the platform home position.

        # Here we use the base length of our linear actuators, somehow
        home_pos= np.array([0, 0, self.top_to_bottom])

        # Allocate for variables
        l = np.zeros((3,6))
        lll = np.zeros((6))
        
        # Angles by the spacemouse are published in [-1,1]. So we scale these appropriately.
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        r, p, y = quat2euler(quat)
        roll = (r* self.max_rot * -1)
        pitch = (p * self.max_rot * -1)
        yaw = (y* self.max_rot)
        
        x_trans = msg.position.x * self.max_lin * -1
        y_trans = msg.position.y * self.max_lin
        z_trans = msg.position.z * self.max_lin
        
        # Given input trans, rotation
        # trans = np.transpose(np.array([y_trans, x_trans, z_trans])) # X, Y, Z
        trans = np.transpose(np.array([0, 0, z_trans])) # X, Y, Z
        rotation = np.transpose(np.array([roll, pitch, yaw])) # R, P, Y
        
        # trans = np.transpose(np.array([0, 0, z_trans])) # X, Y, Z
        # rotation = np.transpose(np.array([0,0,0])) # R, P, Y

        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        
        # TODO: Test both version of this. Which one makes sense?
        # R = np.matmul( np.matmul(rotZ(rotation[2]), rotY(rotation[1])), rotX(rotation[0]) )
        R = np.matmul( np.matmul(self.rotX(rotation[0]), self.rotY(rotation[1])), self.rotZ(rotation[2]) )

        # Get leg length for each leg
        # leg = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(home_pos[:, np.newaxis], 6, axis=1) + np.matmul(np.transpose(R), P) - B 
        l = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(home_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, P) - B 
        lll = np.linalg.norm(l, axis=0)

        # Position of leg in global frame
        L = l + B
        
        # Equation for servo rotation angles:
        # lll - 22.467 (verify) / (1.13) = servo_angle in radians
        
        # Here we convert the leg lengths to servo angles
        length_difference = lll - self.static_length
        motor_angles_radians = length_difference / 1.23
        motor_angles_degrees = np.around(np.degrees(motor_angles_radians), decimals=2)
        
        # DEBUG
        
        
        # This takes all of the motor values and shifts them such that the minimum is zero
        # motor_angles_degrees += 45
        motor_angles_degrees += -1*min(motor_angles_degrees) if min(motor_angles_degrees) < 0 else 0
        
        self.servo_angles = motor_angles_degrees
        # 
        # self.servo_angles = np.clip(motor_angles_degrees, -40, 300)

        # print('Leg lengths to command in order to achieve desired position of plate: \n', lll)
        # print('Motor angles in degrees: \n', motor_angles_degrees)
        # print('Leg positions in global frame: \n', L)
        

    def publish_angles(self):
        # Continuously publish data from the state variable

        self.get_logger().fatal('ITS DOOMED: {}'.format(self.servo_angles))
        msg = ServoAngles()
        
        msg.angle_1 = self.servo_angles[0]
        msg.angle_2 = self.servo_angles[1]
        msg.angle_3 = self.servo_angles[2]
        msg.angle_4 = self.servo_angles[3]
        msg.angle_5 = self.servo_angles[4]
        msg.angle_6 = self.servo_angles[5]
        
        self.publisher_.publish(msg)



def main(args=None):
    
    rclpy.init(args=args)
    node = DynamicsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
