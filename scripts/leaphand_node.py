#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand_densetact.srv import *
#######################################################
"""This Controls the LEAP Hand and also sets up ros services that allow you to query the hand.

The services allow you to always have the latest data when you want it, and not spam the communication lines with unused data.

I recommend you only query using services when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

Subscribes
----------
joint_angles

Services
----------
Makes joint angles, velocity and current available for reading.
Controls robotic hand
"""
########################################################
class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        ####Some parameters to control the hand
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = self.get_parameter('kP').value if self.has_parameter('kP') else 800.0
        self.kI = self.get_parameter('kI').value if self.has_parameter('kI') else 0.0
        self.kD = self.get_parameter('kD').value if self.has_parameter('kD') else 200.0
        self.curr_lim = self.get_parameter('curr_lim').value if self.has_parameter('curr_lim') else 550.0 #don't go past 600ma on this, or it'll overcurrent sometimes for regular, 350ma for lite.
        self.ema_amount = 0.2
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        
        #subscribes to a variety of sources that can command the hand, and creates services that can give information about the hand out
        self.create_subscription(JointState,"/leaphand_node/cmd_leap", self._receive_pose,10)
        self.create_subscription(JointState,"/leaphand_node/cmd_allegro", self._receive_allegro,10)
        self.create_subscription(JointState,"/leaphand_node/cmd_ones", self._receive_ones,10)
        self.pos_srv = self.create_service(LeapPosition, 'Leap_Position', self.pos_srv)
        self.vel_srv = self.create_service(LeapVelocity, 'Leap_Velocity', self.vel_srv)
        self.eff_srv = self.create_service(LeapEffort, 'Leap_Effort', self.eff_srv)

        
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        # while not rospy.is_shutdown():
        #     rospy.spin()

    #Receive LEAP pose and directly control the robot
    def _receive_pose(self, pose):
        pose = pose.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Allegro compatibility, first read the allegro publisher and then convert to leap
    def _receive_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose.position, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim publisher and then convert to leap
    def _receive_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose.position))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, req, response):
        pos_data = self.dxl_client.read_pos()

        position_list = pos_data.tolist()

        response.position = position_list
        # self.get_logger().info('Responded!')
        # return {"position": self.dxl_client.read_pos()}
        return response
    #Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, req):
        return {"velocity": self.dxl_client.read_vel()}
    #Service that reads and returns the effort/current of the robot in LEAP Embodiment
    def eff_srv(self, req):
        return {"effort": self.dxl_client.read_cur()}
#init the arm node
def main(args=None):
    rclpy.init(args=args)
    node = LeapNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    #this is where the script begins, calls the main function
    main()
