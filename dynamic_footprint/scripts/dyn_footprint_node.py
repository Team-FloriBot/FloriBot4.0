#!/usr/bin/env python3
# Further references:
# An example showing how to use tf2_ros API: https://gist.github.com/ravijo/cb560eeb1b514a13dc899aef5e6300c0
# lookupTransform(): https://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1Buffer.html
# tf2TutorialsWriting a tf2 listener (Python): http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29

from inspect import signature
import rospy
from geometry_msgs.msg import Point, PointStamped, Twist
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from enum import Enum
import dynamic_reconfigure.client # for sending modified footprint
from dynamic_reconfigure.server import Server # for choosing between dynamic and static footprint
from dynamic_footprint.cfg import FootprintModeConfig
""" 
Updates the footprint of the floribot's front and rear carriage according to settings in dynamic reconfigure:
LEADING_CAR:                The carriage which is the head of the robot, either FRONT or REAR.
                            The navigation frame (default: base_link) must be positioned at the specified HEAD.
                            - if FRONT, then navigation frame must be front axle (identical to base_link)
                            - if REAR, then navigation frame must be at rear axle (not implemented yet)
FOOTPRINT_MODE:             What mode the footprint is to work in.
Different combinations:     
-----------------------------------------------------------------------------------------------------------------
LEADING_CAR |   FOOTPRINT_MODE  |   footprint behavior                                                          |
-----------------------------------------------------------------------------------------------------------------
FRONT       |   STATIC          |   Only the footprint of the front carriage (which is static) is used,         |
            |                   |   regardless of the driving direction.                                        |
-----------------------------------------------------------------------------------------------------------------
REAR        |   STATIC          |   Only the footprint of the rear carriage (which is static) is used,          |
            |                   |   regardless of the driving direction.                                        |
-----------------------------------------------------------------------------------------------------------------
FRONT/ REAR |   SEMI_STATIC     |   When driving forwards: static front footprint is used.                      |
            |                   |   When driving backwards: dynamic rear footprint is used depending on         |
            |                   |   on current angle of articulation joint.                                     |
-----------------------------------------------------------------------------------------------------------------
FRONT/ REAR |   DYNAMIC         |   Both the static front footprint and the dynamic rear footprint are used     |
            |                   |   depending on current angle of articulation joint and regardless of driving  |
            |                   |   direction.                                                                  |
-----------------------------------------------------------------------------------------------------------------
"""

class LEADING_CAR(Enum):
    FRONT = 0
    REAR = 1

class FOOTPRINT_MODE(Enum):
    STATIC = 0
    SEMI_STATIC = 1
    DYNAMIC = 2

class DRIVING_DIRECTION(Enum):
    FORWARD = 0
    BACKWARD = 1

class FootprintMode():
    def __init__(self):
        self.leading_car = LEADING_CAR.FRONT
        self.footprint_mode = FOOTPRINT_MODE.STATIC
        self.driving_direction = DRIVING_DIRECTION.FORWARD
        self.update_local_footprint = True
        self.update_global_footprint = False
        # constant footprint vertices of front carriage in frame 'axesFront'
        footprint_front = [[-0.383,0.001, 0.0],
                            [-0.3,0.225, 0.0],
                            [0.5,0.225, 0.0],
                            [0.5,-0.225, 0.0],
                            [-0.3,-0.225, 0.0],
                            [-0.383,-0.001, 0.0]]
        self.footprint_front = np.array(footprint_front)

        # constant footprint vertices of rear carriage in frame 'axesRear'
        footprint_rear = [[0.383,-0.001, 0.0],
                            [0.3,-0.225, 0.0],
                            [-0.5,-0.225, 0.0],
                            [-0.5,0.225, 0.0],
                            [0.3,0.225, 0.0],
                            [0.383,0.001, 0.0]]
        self.footprint_rear = np.array(footprint_rear)
        
        self.footprint_lead = self.footprint_front
        self.footprint_follow = self.footprint_rear
        self.source_frame = 'axesRear'
        self.target_frame = 'axesFront'     

        self.period = 1/rospy.get_param("~rate")
        self.rate = rospy.Rate(rospy.get_param("~rate"))
        self.footprint_padding = rospy.get_param("~footprint_padding")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener_tf = tf2_ros.TransformListener(self.tfBuffer)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, callback=self.cmd_cb)
        self.client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=None)#, config_callback=dyn_cb) # infinite timeout
        self.client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap", timeout=None)#, config_callback=dyn_cb) # infinite timeout
        self.srv = Server(FootprintModeConfig, self.srv_cb)

    def cmd_cb(self, cmd_vel):
        self.driving_direction = DRIVING_DIRECTION.FORWARD if cmd_vel.linear.x >= 0 else DRIVING_DIRECTION.BACKWARD

    def srv_cb(self, config, level):
        self.leading_car = LEADING_CAR(config.leading_car)
        self.footprint_mode = FOOTPRINT_MODE(config.footprint_mode)
        self.update_local_footprint = config.update_local_footprint
        self.update_global_footprint = config.update_global_footprint
        rospy.loginfo("Received Reconfigure Requests: %s footprint and %s leading car", self.footprint_mode.name, self.leading_car.name)
        if self.leading_car == LEADING_CAR.FRONT:
            # if forward-driving: transform rear footprint into front frame
            self.source_frame = 'axesRear'
            self.target_frame = 'axesFront'
            self.footprint_lead = self.footprint_front
            self.footprint_follow = self.footprint_rear
        else:
            # if backward-driving: transform front footprint into rear frame
            self.source_frame = 'axesFront'
            self.target_frame = 'axesRear'
            self.footprint_lead = self.footprint_rear
            self.footprint_follow = self.footprint_front
        return config

    def update_footprint(self, footprint):
        footprint_2d = footprint[:,:2] 
        footprint_2d_str = footprint_2d.tolist()
        if self.update_local_footprint:
            self.client_local.update_configuration({"footprint":footprint_2d_str, "footprint_padding":self.footprint_padding})
        if self.update_global_footprint:
            self.client_global.update_configuration({"footprint":footprint_2d_str, "footprint_padding":self.footprint_padding})
        return

    # callback for dynamic client (only needed for debugging purposes)
    # def dyn_cb(config):
    #     """ Prints the robot's footprint after being set."
    #     rospy.loginfo("Footprint set to {footprint}".format(**config))

    def transform_point(self, tf_src_targ, point_wrt_source):
        """ Transforms a point [[x, y, z]] from source to target frame. """
        point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source), tf_src_targ).point
        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

    def transform_point_arr(self, point_arr, transform):
        """ Transforms an array of points [[x1, y1, z1], [x2, y2, z2], ...] from source to target frame. """
        tf_point_arr = []
        for point in point_arr:
            point_wrt_source = Point(point[0], point[1], point[2])
            point_wrt_target = self.transform_point(transform, point_wrt_source)
            tf_point_arr.append(point_wrt_target)
        return np.array(tf_point_arr)

    def main(self):
        while not rospy.is_shutdown():
            try:
                tf_src_targ = self.tfBuffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(self.period))
            except tf2_ros.ConnectivityException:
                rospy.logwarn('Tf tree between {:s} and {:s} is not connected'.format(self.source_frame, self.target_frame))
                continue
            except tf2_ros.ExtrapolationException:
                rospy.logwarn('Requested tf value from {:s} to {:s} is beyond extrapolation limits'.format(self.source_frame, self.target_frame))
                continue
            except tf2_ros.InvalidArgumentException:
                rospy.logwarn('Invalid arguments')
                continue
            except tf2_ros.LookupException:
                rospy.logwarn('Name of required frame is not available or broken tf tree')
                continue
            except tf2_ros.TimoutException:
                rospy.logwarn('Timeout has occured')
                continue
            else:
                footprint_follow_wrt_lead = self.transform_point_arr(self.footprint_follow, tf_src_targ)                    
                if self.footprint_mode == FOOTPRINT_MODE.DYNAMIC:
                    # if front car leads: transform rear footprint into front frame and concatenate with front footprint
                    # if rear car leads: transform front footprint into rear frame and concatenate with rear footprint
                    footprint = np.concatenate((self.footprint_lead, footprint_follow_wrt_lead))
                    self.update_footprint(footprint)
                elif self.footprint_mode == FOOTPRINT_MODE.SEMI_STATIC and (self.driving_direction == DRIVING_DIRECTION.BACKWARD):
                    # if front car leads: transform rear footprint into front frame and only use this new rear footprint
                    # if rear car leads: transform front footprint into rear frame and only use this new front footprint
                    self.update_footprint(footprint_follow_wrt_lead)
                else:
                    try:
                        self.update_footprint(self.footprint_lead)
                    except dynamic_reconfigure.DynamicReconfigureCallbackException:
                        rospy.logwarn('Dynamic Reconfigure server is down. Latest footprint could not be transmitted via callback.')
                        continue
            finally:
                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("robot_footprint", anonymous=True)
    robot_footprint = FootprintMode()
    robot_footprint.main()
