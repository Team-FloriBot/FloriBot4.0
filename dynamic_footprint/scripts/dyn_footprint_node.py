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
Updates the dynamic footprint of the floribot's front and rear carriage according to the
current tf between 'axesRear' and 'axesFront' via dynamic reconfigure.
"""

class LEADING_CAR(Enum):
    FRONT = 0
    REAR = 1

class FOOTPRINT_MODE(Enum):
    STATIC = 0
    DYNAMIC = 1

class FootprintMode():
    def __init__(self):
        self.leading_car = LEADING_CAR.FRONT
        self.footprint_mode = FOOTPRINT_MODE.STATIC

        # constant footprint vertices of front carriage in frame 'axesFront'
        footprint_front = [[-0.383,0.001, 0.0],
                            [-0.243,0.168, 0.0],
                            [0.277,0.168, 0.0],
                            [0.277,-0.168, 0.0],
                            [-0.243,-0.168, 0.0],
                            [-0.383,-0.001, 0.0]]
        self.footprint_front = np.array(footprint_front)

        # constant footprint vertices of rear carriage in frame 'axesRear'
        footprint_rear = [[0.383,-0.001, 0.0],
                            [0.243,-0.168, 0.0],
                            [-0.277,-0.168, 0.0],
                            [-0.277,0.168, 0.0],
                            [0.243,0.168, 0.0],
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

        self.client1 = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=30)#, config_callback=dyn_cb)
        self.client2 = dynamic_reconfigure.client.Client("/move_base/global_costmap", timeout=30)#, config_callback=dyn_cb)
        self.srv = Server(FootprintModeConfig, self.srv_cb)

    def srv_cb(self, config, level):
        self.leading_car = LEADING_CAR(config.leading_car)
        self.footprint_mode = FOOTPRINT_MODE(config.footprint_mode)
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

        # when dynamic reconfigure parameter footprint_type has changed
        # --> update footprint with static footprint of lead carriage
        self.update_footprint(self.footprint_lead)
        return config

    def update_footprint(self, footprint):
        footprint_2d = footprint[:,:2] 
        footprint_2d_str = footprint_2d.tolist()
        self.client1.update_configuration({"footprint":footprint_2d_str, "footprint_padding":self.footprint_padding})
        self.client2.update_configuration({"footprint":footprint_2d_str, "footprint_padding":self.footprint_padding})
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
            if self.footprint_mode == FOOTPRINT_MODE.DYNAMIC:
                try:
                    tf_src_targ = self.tfBuffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(self.period))
                except tf2_ros.ConnectivityException:
                    rospy.logerr('Tf tree between {:s} and {:s} is not connected'.format(self.source_frame, self.target_frame))
                    self.rate.sleep
                    continue
                except tf2_ros.ExtrapolationException:
                    rospy.logerr('Requested tf value from {:s} to {:s} is beyond extrapolation limits'.format(self.source_frame, self.target_frame))
                    self.rate.sleep
                    continue
                except tf2_ros.InvalidArgumentException:
                    rospy.logerr('Invalid arguments')
                    self.rate.sleep
                    continue
                except tf2_ros.LookupException:
                    rospy.logerr('Name of required frame is not available or broken tf tree')
                    self.rate.sleep
                    continue
                except tf2_ros.TimoutException:
                    rospy.logerr('Timeout has occured')
                    self.rate.sleep
                    continue
                else:
                    footprint_follow_wrt_lead = self.transform_point_arr(self.footprint_follow, tf_src_targ)
                    footprint = np.concatenate((self.footprint_lead, footprint_follow_wrt_lead))
                    self.update_footprint(footprint)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("robot_footprint", anonymous=True)
    robot_footprint = FootprintMode()
    robot_footprint.main()
