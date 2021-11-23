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
import dynamic_reconfigure.client

""" 
Updates the dynamic footprint of the floribot's front and rear carriage according to the
current tf between 'axesRear' and 'axesFront' via dynamic reconfigure.
"""

# callback for dynamic client (only needed for debugging purposes)
# def dyn_cb(config):
#     """ Prints the robot's footprint after being set."
#     rospy.loginfo("Footprint set to {footprint}".format(**config))

def transform_point(tf_src_targ, point_wrt_source):
    """ Transforms a point [[x, y, z]] from source to target frame. """
    point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source), tf_src_targ).point
    return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

def transform_point_arr(point_arr, transform):
    """ Transforms an array of points [[x1, y1, z1], [x2, y2, z2], ...] from source to target frame. """
    tf_point_arr = []
    for point in point_arr:
        point_wrt_source = Point(point[0], point[1], point[2])
        point_wrt_target = transform_point(transform, point_wrt_source)
        tf_point_arr.append(point_wrt_target)
    return np.array(tf_point_arr)

if __name__ == "__main__":

    class LeadingCar(Enum):
        FRONT = 1
        REAR = 2
    boss = LeadingCar.FRONT
    # constant footprint vertices of front carriage in frame 'axesFront'='base_link'
    footprint_front = [[-0.383,0.001, 0.0],
                        [-0.243,0.168, 0.0],
                        [0.277,0.168, 0.0],
                        [0.277,-0.168, 0.0],
                        [-0.243,-0.168, 0.0]]
    footprint_front = np.array(footprint_front)

    # constant footprint vertices of rear carriage in frame 'axesRear' --> transform to 'axesFront'
    footprint_rear = [[0.383,-0.001, 0.0],
                        [0.243,-0.168, 0.0],
                        [-0.277,-0.168, 0.0],
                        [-0.277,0.168, 0.0],
                        [0.243,0.168, 0.0]]
    footprint_rear = np.array(footprint_rear)

    rospy.init_node("dynamic_footprint_client", anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener_tf = tf2_ros.TransformListener(tfBuffer)

    client1 = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=30)#, config_callback=dyn_cb)
    client2 = dynamic_reconfigure.client.Client("/move_base/global_costmap", timeout=30)#, config_callback=dyn_cb)

    period = 1/rospy.get_param("~rate")
    rate = rospy.Rate(rospy.get_param("~rate"))
    footprint_padding = rospy.get_param("~footprint_padding")
    
    while not rospy.is_shutdown():
        if boss == LeadingCar.FRONT:
            # if forward-driving: transform rear footprint into front frame
            source_frame = 'axesRear'
            target_frame = 'axesFront'
            footprint_lead = footprint_front
            footprint_follow = footprint_rear
        else:
            # if backward-driving: transform front footprint into rear frame
            source_frame = 'axesFront'
            target_frame = 'axesRear'
            footprint_lead = footprint_rear
            footprint_follow = footprint_front

        try:
            tf_src_targ = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(period))
        except tf2_ros.ConnectivityException:
            rospy.logerr('Tf tree between {:s} and {:s} is not connected'.format(source_frame, target_frame))
            rate.sleep
            continue
        except tf2_ros.ExtrapolationException:
            rospy.logerr('Requested tf value from {:s} to {:s} is beyond extrapolation limits'.format(source_frame, target_frame))
            rate.sleep
            continue
        except tf2_ros.InvalidArgumentException:
            rospy.logerr('Invalid arguments')
            rate.sleep
            continue
        except tf2_ros.LookupException:
            rospy.logerr('Name of required frame is not available or broken tf tree')
            rate.sleep
            continue
        except tf2_ros.TimoutException:
            rospy.logerr('Timeout has occured')
            rate.sleep
            continue
        else:
            footprint_follow_wrt_lead = transform_point_arr(footprint_follow, tf_src_targ)
            footprint = np.concatenate((footprint_lead, footprint_follow_wrt_lead))
            footprint_2d = footprint[:,:2] 
            footprint_2d_str = footprint_2d.tolist()
            client1.update_configuration({"footprint":footprint_2d_str, "footprint_padding":footprint_padding})
            client2.update_configuration({"footprint":footprint_2d_str, "footprint_padding":footprint_padding})
            rate.sleep()