#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist as Twist
from geometry_msgs.msg import PoseStamped as PoseStamped
from geometry_msgs.msg import PointStamped as PointStamped
from nav_msgs.msg import Odometry as Odometry
from nav_msgs.msg import Path as Path
import tf2_ros
import tf2_geometry_msgs

maxLin=0.1
maxRot=0.3
maxSpeed=math.sqrt(maxLin**2+maxRot**2)
target = PoseStamped()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pub_pred = rospy.Publisher('pred_point', PointStamped, queue_size=10)
pub_target = rospy.Publisher('target_point', PointStamped, queue_size=10)
pub_path = rospy.Publisher('pathfollowing_node_path', Path, queue_size=10)
tfBuffer = tf2_ros.Buffer()
newTarget=True
precise=0.05
last_idx=-1
base_link='base_link'
time_dist=1

def getfuturePos(location,timeprediction):
    '''
        Here we predict the future position of the robot

        input:  location        current location of the Robot 
                timeprediction  time which needs to predict. affects the agility of the robot
    '''
    try:
        odom_base=PoseStamped()
        odom_base.header=location.header
        odom_base.header.frame_id=base_link
        odom_base.pose.orientation.w=1.0
        odom_base.pose.position.x+=(maxLin*timeprediction)
        odom_pred=tfBuffer.transform(odom_base,'odom')

        #transform to odom
        predPoint=PoseStamped()
        predPoint.header.stamp=rospy.Time.now()
        predPoint.header.frame_id='odom'
        predPoint.pose=odom_pred.pose

        # only for visualisation
        point =PointStamped()
        point.header=predPoint.header
        point.point=predPoint.pose.position
        pub_pred.publish(point)
        return predPoint
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Unable to transform odom")
        return None

def applyScalarProjection(line,point):
    '''
        this srcipt compute apply a project the future position onto the line segment to get the nomral point between each

        input:  lines   discribes the start and end point of the line 
                point   represents the future position of the robot
    '''

    point=[point.pose.position.x, point.pose.position.y] #generate numpy-point of future position

    #compute point on the line wich represents the normal point between the line and the future position.
    robot_2_p2=np.subtract(point, line[1]) 
    ab =np.subtract(line[1],line[0])
    ab=ab/np.linalg.norm(ab)
    ab=ab*np.dot(robot_2_p2,ab)
    normalPoint = np.add(line[1],ab)

    #get magnitude to each points 
    mag_normalPoint=np.linalg.norm(normalPoint) 
    mag_p1=np.linalg.norm(line[0])
    mag_p2=np.linalg.norm(line[1])

    # set normal point if robot will not stay on segment
    if (mag_normalPoint<mag_p1 and mag_normalPoint<mag_p2): # if robot is on the way to segment set begin as normal point
        normalPoint=line[0]
    if (mag_normalPoint>mag_p1 and mag_normalPoint>mag_p2): # if robot is leaving segment set end as normal point
        normalPoint=line[1]

    return normalPoint

def getClosestDist(path,predPoint):
    closest_dist=100000.0
    target =PoseStamped()
    line_tmp=[]
    distance_temp=0
    normalPointTemp=0
    for i in range(0,len(path.poses)-1):
        if i <(len(path.poses)-1):
            line=[[path.poses[i].pose.position.x, path.poses[i].pose.position.y],
                [path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y]]
            normalPoint=applyScalarProjection(line,predPoint)
            distance=np.linalg.norm([normalPoint[0]-predPoint.pose.position.x,normalPoint[1]-predPoint.pose.position.y])
            print("distance normal {}".format(distance))
        else:
            normalPoint=[path.poses[i].pose.position.x, path.poses[i].pose.position.y]
            distance=np.linalg.norm([normalPoint[0]-predPoint.pose.position.x,normalPoint[1]-predPoint.pose.position.y])
        # print (line)
        # print (normalPoint)

        
        print("distance {}".format(distance))
        if distance<closest_dist:
            target.header=predPoint.header
            target.pose.position.x=normalPoint[0]
            target.pose.position.y=normalPoint[1]
            line_tmp=line
            distance_temp=distance
            normalPointTemp=normalPoint
            closest_dist=distance
    print ("end")
    targetPoint= PointStamped()
    targetPoint.header=target.header
    targetPoint.point=target.pose.position
    pub_target.publish(targetPoint)
    return target             

def getDirectPath(odom,target):
    #subtract the position from the target to get the normalized disired vector
    directPath=None
    try:
        target.header.stamp=odom.header.stamp
        targetpos=tfBuffer.transform(target,base_link, rospy.Duration(0.0))
        directPath=[targetpos.pose.position.x, targetpos.pose.position.y]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Unable to retrive Point")
        rospy.Rate(10).sleep()
    

    return directPath

def cpSteeringVel(normPVector,cmd_vel,maxSpeed,maxRot):
    vel=[maxLin, 0]
    normPVector*=maxSpeed
    rotVel=0.0
    turnVel=normPVector-vel
    turnVel[1]= turnVel[1] if np.abs(turnVel[1])<=maxRot else np.sign(turnVel[1])*maxRot
    rotVel=turnVel[1]
    if(normPVector[0]<0.0):
        rotVel= np.sign(rotVel)*maxRot   
    return rotVel

def followTarget(odom,target):
    cmd_vel =Twist()  
    cmd_vel.linear.x=maxLin
    directPath=getDirectPath(odom,target)
    # print(directPath)
    if directPath is not None: #and np.linalg.norm(directPath)>precise:
        normPVector=directPath/np.linalg.norm(directPath)
        rotVel=cpSteeringVel(normPVector,cmd_vel,maxSpeed,maxRot)  
        cmd_vel.angular.z=rotVel  
    else:
        newTarget=False if directPath is not None else newTarget
        cmd_vel.linear.x=0.0
        cmd_vel.angular.z=0.0
    return cmd_vel

def getPath():
    path =Path()
    coordinates=[[1.0,0.0],[1.5,0.5],[2.0,1.0],[2.0,2.0],[1.5,3.0]]
    path.header.stamp=rospy.Time.now()
    path.header.frame_id='odom'
    for i in range(0,len(coordinates)):
        pose=PoseStamped()
        pose.header.stamp=rospy.Time.now()
        pose.header.frame_id='odom'
        pose.pose.position.x=coordinates[i][0]
        pose.pose.position.y=coordinates[i][1]
        path.poses.append(pose)
    pub_path.publish(path)

    return path


def odom_callback(data):

    cmd_vel = Twist()
    cmd_vel.linear.x=0.0
    cmd_vel.angular.z=0.0 

    point=getfuturePos(data,time_dist)
    path=getPath()
    target=getClosestDist(path,point)

    if newTarget is True : 
        cmd_vel= followTarget(data,target)
   
    pub.publish(cmd_vel)

def pathfollowing_callback (data):
    global newTarget
    global target
    target = data
    newTarget=True
    
def listener():
    rospy.init_node('pathfollowing', anonymous=True)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("/move_base_simple/goal",PoseStamped,pathfollowing_callback)
    tf2_ros.TransformListener(tfBuffer)
    rospy.spin()
  
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass