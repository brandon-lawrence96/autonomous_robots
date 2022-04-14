#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf_conversions
from tf import transformations
import tf2_ros
import geometry_msgs.msg
import math

index = 0


def posecallback(data):
    odom = Odometry()
    odom_box = Odometry()
    global index
    index = 0
 
    #if (len(data.pose) > 1):
    

    odom.pose.pose = data.pose.pose
    pose_orig = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    
    odom.pose.pose.position.z = data.pose.pose.position.z + 0.238
    #rot_angle_roll = 0
    #rot_angle_pitch = 0
    #rot_angle_yaw = 0.69813

    #q_rot = transformations.quaternion_from_euler(rot_angle_roll, rot_angle_pitch, rot_angle_yaw)
    #q_new = transformations.quaternion_multiply(q_rot, q_orig)
    #odom.pose.pose.orientation.x = q_new[0]
    #odom.pose.pose.orientation.y = q_new[1]
    #odom.pose.pose.orientation.z = q_new[2]
    #odom.pose.pose.orientation.w = q_new[3]

    #p_orig = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
    #odom.pose.pose.position.x = p_orig[0] - 0.002075
    #odom.pose.pose.position.y = p_orig[1] + 0.190085
    #odom.pose.pose.position.z = p_orig[2] - 0.0611603


    #robot_rpy = transformations.euler_from_quaternion(q_new)
    #robot_rpy_msg = Vector3()
    #robot_rpy_msg.x = robot_rpy[0]
    #robot_rpy_msg.y = robot_rpy[1]
    #robot_rpy_msg.z = robot_rpy[2]

    #rpy_pub = rospy.Publisher('/riltaur/rpy', Vector3, queue_size=10)
    #py_pub.publish(robot_rpy_msg)
    # odom.pose.pose.orientation.x = 0
    # odom.pose.pose.orientation.y = 0
    # odom.pose.pose.orientation.z = 0
    # odom.pose.pose.orientation.w = 1

    odom.twist.twist = data.twist.twist
    #angularx = odom.twist.twist.angular.x
    #angulary = odom.twist.twist.angular.y
    # rospy.loginfo("bef: {} {}".format(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y))
    #odom.twist.twist.angular.x = angularx * math.cos(rot_angle)-angulary*math.sin(rot_angle)
    #odom.twist.twist.angular.y = angularx * math.sin(rot_angle)+angulary*math.cos(rot_angle)
    # rospy.loginfo("aft: {} {}".format(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y))

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "t265_pose_frame"
    #odom.child_frame_id = "dummy_link"

    pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    pub.publish(odom)

    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "odom"
    # t.child_frame_id = "t265_pose_frame"
    # #t.child_frame_id = "dummy_link"
    # t.transform.translation.x = odom.pose.pose.position.x
    # t.transform.translation.y = odom.pose.pose.position.y
    # t.transform.translation.z = odom.pose.pose.position.z

    # t.transform.rotation.x = odom.pose.pose.orientation.x
    # t.transform.rotation.y = odom.pose.pose.orientation.y
    # t.transform.rotation.z = odom.pose.pose.orientation.z
    # t.transform.rotation.w = odom.pose.pose.orientation.w

    # br.sendTransform(t)

        # print(odom)


def main():
    rospy.init_node('t265_odom_converter', anonymous=True)
    rospy.Subscriber("/t265/odom/sample", Odometry, posecallback)
    rate = rospy.Rate(100)  # 10hz
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass