#! /usr/bin/env python

import rospy
import actionlib
from autonomous_robots.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

class MovetoPoses(object):

    def __init__(self):
        self._my_service = rospy.Service('/get_coordinates', MyCustomServiceMessage , self.my_callback)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def feedback_callback(self,feedback):
        print('[Feedback] Going to Goal Pose...')

    def my_callback(self,request):
        #print "Request Data==> duration="+str(request.duration)
        my_response = MyCustomServiceMessageResponse()
        if request.label == "Rack_1":
            goal1 = MoveBaseGoal()
            goal1.target_pose.header.frame_id = "map"
            goal1.target_pose.pose.position.x = rospy.get_param("point1/position/x")
            goal1.target_pose.pose.position.y = rospy.get_param("point1/position/y")
            goal1.target_pose.pose.position.z = rospy.get_param("point1/position/y")
            goal1.target_pose.pose.orientation.x = rospy.get_param("point1/orientation/x")
            goal1.target_pose.pose.orientation.y = rospy.get_param("point1/orientation/y")
            goal1.target_pose.pose.orientation.z = rospy.get_param("point1/orientation/z")
            goal1.target_pose.pose.orientation.w = rospy.get_param("point1/orientation/w")
    
            self.client.wait_for_server()
            self.client.send_goal(goal1, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            print('[Result] State: %d'%(self.client.get_state()))
            my_response.navigation_successful = True
            my_response.message = "OK"

        elif request.label == "Rack_2":
            goal2 = MoveBaseGoal()
            goal2.target_pose.header.frame_id = "map"
            goal2.target_pose.pose.position.x = rospy.get_param("point2/position/x")
            goal2.target_pose.pose.position.y = rospy.get_param("point2/position/y")
            goal2.target_pose.pose.position.z = rospy.get_param("point2/position/y")
            goal2.target_pose.pose.orientation.x = rospy.get_param("point2/orientation/x")
            goal2.target_pose.pose.orientation.y = rospy.get_param("point2/orientation/y")
            goal2.target_pose.pose.orientation.z = rospy.get_param("point2/orientation/z")
            goal2.target_pose.pose.orientation.w = rospy.get_param("point2/orientation/w")
    
            self.client.wait_for_server()
            self.client.send_goal(goal2, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            print('[Result] State: %d'%(self.client.get_state()))                                                                                         
            my_response.navigation_successful = True
            my_response.message = "OK"

        elif request.label == "Rack_3":
            goal3 = MoveBaseGoal()
            goal3.target_pose.header.frame_id = "map"
            goal3.target_pose.pose.position.x = rospy.get_param("point3/position/x")
            goal3.target_pose.pose.position.y = rospy.get_param("point3/position/y")
            goal3.target_pose.pose.position.z = rospy.get_param("point3/position/y")
            goal3.target_pose.pose.orientation.x = rospy.get_param("point3/orientation/x")
            goal3.target_pose.pose.orientation.y = rospy.get_param("point3/orientation/y")
            goal3.target_pose.pose.orientation.z = rospy.get_param("point3/orientation/z")
            goal3.target_pose.pose.orientation.w = rospy.get_param("point3/orientation/w")
    
            self.client.wait_for_server()
            self.client.send_goal(goal3, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            print('[Result] State: %d'%(self.client.get_state()))                                                                                         
            my_response.navigation_successful = True
            my_response.message = "OK"

        elif request.label == "Rack_4":
            goal3 = MoveBaseGoal()
            goal3.target_pose.header.frame_id = "map"
            goal3.target_pose.pose.position.x = rospy.get_param("point4/position/x")
            goal3.target_pose.pose.position.y = rospy.get_param("point4/position/y")
            goal3.target_pose.pose.position.z = rospy.get_param("point4/position/y")
            goal3.target_pose.pose.orientation.x = rospy.get_param("point4/orientation/x")
            goal3.target_pose.pose.orientation.y = rospy.get_param("point4/orientation/y")
            goal3.target_pose.pose.orientation.z = rospy.get_param("point4/orientation/z")
            goal3.target_pose.pose.orientation.w = rospy.get_param("point4/orientation/w")
    
            self.client.wait_for_server()
            self.client.send_goal(goal3, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            print('[Result] State: %d'%(self.client.get_state()))                                                                                         
            my_response.navigation_successful = True
            my_response.message = "OK"

        elif request.label == "Trolley":
            goal3 = MoveBaseGoal()
            goal3.target_pose.header.frame_id = "map"
            goal3.target_pose.pose.position.x = rospy.get_param("point5/position/x")
            goal3.target_pose.pose.position.y = rospy.get_param("point5/position/y")
            goal3.target_pose.pose.position.z = rospy.get_param("point5/position/y")
            goal3.target_pose.pose.orientation.x = rospy.get_param("point5/orientation/x")
            goal3.target_pose.pose.orientation.y = rospy.get_param("point5/orientation/y")
            goal3.target_pose.pose.orientation.z = rospy.get_param("point5/orientation/z")
            goal3.target_pose.pose.orientation.w = rospy.get_param("point5/orientation/w")
    
            self.client.wait_for_server()
            self.client.send_goal(goal3, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            print('[Result] State: %d'%(self.client.get_state()))                                                                                         
            my_response.navigation_successful = True
            my_response.message = "OK"

        else:
            my_response.navigation_successful = False
            my_response.message = "NOT OK"
    
        return  my_response # the service Response class, in this case MyCustomServiceMessageResponse

if __name__ == "__main__":
    rospy.init_node('goal') 
    move_poses_object  = MovetoPoses()
     # create the Service called my_service with the defined callback
    rospy.spin() # maintain the service open.