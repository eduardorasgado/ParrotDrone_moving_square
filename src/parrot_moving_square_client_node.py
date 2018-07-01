#! /usr/bin/env python

import rospy
import actionlib
from actionlib.msg import TestAction, TestGoal, TestResult, TestFeedback
from std_msgs.msg import Empty
from numbers import Integral

class FlyCaller:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR =4
    
    def __init__(self, size_of_side=2, action_server_name = '/parrot_moving_square_action_server', action= TestAction):
        self.size_of_side = size_of_side
        self.action_server_name = action_server_name
        self.action = action
        
        self.nodo = rospy.init_node('parrot_moving_square_client_action_node')
        self.client = actionlib.SimpleActionClient(action_server_name, self.action)
        
        #waiting for the server rensponse
        self.waitingforServer(self.client)
        #goal passing by and trying to hadle it
        self.goalHandling(self.client, self.size_of_side)
        
        #for feedback receiving
        self.state_result = self.client.get_state()
        
        self.rate = rospy.Rate(1) #1Hz frequency
        
        self.state_result = self.square_communication(self.state_result, self.rate, self.client)
        
        message_result = "[Result] State: ",str(self.state_result)
        rospy.loginfo(message_result)
        
        if self.state_result == self.ERROR:
            rospy.logerr("Something went wrong in the server")
        if self.state_result == self.WARN:
            rospy.logwarn("There is a warning in the server")
        
    def waitingforServer(self, client):
        rospy.loginfo("Waiting for action server..."+ self.action_server_name)
        client.wait_for_server()
        rospy.loginfo("Action server found: "+ self.action_server_name)
        
    def goalHandling(self, client, side):
        #send the goal to the server for start the action
        goal = TestGoal()
        goal.goal = side
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        
    def feedback_callback(self, feedback):
        #handles the data received from server while result is not DONE
        msg = '[Feedback] side of square: '+str(feedback.feedback)
        rospy.loginfo(msg)
        
    def square_communication(self, state_result, rate, client):
        #iterates through the drone flight and receives the result
        while state_result < self.DONE:
            rospy.loginfo("EVERYTHING IS GOING ON WELL-------------------------")
            rate.sleep()
            state_result = client.get_state()
            msg = "state_result: "+str(state_result)
            rospy.loginfo(msg)
        return state_result
        
        

if __name__=='__main__':
    side_walk = input("Insert size of square side for Quadrocopter movement: ")
    if isinstance(side_walk, Integral):
        FlyCaller(side_walk)
    else:
        print("This is not a valid entry data(Please insert a number)...")
        
        