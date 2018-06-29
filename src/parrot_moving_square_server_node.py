#! /usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty
from actionlib.msg import TestFeedback, TestResult, TestAction

"""
goal: size of the side of the square -> the seconds to advance between each side-> integer: 1,2,3,4,5...
result: seconds to finish the task -> integer
feedback: 1, 2, 3, 4-> sides while is getting the state ->integer

To plpay the server:
rosrun parrot_moving_square parrot_moving_square_server_node.py

For calling the server:
rostopic pub /parrot_moving_square_action_server/goal actionlib/TestActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  goal: 4"
  
You can TAB-TAB for avoiding to write the message structure

"""

class UpDownDrone:
    def __init__(self, takeoff = None, land=None):
        self.takeoff = takeoff
        self.land = land
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        
    def up(self):
        #method to get up the drone
        self.takeoff = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
        
    def down(self):
        #method to land the drone
        self.land = rospy.Publisher("/drone/land", Empty, queue_size=1)
        
    def publish(self, choice):
        if choice:
            self.takeoff.publish(self.takeoff_msg)
            return True
        self.land.publish(self.land_msg)
        

class droneflyClass:
    _feedback = TestFeedback()
    _result = TestResult()
    
    def __init__(self, pub=None):
        self.pub = pub
        self.rate1 = rospy.Rate(2) #2 h
        self._as = actionlib.SimpleActionServer("parrot_moving_square_action_server", TestAction, self.goal_callback, False)
        self._as.start()
        
    def goal_callback(self, goal):
        
        #initializing dron take off and landing
        self.droneFlying = UpDownDrone()
        self.droneFlying.up()
        self.droneFlying.down()
        
        #raise the drone
        self.taking_off_land_the_drone(self.droneFlying, 1)
        
        r = rospy.Rate(1) #1 Hz frequency
        success = True
        
        self._feedback.feedback = 0
        
        rospy.loginfo("The movements are going to start")
        
        #initialize the Twist publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=4)
        total_seconds = 0
        while self._feedback.feedback <= 4:
            #check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break
            
            #moving the drone
            moving = self.moving_square(goal.goal)
            total_seconds +=(2+4+0.5+goal.goal)
            
            if moving:
                #publishing the feedback
                self._as.publish_feedback(self._feedback)
                self._feedback.feedback +=1
            else:
                rospy.logerr("Hubo un problema moviendo el drone")
            
            r.sleep()
        #end while---------------
        #landing the drone
        self.taking_off_land_the_drone(self.droneFlying, 0)
        
        if success:
            self._result.result = total_seconds
            rospy.loginfo("Succeeded moving in a square of size: %i" % goal.goal)
            self._as.set_succeeded(self._result)
        
    def taking_off_land_the_drone(self, drone, up_or_down):
        #We takeoff the drone during the first 3 seconds
        i=0
        while not i == 3:
            #taking off the drone
            drone.publish(up_or_down)
            rospy.loginfo('Taking off...')
            time.sleep(1)
            i += 1
    
    def moving_square(self, side_size):
        # Move Forwards
        self.do_a_move(side_size, 0.2, 0.0)
        # Stop
        self.do_a_move(2.0, 0, 0.0)
        # Turn
        self.do_a_move(4, 0, 0.4)
        # Stop
        self.do_a_move(0.5, 0, 0.0)
        
        return True
        
    def do_a_move(self, timing, linearist, angularist):
        #create the object to move
        my_vel=Twist()
        #set the variables to move
        my_vel.linear.x= linearist
        my_vel.angular.z= angularist
        
        self.publish_cmd_vel(my_vel)
        
        #from time python library in seconds
        time.sleep(timing)
        
        #now stop the bb8 for avoiding action to continue
        self.stopthedrone(my_vel)
        
    #just publish the actions
    def publish_cmd_vel(self, my_vel):
        #check if there are connections to publish
        connections = self.pub.get_num_connections()
        if connections > 0:
            try:
                #publishing the actions
                self.pub.publish(my_vel)
                rospy.loginfo("action published")
                return True
            except:
                pass
        else:
            #if no connections
            self.rate1.sleep()
            
    #in case to stop the robot
    def stopthedrone(self, my_vel):
        #stop the bb8
        my_vel.linear.x = 0
        my_vel.linear.y = 0
        my_vel.linear.z = 0
        my_vel.angular.z = 0
        
        self.pub.publish(my_vel)
        rospy.loginfo("Drone stopped")
        self.rate1.sleep()


if __name__=="__main__":
    rospy.init_node("parrot_square_node")
    droneflyClass()
    rospy.spin()
