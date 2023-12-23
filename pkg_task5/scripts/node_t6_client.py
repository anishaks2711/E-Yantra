#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import *
from hrwros_gazebo.msg import *
from pkg_task5.msg import beltAction
from pkg_task5.msg import beltFeedback
from pkg_task5.msg import beltGoal
from pkg_task5.msg import beltResult

class Ur5MoveitClient:
    """ 
    This is a class for controlling the conveyor belt and to tell the Ur5_2 the position of the package on the belt. 
      
    """

    # Constructor
    def __init__(self):
        """ 
        The constructor Ur5MoveitClient class. 
        It also initializes the Action Client. 
  
        Parameters: 
            flag (bool): Boolean Flag to make sure the belt service isnt called everytime the callback function is called.
            f (bool): Boolean Flag to make sure the previously sent goal is completed before sending a new goal to the Server.
            i (int): The counter variable to keep track of number of goals sent to the Ur5_2 Server.   
        """
        self.flag=False
        self.f=True
        self.i=0
        self._ac = actionlib.SimpleActionClient('/belt_action',
                                                beltAction)
        self._ac.wait_for_server()
        
        rospy.loginfo("Action server is up, we can send new goals!")
        rospy.sleep(1)
        self.sb=rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,self.Callback)

    # Function to send Goals to Action Servers
    def send_goal(self, arg_type, arg_x):
        """ 
        The function to send goals to the Action Server. 
  
        Parameters: 
            arg_type (str): The name of the package is sent over to the server to help it sort it into the correct bin.
            arg_x (int): The relative position of the package in the frame that helps the Ur5 indentify the necessary adjustments to be made before picking up the package.   
        """

        # Create Goal message for Simple Action Server
        goal = beltGoal(type=arg_type, x=arg_x)
        print(goal)
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")

    # Function print result on Goal completion
    def done_callback(self, status, result):

        """
        The function to notify that goal has been completed by the action server.
        
        Parameters:
            result (bool): The status of the goal is sent.
        """
        print("goal completed")
        self.f=result.flag
        self.i=self.i+1
        print(self.i)
        if self.i>12:
            print("unsubscribed")
            self.sb.unregister()
            self.belt(False)

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        # self.f=feedback.feedback
        pass

    def Callback(self,msg):

      
        """ 
        The function Callback is called each time the logic camera detects a package and publishes. It also stops the belt when the package is in the centre of the frame, and sends goal to the action server along with type and color. When this is completed, the function done_callback is called.
  
        Parameters: 
            msg (object) : This is used to obtain the type and position of the package. 
        """
        # print(msg)
        pkgtype=""
        pkgposez=0
        pkgposey=1
        l=len(msg.models)-1
        for i in range (len(msg.models)):
            if "package" in msg.models[l].type:
                pkgtype=msg.models[l].type
                pkgposez=msg.models[l].pose.position.z
                pkgposey=msg.models[l].pose.position.y-0.02
        if pkgposey<0.03:
            if self.flag:
                self.belt(False)
                self.flag=False
            if self.f:
                print(pkgtype+" goal sent")
                self.send_goal(pkgtype,pkgposez)
                self.f=False
        elif not self.flag:
            self.belt(True)
            self.flag=True

    def belt(self, flag):

        """
        The function controls the working of the belt.

        Parameters:
            flag (bool) : indicates if the belt is moving.
            
        """
        speed=0
        if(flag):
            speed=100
        else:
            speed=0
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        #rospy.loginfo('\033[94m' + str(speed) + '\033[0m')
        try:
            activate = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            resp = activate(speed)
            return resp.result
        except rospy.ServiceException, e:
            print ("Service call failed: %s"+e)        


    # Destructor
    def __del__(self):
        """ 
        The destructor is used to deallocate memory assigned for the ur5 object.

        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5MoveitClient Deleted." + '\033[0m')


def main():
    rospy.init_node('node_t5_client')
    ur5 = Ur5MoveitClient()
    
    rospy.spin()    
        

if __name__ == '__main__':
    main()

