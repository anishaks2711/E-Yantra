#! /usr/bin/env python

import rospy
import sys
import copy
import yaml
import os
import math
import rospkg

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import *
from hrwros_gazebo.msg import *
from pkg_ros_iot_bridge.msg import *
from pkg_task5.msg import *
from std_srvs.srv import Empty

home=[0.1368296643098441, -2.3735458519123833, -0.7537768120071, -1.5850663175928261, 1.5707963253414423, 0.1368296651320131]

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        """ 
        This is a Constructor for getting the arm to execute the planned trajectory based on the requests from the client node.

        Parameters:
            flag (bool) : Boolean flag to indicate whether the belt is called or not.

            box_length (float) : Stores the length of the package.

            vacuum_gripper_width (float) : Stores the width of the vacuum gripper.

            delta (float) : the algebraic sum of vacuum gripper width and package length, used to ensure that there is no collision. 

            ur5_2_home_pose (list) : this is the coordinate value of the pose of the arm.

            ur5_2_home_pose.position.x (float) : stores the x coordinate of the home position of the arm

            ur5_2_home_pose.position.y (float) : stores the y coordinate of the home position of the arm

            ur5_2_home_pose.position.z (float) : stores the z coordinate of the home position of the arm obtained by adding one to the vacuum gripper width and half of the box length

            self.ur5_2_home_pose.orientation.x (float) : stores home pose orientation along x axis

            self.ur5_2_home_pose.orientation.y (float) : stores home pose orientation along y axis

            self.ur5_2_home_pose.orientation.z (float) : stores home pose orientation along z axis

            self.ur5_2_home_pose.orientation.w (float) : stores home pose orientaion

        
        """

         # Initialize Simple Action Server
        self._sas = actionlib.SimpleActionServer('/belt_action',
                                                 beltAction,
                                                 execute_cb=self.func_on_rx_goal,
                                                 auto_start=False)
        
        self.flag=True
        self.box_length = 0.15               # Length of the Package
        self.vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = self.vacuum_gripper_width + (self.box_length/2)  # 0.19
        # Teams may use this info

        self.ur5_2_home_pose = geometry_msgs.msg.Pose()
        self.ur5_2_home_pose.position.x = -0.8
        self.ur5_2_home_pose.position.y = 0
        self.ur5_2_home_pose.position.z = 1 + self.vacuum_gripper_width + (self.box_length/2)
        # This to keep EE parallel to Ground Plane
        self.ur5_2_home_pose.orientation.x = -0.5
        self.ur5_2_home_pose.orientation.y = -0.5
        self.ur5_2_home_pose.orientation.z = 0.5
        self.ur5_2_home_pose.orientation.w = 0.5
        rospy.init_node('node_t5_server', anonymous=True)
        rospy.Subscriber("qr_decode", barcodes, self.func_callback_barcode)
        rospy.Subscriber("box_pos", boxes, self.func_callback_box)
        

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = []
        self._box_id = [["","",""],["","",""],["","",""],["","",""]]
        self._box_city = [["","",""],["","",""],["","",""],["","",""]]
        self.ship_pub = rospy.Publisher("shipBridge",shipbridge,queue_size=10)

        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        # Start the Action Server
        self._sas.start()
        rospy.loginfo("Started Simple Action Server.")
        self.grip_state = rospy.get_param('gripper')

    def func_callback_barcode(self,myMsg):

        """
        The function is called when there is a message published onto the barcodes topic.
        
        Parameters:
            myMsg (object): it is an object containing all the dtat in the barcodes message.

        """
        print('barcodes message recieved!')
        self._box_name=[myMsg.colour[r:r+3] for r in range(0,12,3)]


    def func_callback_box(self,myMsg):
        
        print('box details recieved')
        i=int(myMsg.i)
        j=int(myMsg.j)
        self._box_city[i][j]=myMsg.city
        self._box_id[i][j]=myMsg.orderId
        print(self._box_city[i][j])
        print(self._box_id[i][j])


    def clear_octomap(self):


        """
            This function takes an empty service call to activate vacuum gripper.
            
            Parameter: none
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):


        """
        The function path is planned to achieve the given joint angles.

        Parameters:
            arg_list_joint_angles (list): list of joint angle values. 
        """


        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        """
        The function makes the rviz plan a maximum number of times until it plans a valid path.
        
        Parameters:
            arg_list_joint_angles (list): list of joint angles for a particular function.

            arg_max_attempts (int): maximum number of attempts.
        
        """

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):

        """
        The function tries to play the planned path only once.

        Parameters:
            arg_file_path (str): the path to the yaml file that stores the trajectories.
            arg_file_name (str): name of the file storing the trajectories.

        
        """

        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):

        """
        The function attempts to play the planned path a maximum number of times until it is played successfully.   

        Paramters:
            arg_file_path (str): the path to the yaml file that stores the     trajectories.
            arg_file_name (str): name of the file storing the trajectories.
            arg_max_attempts (int): maximum number of attempts to play the planned path.
        
        """
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    def move_ur5(self,arg_type,arg_x):

        """
            This function is used to move the package from the conveyer belt to its respective bin, based on the colour of the package.

            Parameters:
                arg_type (string) : stores the name of the package.

                arg_x (float) : stores the difference between 0 and the x coordinate of the arm.

        """
        obj_msg_feedback = beltFeedback()
        obj_msg_feedback.feedback = False
        movex= round((arg_x),2)
        shipped_orders = shipbridge()
        # self.ee_cartesian_translation(movex,0,0)
        # self.gripper(True)
        if "package" in arg_type:
            i=int(arg_type[-2:-1])
            j=int(arg_type[-1:])
            print(arg_type)
            print(i)
            print(j)
            print(self._box_name[i][j])
            print(self._box_city[i][j])
            print(self._box_id[i][j])
            if "red" in self._box_name[i][j]:
                shp = ", Medicine, HP, 1, 450"
                self.ee_cartesian_translation(movex,0,0)
                print("entering loop")
                while not rospy.get_param('/gripper/state'):
                    pass
                print("exiting loop")
                rospy.set_param('/gripper/state', False)
                self.gripper(True)
                self.hard_set_joint_angles(home,2)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_red.yaml', 5)
                self.gripper(False)
                rospy.set_param('/gripper/state', True)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'red_to_home.yaml', 5)
            elif "green" in self._box_name[i][j]:
                shp = ", Clothes, LP, 1, 150"
                self.ee_cartesian_translation(movex,0,0)
                print("entering loop")
                while not rospy.get_param('/gripper/state'):
                    pass
                print("exiting loop")
                rospy.set_param('/gripper/state', False)
                self.gripper(True)
                self.hard_set_joint_angles(home,2)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_green.yaml', 5)
                self.gripper(False)
                rospy.set_param('/gripper/state', True)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'green_to_home.yaml', 5)
            elif "yellow" in self._box_name[i][j]:
                shp = ", Food, MP, 1, 250"
                self.ee_cartesian_translation(movex,0,0)
                print("entering loop")
                while not rospy.get_param('/gripper/state'):
                    pass
                print("exiting loop")
                rospy.set_param('/gripper/state', False)
                self.gripper(True)
                self.hard_set_joint_angles(home,2)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_yellow.yaml', 5)
                self.gripper(False)
                rospy.set_param('/gripper/state', True)
                self.moveit_hard_play_planned_path_from_file(self._file_path, 'yellow_to_home.yaml', 5)
            shipped_orders.message = self._box_id[i][j]+", "+self._box_city[i][j]+shp
            self.ship_pub.publish(shipped_orders)
        else:
            print("booting...")

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):

        """
        The function moves the ur5 by the given values with respect to its current position.

        Parameters:
            trans_x (float): value by which ur5 must be moved along x direction with respect to its current position.
            trans_y (float): value by which ur5 must be moved along y direction with respect to its current position.
            trans_z (float): value by which ur5 must be moved along z direction with respect to its current position.
        
        """
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    
    def go_to_pose(self, arg_pose):

        """
        The function is used to go to the given pose.

        Parameters:
            arg_pose (list): contains the required pose.
        
        """

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def gripper(self, flag):

        """
            This function is used for calling the gripper service.

            Parameters:
                flag (Boolean) : Boolean flag is used to indicate whether the gripper      
        """
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        rospy.loginfo(
                '\033[94m' + str(flag) + '\033[0m')

        
        try:
            activate = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            resp = activate(flag)
            return resp.result
        except rospy.ServiceException, e:
            print ("Service call failed: %s"+e)     

    def func_on_rx_goal(self, obj_msg_goal):

        """
        The function notifies if the goal is completed successfully.

        Parameters:
            obj_msg_goal (object): It is the goal sent from the client
        
        """
        rospy.loginfo("Received a Goal from Client.")
        rospy.loginfo(obj_msg_goal)

        flag_success = False        # Set to True if Goal is successfully achieved
        flag_preempted = False      # Set to True if Cancel req is sent by Client

        # --- Goal Processing Section ---
        self.move_ur5(obj_msg_goal.type,obj_msg_goal.x)
        

        # Send Result to the Client
        obj_msg_result = beltResult()
        obj_msg_result.flag=True

        rospy.loginfo("send goal result to client")
        self._sas.set_succeeded(obj_msg_result)

    # Destructor
    def __del__(self):

        """ 
            The destructor is used to deallocate memory assigned for the ur5_2 object.

        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    # rospy.sleep(10)
    ur5 = Ur5Moveit("ur5_2")
    
    ur5.go_to_pose(ur5.ur5_2_home_pose)
    ur5.gripper(True)
    ur5.gripper(False)
    ur5.gripper(True)
    ur5.gripper(False)
    rospy.spin()
        
         
    del ur5


if __name__ == '__main__':
    main()

