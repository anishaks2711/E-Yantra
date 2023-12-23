#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2
import numpy as np

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty
from pkg_task5.msg import barcodes
from pkg_task5.msg import boxes
from pkg_ros_iot_bridge.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
from pkg_vb_sim.srv import *


home=[0.1368296643098441, -2.3735458519123833, -0.7537768120071, -1.5850663175928261, 1.5707963253414423, 0.1368296651320131]
l=[[[2.804992861764301, -1.915671565966706, -0.16692548385639583, -1.1194641566116257, 0.33711691874173244, 1.6281326011970512],[-2.0942206935053047, -1.4772974935735315, 0.28612812703535706, -1.9273462328591204, -1.047351787207429, 1.559364210078785],[-2.7912079662889324, -1.1795468413203132, 0.03701190950735356, -1.9407910020668613, -0.3508881528998753, 1.5161980951343228]],
    [[2.8065175440387105, -1.4369434262119372, -1.454463125750575, -0.31197626087678554, 0.33566694682550136, 1.6292538374630654],[2.104609017179069, -1.1107233465311523, -1.673206833550637, -0.3808235326068994, 1.037260872276062, 1.5827465830820913],[0.9354007944220921, -1.4767454274018847, -1.4202801236034333, -0.26935014867323837, 2.2059895685567614, 1.5561052865590783]],
    [[2.8189317303343717, -1.4418319756913789, -1.996799013018995, 0.23304740198454343, 0.32332776889606674, 1.6316780363960994],[2.1844091652264463, -1.0445275172359478, -2.25332410302647, 0.13139966369973877, 0.9571947584380291, 1.5852715041473031],[0.9461705528112967, -1.4692556075965388, -1.9737924727395217, 0.2760749516634302, 2.1952529219056203, 1.55620889368799]],
    [[-0.9692823817750327, -1.6244465088862174, 2.04195471120784, -0.39278610967933236, 2.172513290119539, -1.556263299487771],[-2.0449158737039212, -2.0324236092786547, 2.3309315018629952, -0.2751713916203622, 1.0973401964733753, -1.5808063690889567],[-2.804654074345218, -1.6177169191442875, 2.0369105711697237, -0.35676937672549247, 0.33798678021951023, -1.6291244624893935]]]

class Pub():
    def __init__(self):
        self.var_handle_pub = rospy.Publisher('qr_decode', barcodes, queue_size=10)

    def pub(self, vals):
        self.var_handle_pub.publish(vals)
        print("published")

class Camera1:
    bar=[]
    def __init__(self):
        self.cvbridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.cam_callback)
        self.inv_pub = rospy.Publisher("invBridge",invbridge,queue_size=10)
        self.var_handle_pub = rospy.Publisher('qr_decode', barcodes, queue_size=10)

    def sortfunc(self,sortlist):
        return sorted(sortlist , key=lambda x:x.polygon[0].x)

    def cam_callback(self,data):
        print("in cam callback")
        image = self.cvbridge.imgmsg_to_cv2(data, "bgr8")

        contrast_img = cv2.addWeighted(image, 6, np.zeros(image.shape, image.dtype), 0, 0)
        barco = pyzbar.decode(contrast_img)
        barco = sorted(barco, key = lambda bar:bar.polygon[0].y)
        barco = [barco[r:r+3] for r in range(0,12,3)]
        barco = list(map(self.sortfunc,barco))
        Camera1.bar=[barco[i][j].data for i in range(len(barco)) for j in range(len(barco[i]))]
        print([Camera1.bar[r:r+3] for r in range(0,12,3)])

        inv_msg = invbridge()
        for i in range(len(barco)):
            for j in range(len(barco[i])):
                if barco[i][j].data == "red":
                    sku = "R"+str((i*3)+j+1)+"0221"
                    priority = "HP"
                    item = "Medicine"
                    cost = "450"
                elif barco[i][j].data == "yellow":
                    sku = "Y"+str((i*3)+j+1)+"0221"
                    priority = "MP"
                    item = "Food"
                    cost = "250"
                elif barco[i][j].data == "green":
                    sku = "G"+str((i*3)+j+1)+"0221"
                    priority = "LP"
                    item = "Clothes"
                    cost = "150"
                strnu = "R"+str(i)+" C"+str(j)
                inv_msg=sku+", "+item+", "+priority+", "+strnu+", "+cost+", 1"
                self.inv_pub.publish(inv_msg)
                    
        obj_msg = barcodes()

        obj_msg.colour = Camera1.bar
        rospy.loginfo("Publishing: ")
        rospy.loginfo(obj_msg)

        self.var_handle_pub.publish(obj_msg)
        self.image_sub.unregister()




class Ur5Moveit:
    
    # Constructor
    def __init__(self, arg_robot_name, arg_colours):

        self.grip_state = rospy.get_param('gripper')
        self.tdlist = arg_colours
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._group.set_planning_time(99)
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.box_name = [['packagen00','packagen01','packagen02'],['packagen10','packagen11','packagen12'],['packagen20','packagen21','packagen22'],['packagen30','packagen31','packagen32']]
        self.orders = []
        self.orderssorted = []
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

        self.disp_pub = rospy.Publisher("dispBridge",dispbridge,queue_size=10)
        rospy.Subscriber("incoBridge",incobridge,self.incoOrders)
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self.check = True
        self.var_handle_pub = rospy.Publisher('box_pose', boxes, queue_size=10)

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    

    def set_joint_angles(self, arg_list_joint_angles):

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

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    def add_box(self,x,z,i,j):
        # print("called add box")
        box_name = self.box_name[i][j]
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = -.41
        box_pose.pose.position.z = z
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        self.box_name[i][j] = box_name

    def spawn_box(self):
        self.add_box(.28,1.92,0,0)
        self.add_box(0,1.92,0,1)
        self.add_box(-.28,1.92,0,2)
        self.add_box(.28,1.65,1,0)
        self.add_box(0,1.65,1,1)
        self.add_box(-.28,1.65,1,2)
        self.add_box(.28,1.42,2,0)
        self.add_box(0,1.42,2,1)
        self.add_box(-.28,1.42,2,2)
        self.add_box(.28,1.19,3,0)
        self.add_box(0,1.19,3,1)
        self.add_box(-.28,1.19,3,2)

    def attach_box(self,i,j):
        box_name = self.box_name[i][j]
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

    def detach_box(self,i,j):
        box_name = self.box_name[i][j]
        scene = self._scene
        eef_link = self._eef_link
        scene.remove_attached_object(eef_link, name=box_name)

    
    def remove_box(self,i,j):
        # print("called remove box")

        box_name = self.box_name[i][j]
        scene = self._scene
        scene.remove_world_object(box_name)

    def remove(self):
        self.remove_box(0,0)
        self.remove_box(0,1)
        self.remove_box(0,2)
        self.remove_box(1,0)
        self.remove_box(1,1)
        self.remove_box(1,2)
        self.remove_box(2,0)
        self.remove_box(2,1)
        self.remove_box(2,2)
        self.remove_box(3,0)
        self.remove_box(3,1)
        self.remove_box(3,2)

    def gripper(self, flag):
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        rospy.loginfo(
                '\033[94m' + str(flag) + '\033[0m')
        
        try:
            activate = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            resp = activate(flag)
            return resp.result
        except rospy.ServiceException, e:
            print ("Service call failed: %s"+e)    

    def doit(self):
        print("in doit()")
        disp_orders = dispbridge()
        box = boxes()
        while (len(self.orderssorted)>0):
            self.orderssorted[0][1]
            self.orderssorted[0][0]
            self.orderssorted[0][3]
            for i in range(0,4):
                for j in range(0,3):  
                    if (self.orderssorted[0][3]=="HP") and (self.tdlist[i][j]=="red"):  
                        self.hard_set_joint_angles(l[i][j], 10)
                        rospy.sleep(1)
                        self.attach_box(i,j)
                        print("entering loop")
                        while not rospy.get_param('/gripper/state'):
                            pass
                        print("exiting loop")
                        rospy.set_param('/gripper/state', False)
                        self.gripper(True)
                        self.hard_set_joint_angles(home,10)
                        self.detach_box(i,j)
                        self.gripper(False)
                        rospy.set_param('/gripper/state', True)
                        self.remove_box(i,j)
                        self.tdlist[i][j]=""
                        disp_orders.message = self.orderssorted[0][0]+", "+self.orderssorted[0][1]+", Medicine, HP, 1, 450"   
                        self.disp_pub.publish(disp_orders) 
                        print(self.orderssorted[0][1])
                        print(self.orderssorted[0][0])
                        box.city = self.orderssorted[0][1]
                        box.orderId = self.orderssorted[0][0]
                        box.i = i
                        box.j = j
                        self.var_handle_pub.publish(box)
                        print("dipatched:"+self.orderssorted[0][0])
                        del self.orderssorted[0]   
                    if (self.orderssorted[0][3]=="MP") and (self.tdlist[i][j]=="yellow"):  
                        self.hard_set_joint_angles(l[i][j], 10)
                        rospy.sleep(1)
                        self.attach_box(i,j)
                        print("entering loop")
                        while not rospy.get_param('/gripper/state'):
                            pass
                        print("exiting loop")
                        rospy.set_param('/gripper/state', False)
                        self.gripper(True)
                        self.hard_set_joint_angles(home,10)
                        self.detach_box(i,j)
                        self.gripper(False)
                        rospy.set_param('/gripper/state', True)
                        self.remove_box(i,j)
                        self.tdlist[i][j]=""
                        disp_orders.message = self.orderssorted[0][0]+", "+self.orderssorted[0][1]+", Food, MP, 1, 250"    
                        self.disp_pub.publish(disp_orders) 
                        print(self.orderssorted[0][1])
                        print(self.orderssorted[0][0])
                        box.city = self.orderssorted[0][1]
                        box.orderId = self.orderssorted[0][0]
                        box.i = i
                        box.j = j
                        self.var_handle_pub.publish(box)
                        print("dipatched:"+self.orderssorted[0][0])
                        del self.orderssorted[0]  
                    if (self.orderssorted[0][3]=="LP") and (self.tdlist[i][j]=="green"):  
                        self.hard_set_joint_angles(l[i][j], 10)
                        rospy.sleep(1)
                        self.attach_box(i,j)
                        print("entering loop")
                        while not rospy.get_param('/gripper/state'):
                            pass
                        print("exiting loop")
                        rospy.set_param('/gripper/state', False)
                        self.gripper(True)
                        self.hard_set_joint_angles(home,10)
                        self.detach_box(i,j)
                        self.gripper(False)
                        rospy.set_param('/gripper/state', True)
                        self.remove_box(i,j)
                        self.tdlist[i][j]=""
                        disp_orders.message = self.orderssorted[0][0]+", "+self.orderssorted[0][1]+", Clothes, LP, 1, 150"      
                        self.disp_pub.publish(disp_orders) 
                        print(self.orderssorted[0][1])
                        print(self.orderssorted[0][0])
                        box.city = self.orderssorted[0][1]
                        box.orderId = self.orderssorted[0][0]
                        box.i = i
                        box.j = j
                        self.var_handle_pub.publish(box)
                        print("dipatched:"+self.orderssorted[0][0])
                        del self.orderssorted[0]      


        # self.check = True

    def incoOrders(self, vals):
        rospy.loginfo("new order recieved")
        self.orders.append(list(vals.message.split(", ")))


        if len(self.orders)==9:
            self.orderssorted=sorted(self.orders,key=lambda x: 1 if x[3] == "HP" else(2 if x[3]=="MP" else 3))
            self.check = False
            self.doit()
        # vals=tuple(vals.mess.split(", "))
        # print(vals)
        # paramlist=["Order Id","City","Item","Priority","Order Quantity","Order Date and Time","Longitude","Latitude","Cost"]
        # for val,i in zip(vals,paramlist):
        #     parameters[i] = val
        
        # print(parameters)
        # response = requests.get(url, params=parameters)
        # print(response.content)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')




def main():
    rospy.init_node('node_t5_play', anonymous=True)
    # rospy.sleep(5)
    p = Pub()
    cam = Camera1()
    rospy.sleep(5)

    
    pkgnm=[Camera1.bar[r:r+3] for r in range(0,12,3)]

    ur5 = Ur5Moveit("ur5_1",pkgnm)
    ur5.spawn_box()
    # ur5.doit()
    while not rospy.is_shutdown():
        pass

    ur5.remove()
    del ur5



if __name__ == '__main__':
    main()

