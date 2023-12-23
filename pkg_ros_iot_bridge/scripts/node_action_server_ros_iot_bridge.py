#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import requests
import json

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import *
from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks


class color:
   PURPLE = '\033[1;35;48m'
   CYAN = '\033[1;36;48m'
   BOLD = '\033[1;37;48m'
   BLUE = '\033[1;34;48m'
   GREEN = '\033[1;32;48m'
   YELLOW = '\033[1;33;48m'
   RED = '\033[1;31;48m'
   BLACK = '\033[1;30;48m'
   UNDERLINE = '\033[4;37;48m'
   END = '\033[1;37;0m'

class RosIotBridgeActionServer:

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)


        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)


        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_ros_iot.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        self._config_mqtt_sub_topic, 
                                                        self._config_mqtt_qos   )
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")


        # Start the Action Server
        self._as.start()
        
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")
    
    # defining our sheet name in the 'id' variable and the the column where we want to update the value
        
        rospy.Subscriber("invBridge",invbridge,self.invPush)
        rospy.Subscriber("shipBridge",shipbridge,self.shipPush)
        rospy.Subscriber("dispBridge",dispbridge,self.dispPush)
        self.orders_pub = rospy.Publisher("incoBridge",incobridge,queue_size=10)
    
    #response = requests.get(URL, params=parameters)
    

    def invPush(self, vals):

        '''
            The function invPush pushes the values to the Inventory sheet of the spreadsheet. Every value is added under the header as mentioned in the paramlist. 
        '''

        url = "https://script.google.com/macros/s/AKfycbwzgzPPN_7YEeDKDRVFKrfpIux5LW9LEMYvxXDoPS3RkIw8UOxpztsuOw/exec"
        parameters = {"id":"Inventory","Team Id":"VB_1021","Unique Id":"UNfeoAG"} 
        vals=tuple(vals.message.split(", "))
        print(vals)
        paramlist=["SKU","Item","Priority","Storage Number","Cost","Quantity"]
        for val,i in zip(vals,paramlist):
            parameters[i] = val
        response = requests.get(url, params=parameters)
        print(response.content)

    def shipPush(self, vals):

        '''
            The function shipPush pushes the values to the Orders Shipped sheet of the spreadsheet. Every value is added under the header as mentioned in the paramlist. 
           
        '''
        url = "https://script.google.com/macros/s/AKfycbwzgzPPN_7YEeDKDRVFKrfpIux5LW9LEMYvxXDoPS3RkIw8UOxpztsuOw/exec"
        parameters = {"id":"Orders Shipped","Team Id":"VB_1021","Unique Id":"UNfeoAG","Shipped Status":"Yes"} 
        
        vals=tuple(vals.message.split(", "))
        paramlist=["Order Id","City","Item","Priority","Shipped Quantity","Cost"]
        for val,i in zip(vals,paramlist):
            parameters[i] = val
        
        response = requests.get(url, params=parameters)
        print(response.content)

    def dispPush(self, vals):

        '''
            The function dispPush pushes the values to the Orders Dispatched sheet of the spreadsheet. Every value is added under the header as mentioned in the paramlist. 
           
        '''
        url = "https://script.google.com/macros/s/AKfycbwzgzPPN_7YEeDKDRVFKrfpIux5LW9LEMYvxXDoPS3RkIw8UOxpztsuOw/exec"
        parameters = {"id":"Orders Dispatched","Team Id":"VB_1021","Unique Id":"UNfeoAG","Dispatch Status":"Yes"} 
        
        vals=tuple(vals.message.split(", "))
        paramlist=["Order Id","City","Item","Priority","Dispatch Quantity","Cost"]
        for val,i in zip(vals,paramlist):
            parameters[i] = val
        
        response = requests.get(url, params=parameters)
        print(response.content)

    def incoPush(self, vals):

        '''
            The function incoPush pushes the values to the Incoming Orders sheet of the spreadsheet. Every value  is added under the header as mentioned in the paramlist. It also sends it to the play node.
           
        '''   
        url = "https://script.google.com/macros/s/AKfycbwzgzPPN_7YEeDKDRVFKrfpIux5LW9LEMYvxXDoPS3RkIw8UOxpztsuOw/exec"
        parameters = {"id":"Incoming Orders","Team Id":"VB_1021","Unique Id":"UNfeoAG"} 
        
        vals=tuple(vals.split(", "))
        paramlist=["Order Id","City","Item","Priority","Order Quantity","Order Date and Time","Longitude","Latitude","Cost"]
        for val,i in zip(vals,paramlist):
            parameters[i] = val
        
        response = requests.get(url, params=parameters)
        print(response.content)

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):


        '''
            The function mqtt_sub_callback 
           
        '''
        payload = json.loads(message.payload)
        if payload["item"]=="Medicine":
            cost = "450"
            priority = "HP"
        elif payload["item"]=="Food":
            cost = "250"
            priority = "MP"
        elif payload["item"]=="Clothes":
            cost = "150"
            priority = "LP"
        vav = payload["order_id"]+", "+payload["city"]+", "+payload["item"]+", "+priority+", "+payload["qty"]+", "+payload["order_time"]+", "+payload["lon"]+", "+payload["lat"]+", "+cost
        self.incoPush(vav)
        orders = incobridge()
        orders.message = vav
        # msg_mqtt_sub = msgMqttSub()
        # msg_mqtt_sub.timestamp = rospy.Time.now()
        # msg_mqtt_sub.topic = message.topic
        # msg_mqtt_sub.message = payload
        self.orders_pub.publish(orders)
        print(orders)
        
    
    
    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == "mqtt"):
            
            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(  name="worker",
                                            target=self.process_goal,
                                            args=(goal_handle,) )
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        
        else:
            goal_handle.set_rejected()
            return


    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        
        # Goal Processing
        if(goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if(goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish( self._config_mqtt_server_url, 
                                        self._config_mqtt_server_port,
                                        goal.topic, 
                                        goal.message, 
                                        self._config_mqtt_qos   )

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        goal.topic, 
                                                        self._config_mqtt_qos   )
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    
    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


# Main
def main():
    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = RosIotBridgeActionServer()
    print(color.CYAN+"\tMQTT Pub Topic:  "+action_server._config_mqtt_pub_topic+color.END)
    print(color.CYAN+"\tMQTT Sub Topic:  "+action_server._config_mqtt_sub_topic+color.END)
    rospy.spin()



if __name__ == '__main__':
    main()