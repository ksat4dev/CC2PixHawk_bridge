// BACKGROUND INFO
// vrpn_client_ros is a ROS package that publishes the data streamed by Optitrack Motive over LAN on a topic - vrpn_client_node/<robot_name>/pose 
// This source code should serve three motives - a. subscribes to the data b. copies the contents of the message to zmq message queue 
//c. Creates ZMQ sockets, binds and publishes the same message that was subscribed suitable for ZMQ subscription by another module 

// ROS includes
#include "ros/ros.h"
// Includes for message type that is going to be published. PoseStamped includes 3D Pose of a rigid body at respective time stamps
#include "geometry_msgs/PoseStamped.h"
// ZMQ includes for messaging
#include "zhelpers.hpp"
#include<zmq.hpp>
// Local includes
#include<time.h>
#include<string.h>
#include<iostream>
#include<array>
// Include class definition for constructor invocation
#include "pub_socket.h"


pub_socket pub_sock;

pub_socket::pub_socket() : context(1), publisher(context,ZMQ_PUB), sub_context(1), subscriber(sub_context,ZMQ_SUB) {
    publisher.bind("tcp://192.168.1.3:3885");

}


void Callback(const geometry_msgs::PoseStamped & ps)
{
    // Print out received data
    ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n \t w:[%f] \n",
              ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
              ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);

   //while(1)
    //{
         // Declare a ZMQ message queue 
            zmq::message_t message(70);
    
    // Serialize (convert the data into a string) over zmq message
            snprintf((char *) message.data(),70, "%f %f %f %f %f %f %f", ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
                     ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);
    
    // Publish the message 
            pub_sock.publisher.send(message);
    
    // Acts as a publishing rate 10 sec
            sleep(10); 
    //}
    

}

int main(int argc, char **argv)
{

    // Initialize ROS and give a unique name for this node
    ros::init(argc,argv,"opti_sub");

    // Create a handle for this process' node
    ros::NodeHandle nh;

    // Create an class object to initialize the socket defaults once
    //pub_socket pub_sock;
    
    // Subscribe to the message of type "PoseStamped" published over the topic mentioned below in a buffer of size 1000
    ros::Subscriber sub = nh.subscribe("vrpn_client_node/Test_RB1/pose",1000, &Callback);

    // Keeps c++ from exiting until the node is stopped
    ros::spin();

    return 0;

}