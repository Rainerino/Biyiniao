//
// Created by yiyi on 18/07/18.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <string>

mavros_msgs::State current_state;
sensor_msgs::Range current_height;

float MAX_ERROR = 0.05;
float DESIRED_ATIITUDE = 0.3;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void ul_sensor_cb(const sensor_msgs::Range::ConstPtr& msg){
    //ROS_INFO("Current height is : %f", (msg->range));
    current_height = *msg;
}

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "ul_attitude_control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber ul_sensor_sub = nh.subscribe<sensor_msgs::Range>("ultrasonic_sensor", 10, ul_sensor_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    ros::Rate rate(20.0);

    //Connect to FCU
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.3; // in meter

    for(int i = 100; ros::ok() && i > 0; i --){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = static_cast<unsigned char>(true);

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //Try to hover 0.3 meter above the ground

        if(current_height.range > DESIRED_ATIITUDE + MAX_ERROR || current_height.range < DESIRED_ATIITUDE - MAX_ERROR ){

            // add a pid controller:
            float KP = 0.5;
            float error = 0.5 * (DESIRED_ATIITUDE - current_height.range);
            pose.pose.position.z = current_height.range + error;
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();

        rate.sleep();
    }
    return 0;


}
