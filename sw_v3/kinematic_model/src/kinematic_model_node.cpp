#include "ros/ros.h"
#include "kinematic_model/pos.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <math.h>
#include <sstream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h> 
#include <ctime>

#define length 0.65
#define pi 3.14159265359

sensor_msgs::Joy hold;
sensor_msgs::JointState hold_pos;

ros::Publisher theta1_publisher;
ros::Publisher theta2_publisher;
ros::Publisher theta3_publisher;
ros::Publisher theta4_publisher;

ros::Publisher pos_publisher;

float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;

int state = 0;

float x = 0;
float y = 0;

std_msgs::Float64 theta1p;
std_msgs::Float64 theta2p;
std_msgs::Float64 theta3p;
std_msgs::Float64 theta4p;

void get_joystick(const sensor_msgs::Joy::ConstPtr& msg);
void get_arm_point(const kinematic_model::pos::ConstPtr& msg);
void move_joystick(const sensor_msgs::Joy::ConstPtr& msg);
void move_arm_point(const kinematic_model::pos::ConstPtr& msg);
bool isButtonPressed(int i);

int main(int argc, char **argv){

    time_t current_time;

	current_time = time(NULL);

    ros::init(argc, argv, "kinematic_model");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/joy", 1000, get_joystick);

    ros::Subscriber sub2 = n.subscribe("/arm_point", 1000, get_arm_point);

    theta1_publisher = n.advertise<std_msgs::Float64>("/sw_v3/joint1_position_controller/command", 1000);
    theta2_publisher = n.advertise<std_msgs::Float64>("/sw_v3/joint2_position_controller/command", 1000);
    theta3_publisher = n.advertise<std_msgs::Float64>("/sw_v3/joint3_position_controller/command", 1000);
    theta4_publisher = n.advertise<std_msgs::Float64>("/sw_v3/joint4_position_controller/command", 1000);
    
    pos_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    ros::spin();

    //printf("test\n");
    return 0;
}
//----------------------------------------------------------------------------
//                              Moving with Joystick
//----------------------------------------------------------------------------
void get_joystick(const sensor_msgs::Joy::ConstPtr& msg){
    //std::cout << "He got here -2\n";
    move_joystick(msg);
    //ROS_INFO("I heard something: [%f]",hold_info.x);
}

void move_joystick(const sensor_msgs::Joy::ConstPtr& msg){
    float hyp;
    float hyp1;

    //std::cout << "He got here -1\n";
    hold.axes = msg->axes;
    hold.axes[0]=-hold.axes[0]/2;
    hold.axes[1]=hold.axes[1]/2;

    hold.axes[3]=-hold.axes[3]/8;
    hold.axes[4]=hold.axes[4]/8;

    hold.buttons = msg->buttons;

    if (isButtonPressed(2)){
        state++;
        if (state > 2){
        state = 0;
        }
        //ROS_INFO("%i",state);
    }
    
    switch(state){
        case 0://free mode
            ROS_INFO("-------Free Mode-------------");
            //ROS_INFO("axes_0: [%f]",hold.axes[0]);
            //ROS_INFO("axes_1: [%f]",hold.axes[1]);
            hyp = sqrt((hold.axes[0]*hold.axes[0])+(hold.axes[1]*hold.axes[1]));
            //ROS_INFO("%f",hyp);
            theta2 = 2*asin(hyp/length);
            theta2p.data = theta2;
            //std::cout << "He got here 1\n";
            ROS_INFO("I heard theta2: [%f]",theta2);
            if (hold.axes[0] > 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] < 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2) + pi;
            }
            else if (hold.axes[0] < 0 && hold.axes[1] < 0)
            {
            theta1 = pi + atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] > 0 && hold.axes[1] < 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            //std::cout << "He got here 2\n";
            theta1p.data = theta1;
            ROS_INFO("I heard theta1: [%f]",theta1);
            //----------------------------------------------------------------------------------
            hyp1 = sqrt((hold.axes[3]*hold.axes[3])+(hold.axes[4]*hold.axes[4]));
            theta4 = (pi*hyp1)/0.128;
            theta4p.data = theta4;
            //std::cout << "He got here 1\n";
            ROS_INFO("I heard theta4: [%f]",theta4);
            if (hold.axes[3] >= 0 && hold.axes[4] >= 0)
            {
            theta3 = atan(hold.axes[4]/hold.axes[3]) - ((pi-theta4)/2);
            }
            else if (hold.axes[3] < 0 && hold.axes[4] >= 0)
            {
            theta3 = atan(hold.axes[4]/hold.axes[3]) - ((pi-theta4)/2) + pi;
            }
            else if (hold.axes[3] < 0 && hold.axes[4] < 0)
            {
            theta3 = pi + atan(hold.axes[4]/hold.axes[3]) - ((pi-theta4)/2);
            }
            else if (hold.axes[3] >= 0 && hold.axes[4] < 0)
            {
            theta3 = atan(hold.axes[4]/hold.axes[3]) - ((pi-theta4)/2);
            }
            //std::cout << "He got here 2\n";
            theta3p.data = theta3;
            ROS_INFO("I heard theta3: [%f]",theta3);
        break;
        case 1://leveled mode
            ROS_INFO("---------Leveling Mode-----------");

            hyp = sqrt((hold.axes[0]*hold.axes[0])+(hold.axes[1]*hold.axes[1]));
            theta2 = 2*asin(hyp/length);

            if (hold.axes[0] >= 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] < 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2) + pi;
            }
            else if (hold.axes[0] < 0 && hold.axes[1] < 0)
            {
            theta1 = pi + atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] >= 0 && hold.axes[1] < 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            theta3 = pi;
            theta4 = -theta2;

            theta1p.data = theta1;
            theta2p.data = theta2;
            theta3p.data = theta3;
            theta4p.data = theta4;
        break;
        case 2://outwards mode
            ROS_INFO("---------Outwards Mode-----------");

            hyp = sqrt((hold.axes[0]*hold.axes[0])+(hold.axes[1]*hold.axes[1]));
            theta2 = 2*asin(hyp/length);

            if (hold.axes[0] >= 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] < 0 && hold.axes[1] >= 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2) + pi;
            }
            else if (hold.axes[0] < 0 && hold.axes[1] < 0)
            {
            theta1 = pi + atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            else if (hold.axes[0] >= 0 && hold.axes[1] < 0)
            {
            theta1 = atan(hold.axes[1]/hold.axes[0]) - ((pi-theta2)/2);
            }
            theta3 = 0.0;
            theta4 = -theta2;

            theta1p.data = theta1;
            theta2p.data = theta2;
            theta3p.data = theta3;
            theta4p.data = theta4;
        break;
    }
    
    //----------------------------------------------------------------------------------
    hold_pos.name.push_back("joint0");
    hold_pos.name.push_back("joint1");
    hold_pos.name.push_back("joint2");
    hold_pos.name.push_back("joint3");

    hold_pos.name.resize(4);
    hold_pos.position.resize(4);

    hold_pos.position[0] = theta1;
    hold_pos.position[1] = theta2;
    hold_pos.position[2] = theta3;
    hold_pos.position[3] = theta4;

    theta1_publisher.publish(theta1p);
    theta2_publisher.publish(theta2p);
    theta3_publisher.publish(theta3p);
    theta4_publisher.publish(theta4p);

    pos_publisher.publish(hold_pos);
}

bool isButtonPressed(int i){
  int prev_state = 0;
  bool result1 = false;
  int button_state = hold.buttons[i];
  if (button_state != prev_state)
  {
    if (button_state == 1)
    {
      result1 = true;
    }
    prev_state = button_state;
  }
  return result1;
}
//----------------------------------------------------------------------------
//                              Moving with /arm_point topic
//----------------------------------------------------------------------------
void get_arm_point(const kinematic_model::pos::ConstPtr& msg){
    //std::cout << "He got here -2\n";
    move_arm_point(msg);
    //ROS_INFO("I heard something: [%f]",hold_info.x);
}

void move_arm_point(const kinematic_model::pos::ConstPtr& msg){
    float hyp;
    float hyp1;

    //std::cout << "He got here -1\n";
    x = msg->x;
    y = msg->y;

    ROS_INFO("-------Free Mode-------------");
    //ROS_INFO("axes_0: [%f]",x);
    //ROS_INFO("axes_1: [%f]",y);
    hyp = sqrt((x*x)+(y*y));
    //ROS_INFO("%f",hyp);
    theta2 = 2*asin(hyp/length);
    theta2p.data = theta2;
    //std::cout << "He got here 1\n";
    ROS_INFO("I heard theta2: [%f]",theta2);
    if (x >= 0 && y >= 0)
    {
    theta1 = atan(y/x) - ((pi-theta2)/2);
    }
    else if (x < 0 && y >= 0)
    {
    theta1 = atan(y/x) - ((pi-theta2)/2) + pi;
    }
    else if (x < 0 && y < 0)
    {
    theta1 = pi + atan(y/x) - ((pi-theta2)/2);
    }
    else if (x >= 0 && y < 0)
    {
    theta1 = atan(y/x) - ((pi-theta2)/2);
    }
    //std::cout << "He got here 2\n";
    theta1p.data = theta1;
    ROS_INFO("I heard theta1: [%f]",theta1);
    //----------------------------------------------------------------------------------
    hold_pos.name.push_back("joint0");
    hold_pos.name.push_back("joint1");
    hold_pos.name.push_back("joint2");
    hold_pos.name.push_back("joint3");

    hold_pos.name.resize(4);
    hold_pos.position.resize(4);

    hold_pos.position[0] = theta1;
    hold_pos.position[1] = theta2;
    hold_pos.position[2] = theta3;
    hold_pos.position[3] = theta4;

    theta1_publisher.publish(theta1p);
    theta2_publisher.publish(theta2p);

    pos_publisher.publish(hold_pos);
}