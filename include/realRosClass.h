/*  CANYONERO  */


// ROS Controller

// MSG generation to communicate cameras and GPIOs

#ifndef REALROSCLASS_H
#define REALROSCLASS_H

// C++
#include <vector>
#include <string.h>
#include <iostream>
#include <curses.h>
// OpenCV
//#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// ROS_MSGS
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

class realRosClass
{
    private:

        // Private ROS variables
        ros::Publisher pub_direction;
        ros::Subscriber state_subscriber;
        image_transport::Subscriber sub;
        ros::Rate* rate;

        sensor_msgs::ImagePtr ros_img;
        cv::Mat frame;
        void streamCallback(const sensor_msgs::ImageConstPtr& img);
        void stateCallback(const std_msgs::String::ConstPtr& msg);

        string controller_name = "canyonero_ros_control_node";
        string stream_name = "canyonero_ros_streaming_node";
        int _Direction = 0;
        int speed = 50;
        WINDOW * win;
        int rate_error;

    public:

        // Constructor
        realRosClass();

        // Destructor
        ~realRosClass();

        // Public methods
        void start_control_node();
        void start_streaming_node();

        void show_streaming();
        string get_state();
        void send_command(int direc);
        void keyboard_control();
        
        void rosSpin();
        void rosSpinOnce();
        
        void start_ncurses();
        void end_ncurses();
        void info_control(string ss);

        // Public methods
        string state = "INIT";
        bool running;
};

#endif //REALROSCLASS_H