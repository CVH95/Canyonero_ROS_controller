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

using namespace std;
namespace enc = sensor_msgs::image_encodings;

class realRosClass
{
    private:

        // Private ROS variables
        ros::Publisher pub_direction;
        //ros::Subscriber img_subscriber;
        image_transport::Subscriber sub;
        ros::Rate* rate;

        sensor_msgs::ImagePtr ros_img;
        cv::Mat frame;
        void streamCallback(const sensor_msgs::ImageConstPtr& img);
        //void streamCallback(const sensor_msgs::Image& img);

        string controller_name = "canyonero_ros_control_node";
        string stream_name = "canyonero_ros_streaming_node";
        int _Direction = 0;
        int speed = 50;
        string state = "STOP";
        WINDOW * win;

    public:

        // Constructor
        realRosClass();

        // Destructor
        ~realRosClass();

        // Public methods
        void start_control_node();
        void start_streaming_node();

        void show_streaming();
        void send_command(int direc);
        void keyboard_control();
        
        void rosSpin();
        void rosSpinOnce();
        
        void start_ncurses();
        void end_ncurses();
        void info_control();

        // Public methods
        bool running;
};

#endif //REALROSCLASS_H