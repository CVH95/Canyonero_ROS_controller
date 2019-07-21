/*  CANYONERO  */


// ROS Controller

// MSG generation to communicate cameras and GPIOs

#include "realRosClass.h"


// Costructor
realRosClass::realRosClass(int argc, char** argv)
{
    int _argc=0;
	char** _argv = NULL;
	
	string nodename = "canyonero_ros_controller";
	
	ros::init(_argc, _argv, nodename);
	
	if(!ros::master::check())
		ROS_ERROR("ros::master::check() did not pass!");
		
	ros::NodeHandle node("~");
	ROS_INFO("ROS node started under the name %s.", nodename.c_str());

    //image_transport::ImageTransport it(node);
    
    // Start publisher:
    pub_direction = node.advertise<std_msgs::Int32>("/canyonero_ros_drivers/canyonero/joint_direction", 1);

    // Start subscriber:
    img_subscriber = node.subscribe("canyonero_ros_drivers/canyonero/camera/image_raw", 1, &realRosClass::streamCallback, this);
    //sub = it.subscribe("canyonero_ros_drivers/canyonero/camera/image_raw", 1, &realRosClass::streamCallback, this);

    rate = new ros::Rate(17*4);
}


// Destructor
realRosClass::~realRosClass()
{
    ROS_INFO("ROS node canyonero_ros_controller was terminated");
	ros::shutdown();
}


// Receive video stream (image transport)
void realRosClass::streamCallback(const sensor_msgs::ImageConstPtr& img)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        frame = cv_ptr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}


// Receive video stream
/*/void realRosClass::streamCallback(const sensor_msgs::Image& img)
{
    try
    {
        cv_bridge::CvImagePtr cv_img;
        cv_img = cv_bridge::toCvCopy(img, "bgr8");
        frame = cv_img->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from Image Msg to 'bgr8'.");
    }
}*/


// Show video stream
void realRosClass::show_streaming()
{
    cv::imshow("Canyonero Onboard View", frame);
    cv::waitKey(30);
}


// Publish the new direction
void realRosClass::send_command(int direc)
{
    std_msgs::Int32 d;
    d.data = direc;
    pub_direction.publish(d);
}


// ROS Keyboard Teleop
bool realRosClass::keyboard_control()
{
    int cht = 0;
	
	bool running = true;
	//int sp = dutyCycleValue;
	
	info_control();
	
	cht = getch();
	
	switch(cht)
	{
		case 'w':
			_Direction = 8;
			state = "FWRD";
			running = true;
			break;
		case 's':
			_Direction = 2;
			state = "BACK";
			running = true;
			break;
		case 'd':
			_Direction = 6;
			state = "RGHT";
			running = true;
			break;
		case 'a':
			_Direction = 4;
			state = "LEFT";
			running = true;
			break;
		case 'b':
			_Direction = 0;
			state = "STOP";
			running = true;
			break;
		case 'k':
			_Direction = 9;
			running = true;
			break;
		case 'j':
			_Direction = 7;
			running = true;
			break;
		case 'p':
			_Direction = 5;
            state = "STOP";
			move(15, 0);
			printw("Shutting down Canyonero");
			running = false;
			break;
	}
	
    // Publish command
    send_command(_Direction);

	// Update state on ncurses window
    wrefresh(win);

	return running;
}


// Handling ros::spin()
void realRosClass::rosSpin()
{
    ros::spin();
}


// Handling ros::spinOnce()
void realRosClass::rosSpinOnce()
{
    ros::spinOnce();
	bool rateMet = rate->sleep();
	
	if(!rateMet)
	{
		ROS_ERROR("Sleep rate not met!");
	}
}


// Handling creation of ncurses object
void realRosClass::start_ncurses()
{
    // Start ncurses
	initscr();
		
	// No echo (not printing keys pressed)
	noecho();
		
	// One key at a time
	cbreak();
	
	// Create window
	win = newwin(16, 50, 0, 0);
}


// Handling ending of ncurses object
void realRosClass::end_ncurses()
{
    endwin();
    cout << "Shutting down Canyonero's controller." << endl;
}


// Display message in ncurses window
void realRosClass::info_control()
{
    move(0,0);
	printw("... ROS Keyboard Teleoperation ...");
	move(2,0);
	printw("  >> Forward: w");
	move(3,0);
	printw("  >> Backward: s");
	move(4,0);
	printw("  >> Turn right (on spot): d");
	move(5,0);
	printw("  >> Turn left (on spot): a");
	move(6,0);
	printw("  >> Stop: b");
	move(7,0);
	printw("  >> Increase speed: k");
	move(8,0);
	printw("  >> Reduce speed: j");
	move(9,0);
	printw("  >> Exit program: p");
	move(13,0);
	printw("Direction = %s", state.c_str());
	move(14,0);
	printw("Current speed = %d (%)", speed);
}