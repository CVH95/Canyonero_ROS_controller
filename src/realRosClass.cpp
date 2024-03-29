/*  CANYONERO  */


// ROS Controller

// MSG generation to communicate with GPIOs

#include "realRosClass.h"


// Costructor
realRosClass::realRosClass()
{
    cout << "CANYONERO" << endl << endl;
	cout << "Creating a new node..." << endl << endl;
}


// Destructor
realRosClass::~realRosClass()
{
    ROS_INFO("ROS node was terminated");
	ros::shutdown();
}


void realRosClass::start_control_node()
{
	int _argc=0;
	char** _argv = NULL;
	
	ros::init(_argc, _argv, controller_name);
	
	if(!ros::master::check())
	{
		ROS_ERROR("ros::master::check() did not pass!");
	}
	else
	{
		const string _host = ros::master::getHost();
		const string _uri = ros::master::getURI();
		ROS_INFO("ROS Master Hostname: %s.", _host.c_str());
		ROS_INFO("ROS Master URI: %s/ \n", _uri.c_str());
	}
		
	ros::NodeHandle node("~");
	ROS_INFO("ROS node started under the name %s. \n", controller_name.c_str());

	// Start publisher:
    pub_direction = node.advertise<std_msgs::Int32>("/gpio_driver/joint_direction", 1);

	rate = new ros::Rate(17*4);
}


void realRosClass::start_streaming_node()
{
	int _argc=0;
	char** _argv = NULL;
	
	ros::init(_argc, _argv, stream_name);
	
	if(!ros::master::check())
	{
		ROS_ERROR("ros::master::check() did not pass!");
	}
	else
	{
		const string _host = ros::master::getHost();
		const string _uri = ros::master::getURI();
		ROS_INFO("ROS Master Hostname: %s.", _host.c_str());
		ROS_INFO("ROS Master URI: %s/ \n", _uri.c_str());
	}

	ros::NodeHandle node("~");
	ROS_INFO("ROS node started under the name %s. \n", stream_name.c_str());

	//cv::namedWindow("Canyonero Onboard View", CV_WINDOW_AUTOSIZE);
	//cv::startWindowThread();
	image_transport::ImageTransport it(node);

	//Start subscriber:
    image_transport::Subscriber img_subscriber = it.subscribe("/camera_driver/image_raw", 1, &realRosClass::streamCallback, this);
	state_subscriber = node.subscribe("/gpio_driver/state", 1, &realRosClass::stateCallback, this);
	rosSpin();

	//cv::destroyWindow("Canyonero Onboard View");
}


// Receive video stream (image transport)
void realRosClass::streamCallback(const sensor_msgs::ImageConstPtr& img)
{
	//ROS_INFO("Entered callback");
    try
    {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
		//cv::imshow("Canyonero Onboard View", cv_bridge::toCvShare(img, "bgr8")->image);
		cv::Mat fframe = cv_ptr->image;

		if(!fframe.empty())
		{
			//ROS_INFO("Received frame data.");
			cv::imshow("Canyonero Onboard View", fframe);
        	cv::waitKey(1);
		}
		else
		{
			ROS_ERROR("Did not receive any frame data.");
		}
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}


// Receive video stream
void realRosClass::stateCallback(const std_msgs::String::ConstPtr& msg)
{
   state = msg->data;
   ROS_INFO("%s", state.c_str());
}


// Show video stream
void realRosClass::show_streaming()
{
    cv::imshow("Canyonero Onboard View", frame);
    cv::waitKey(30);
}


string realRosClass::get_state()
{
	string s = state;
	return s;
}


// Publish the new direction
void realRosClass::send_command(int direc)
{
    std_msgs::Int32 d;
    d.data = direc;
    pub_direction.publish(d);
}


// ROS Keyboard Teleop
void realRosClass::keyboard_control()
{
	//start_ncurses();
	//wclear(win);

    int cht = 0;
	running = true;
	//string _st = get_state();
	//info_control(_st);
	cht = getch();
	
	switch(cht)
	{
		case 'w':
			_Direction = 8;
			//state = "FWRD";
			running = true;
			break;
		case 's':
			_Direction = 2;
			//state = "BACK";
			running = true;
			break;
		case 'd':
			_Direction = 6;
			//state = "RGHT";
			running = true;
			break;
		case 'a':
			_Direction = 4;
			//state = "LEFT";
			running = true;
			break;
		case 'b':
			_Direction = 0;
			//state = "STOP";
			running = true;
			break;
		case 'x':
			_Direction = 9;
			running = true;
			break;
		case 'z':
			_Direction = 7;
			running = true;
			break;
		case 'm':
			_Direction = 11;
			break;
		case 'i':
			_Direction = 21;
			running = true;
			break;
		case 'k':
			_Direction = 22;
			running = true;
			break;
		case 'l':
			_Direction = 23;
			running = true;
			break;
		case 'j':
			_Direction = 24;
			running = true;
			break;
		case 't':
			_Direction = 31;
			running = true;
			break;
		case 'y':
			_Direction = 32;
			running = true;
			break;
		case 'p':
			_Direction = 5;
            //state = "STOP";
			move(15, 0);
			printw("Shutting down Canyonero");
			running = false;
			break;
	}
	
    // Publish command
    send_command(_Direction);

	// Update state on ncurses window
	//refresh();
}


// Handling ros::spin()
void realRosClass::rosSpin()
{
	wrefresh(win);
    ros::spin();
}


// Handling ros::spinOnce()
void realRosClass::rosSpinOnce()
{
    //redrawwin(win);
	
	ros::spinOnce();
	bool rateMet = rate->sleep();

	//wclear(win);
	//string _s = get_state();
	//info_control(_s);
	
	
	if(!rateMet)
	{
		//ROS_ERROR("Sleep rate not met!");
		rate_error++;
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
	win = newwin(25, 50, 0, 0);
}


// Handling ending of ncurses object
void realRosClass::end_ncurses()
{
    endwin();
    cout << "Shutting down Canyonero's controller." << endl;
	cout << "Sleep rate errors: " << rate_error << endl;
}


// Display message in ncurses window
void realRosClass::info_control(string ss)
{
	wclear(win);
	move(0,0);
	printw("... Runing Keyboard Teleoperation ...");
	move(2,0);
	printw("MOTION:");
	move(4,0);
	printw("  >> Forward: w");
	move(5,0);
	printw("  >> Backward: s");
	move(6,0);
	printw("  >> Turn right (on spot): d");
	move(7,0);
	printw("  >> Turn left (on spot): a");
	move(8,0);
	printw("  >> Stop: b");
	move(9,0);
	printw("  >> Increase speed: x");
	move(10,0);
	printw("  >> Reduce speed: z");
	move(11,0);
	printw("  >> Switch OFF Canyonero: p");
	move(13,0);
	printw("VISION PLATFORM:");
	move(15,0);
	printw("  >> Unlock platform: t");
	move(16,0);
	printw("  >> Lock platform: y");
	move(17,0);
	printw("  >> Rotate up: i");
	move(18,0);
	printw("  >> Rotate Down: k");
	move(19,0);
	printw("  >> Rotate right: l");
	move(20,0);
	printw("  >> Rotate left:j");
	move(21,0);
	printw("  >> Lights: m");
}