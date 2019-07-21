/*  CANYONERO  */


// ROS Controller

// Receives the webCam streaming.
// Publish GPIO commands from keyboard control.

#include "realRosClass.h"

using namespace std;

realRosClass * realRos;

bool run_controller()
{
    if(ros::ok())
    {
        realRos->show_streaming();
        bool _cntr = realRos->keyboard_control();
        realRos->rosSpinOnce();
    }
    else
    {
        realRos->end_ncurses();
        cout << "Shutting down Canyonero's controller." << endl;
    }
}

int main(int argc, char** argv)
{
    // Start ROS
    realRos = new realRosClass(argc, argv);
    
    // Start ncurses
    realRos->start_ncurses();

    while(run_controller()){}

    delete realRos;

    return 0;
}