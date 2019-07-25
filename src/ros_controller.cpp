/*  CANYONERO  */


// ROS Controller

// Receives the webCam streaming.
// Publish GPIO commands from keyboard control.

#include "realRosClass.h"

using namespace std;

/*realRosClass * realRos;

bool run_controller()
{
    bool _cntr = true;

    if(ros::ok())
    {
        _cntr = realRos->keyboard_control();
        realRos->rosSpinOnce();
    }
    else
    {
        _cntr = false;
        realRos->end_ncurses();
        //cout << "Shutting down Canyonero's controller." << endl;
    }
    
    return _cntr; 
}*/

int main(int argc, char** argv)
{
    // Start ROS
    realRosClass * realRos = new realRosClass();
    
    // Start ncurses
    realRos->start_control_node();

    while(ros::ok())
    {
        realRos->keyboard_control();
        if(!realRos->running)
        {
            break;
        }
    }

    // Remove window
    realRos->end_ncurses();

    // Shutdown
    delete realRos;

    return 0;
}