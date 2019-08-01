/*  CANYONERO  */


// ROS Controller

// Publish GPIO commands from keyboard control.

#include "realRosClass.h"


using namespace std;


int main(int argc, char** argv)
{
    // Start ROS
    realRosClass * realRos = new realRosClass();
    
    // Start ncurses
    realRos->start_ncurses();

    // Start nodes
    realRos->start_control_node();

    while(ros::ok())
    {   
        string _state = realRos->get_state();
        realRos->info_control(_state);
        realRos->keyboard_control();
        if(!realRos->running)
        {
            break;
        }
        realRos->rosSpinOnce();
    }

    // Remove window
    realRos->end_ncurses();

    // Shutdown
    delete realRos;

    return 0;
}