/*  CANYONERO  */


// ROS Controller

// MSG generation to communicate with camera

#include "realRosClass.h"

using namespace std;

int main(int argc, char** argv)
{
    realRosClass * realRos = new realRosClass();

    realRos->start_streaming_node();

    //realRos->rosSpin();

    // Detele shit after spin is over
    delete realRos;

    return 0; 
}