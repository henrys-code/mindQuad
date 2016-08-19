#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "robot");
    NodeHandle n;
    Rate r(10);
    return 0;
}
