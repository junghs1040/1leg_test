#include <iostream>
#include <vector>
#include <string>

#include "sensor_msgs/JointState.h"
#include "urdf/model.h"
#include <robot_node.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "leg1_project");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");
    
    
    Leg1Project leg(&nh, &nh_priv);

    ros::spin();
    return 0;
}
