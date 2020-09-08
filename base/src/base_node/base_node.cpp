#include "base_node/base_publisher.h"

void ExitFcn();

int main(int argc, char** argv)
{
    std::atexit(ExitFcn);
    ros::init(argc, argv, "Kinematics");
    
    ros::NodeHandle Nh;

    KinematicsPublisher Pub(&Nh, kinematics::coordinate::Front);
    
    try
    {
        ros::spin();
    }
    catch( std::runtime_error* e)
    {
        ROS_ERROR("Exiting with error:\n%s\n", e->what());
        exit(1);
    }
    return 0;
}

void ExitFcn()
{
    ROS_ERROR("Exiting Node: %s \n", ros::this_node::getName().c_str());
}