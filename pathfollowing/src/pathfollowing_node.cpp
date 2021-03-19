#include <pathfollowing_node/pathfollowing.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathFollowingControleNode");
    ros::NodeHandle nh;

    PathFollowingControl pfc;
    pfc.initialise(nh,0.1,0.1,1,"odom","base_link");

    ROS_INFO("Initialize Path following controle node");
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        pfc.runControler();
        pfc.publish(true);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
