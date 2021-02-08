#include <pathfollowing_node/pathfollowing.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathFollowingControleNode");
    ros::NodeHandle nh;

    PathFollowingControl pfc;
    pfc.initialise(nh,0.1,0.1,1);

    ros::Time previous=ros::Time::now();

    ros::Rate rate(140.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
