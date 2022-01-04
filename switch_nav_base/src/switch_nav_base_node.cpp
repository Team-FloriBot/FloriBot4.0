#include "switch_nav_base/switch_nav_base.h"

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "switch_nav_base_node");
    ros::NodeHandle nh("~");
    SwitchNavBase switch_nav_base(&nh);
    ros::spin();
}


