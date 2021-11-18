#include "switch_base_link/switch_base_link.h"

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "switch_base_link_node");
    ros::NodeHandle nh("~");
    SwitchBaseLink sbl = SwitchBaseLink(&nh);
    ros::spin();
}


