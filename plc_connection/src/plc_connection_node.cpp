#include "plc_connection/plc_connection.h"
#include"network/udp/udp_socket.h"

void ExitFcn();

int main(int argc, char** argv)
{

    ros::init(argc, argv, "PLC_Connection");
    std::atexit(ExitFcn);
    ros::NodeHandle nh;
    try
    {
        PlcConnectionNode PlcConnection;
        ros::Rate r(10);

        while (ros::ok())
        {
            PlcConnection();
            ros::spinOnce();
            r.sleep();
        }
    }
    //Log Error before exiting node with error
    catch(const std::exception* e)
    {
        ROS_ERROR("Exiting with error:\n%s\n", e->what());
        std::cerr << e->what() << '\n';
        delete e;
        exit(-1);
    }
    
    return 0;
}

void ExitFcn()
{
    ROS_ERROR("Exiting Node: %s \n", ros::this_node::getName().c_str());
}