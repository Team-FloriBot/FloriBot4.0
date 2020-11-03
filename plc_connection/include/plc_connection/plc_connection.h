#ifndef PLC_CONNECTION_OBJ_H
#define PLC_CONNECTION_OBJ_H

#include "network/udp/udp_socket.h"
#include <linux/if_link.h>
#include <ifaddrs.h>

#include <base/Wheels.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h> 
#include <base/Angle.h>   

//Struct for receiving Data
struct FromPLC
{
    uint32_t MessageID;
    uint32_t Mode;
    float Speed[4];
    float Angle;
};

//struct for sending data
struct ToPLC
{
    uint32_t MessageID;
    uint32_t Mode;
    float Speed[4];
    float Accelleration[4];
    float Torque[4];
};

//Struct for Exachanged Data PLC
struct PLC_Data
{
    FromPLC From;
    ToPLC To;
};

//struct for Target Device
struct TargetDevice
{
    OwnUDP::Address IP;
    uint64_t LastID;
    ros::Time LastMsgTime;
    bool ComOk;
};

//node class
//ToDo: Trigger operator() with Timer and then do not use operator()
class PlcConnectionNode
{
    public:
    //Constructor
    PlcConnectionNode(OwnUDP::Address* OwnIP);
    PlcConnectionNode(OwnUDP::Address* OwnIP, OwnUDP::Address* TargetIP);
    PlcConnectionNode();

    //Read ROS-Param
    void ReadParams();

    //Timer Function
    void SendRecv(const ros::TimerEvent &e);
    
    //Send/receive Data
    void SendData();
    bool ReadData();
    void PublishData();

    //Callback for Speed subscriber 
    void SpeedCallback(const base::Wheels::ConstPtr& msg);


    //Callback for Torque subscriber
    void TorqueCallback(const base::Wheels::ConstPtr& msg);
    
    //Callback for Accelaration subscriber
    void AccelerationCallback(const base::Wheels::ConstPtr& msg);
    
    private:
    //Initialize Subscriber/Publisher/Socket
    void Subscribe();
    void CreatePublisher();
    void InitializeSocket();

    //Member
    //nodehandle
    ros::NodeHandle nh_;
    //Subscriber
    ros::Subscriber SpeedSubscriber_, TorqueSubscriber_, AccelerationSubscriber_;
    //Publisher
    ros::Publisher SpeedPublisher_, AnglePublisher_;
    tf2_ros::TransformBroadcaster TransformBroadcaster_;

    ros::Timer SendRecvTimer_;

    //Strings for Rosparams
    std::string strTargetIP_, strOwnIP_;
    uint TargetPort_, OwnPort_, Mode_, ReceiveTimeoutSec_, ReceiveTimeoutUsec_;
    double PLCTimeout_;
    //Duration for Connection Timeout
    ros::Duration ConnectionTimeout_;

    //Connection ok status
    bool ComOk_;
    unsigned int seq_;

    //UDP-Socket
    OwnUDP::UDPSocket PLC_Socket_;
    //PLC Data to Exchange
    PLC_Data Data_;

    //Target plc data
    TargetDevice Target_;   
};

//PLC Data to network/host 
void ntohPLC(PLC_Data* Host, PLC_Data* Network);
void htonPLC(PLC_Data* Network, PLC_Data* Host);

#endif