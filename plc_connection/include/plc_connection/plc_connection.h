#ifndef PLC_CONNECTION_OBJ_H
#define PLC_CONNECTION_OBJ_H

#include "network/udp/udp_socket.h"
#include <linux/if_link.h>
#include <ifaddrs.h>

#include <base/Wheels.h>
#include <ros/ros.h>
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
    ros::NodeHandle nh;
    //Subscriber
    ros::Subscriber SpeedSubscriber, TorqueSubscriber, AccelerationSubscriber;
    //Publisher
    ros::Publisher SpeedPublisher, AnglePublisher;

    ros::Timer SendRecvTimer_;

    //Strings for Rosparams
    std::string strTargetIP, strOwnIP;
    uint TargetPort, OwnPort, Mode, ReceiveTimeoutSec, ReceiveTimeoutUsec;
    double PLCTimeout;
    //Duration for Connection Timeout
    ros::Duration ConnectionTimeout;

    //Connection ok status
    bool ComOk;
    unsigned int seq_;

    //UDP-Socket
    OwnUDP::UDPSocket PLC_Socket;
    //PLC Data to Exchange
    PLC_Data Data;

    //Target plc data
    TargetDevice Target;   
};

//PLC Data to network/host 
void ntohPLC(PLC_Data* Host, PLC_Data* Network);
void htonPLC(PLC_Data* Network, PLC_Data* Host);

#endif