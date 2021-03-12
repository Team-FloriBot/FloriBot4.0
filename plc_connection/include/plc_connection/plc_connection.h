#ifndef PLC_CONNECTION_OBJ_H
#define PLC_CONNECTION_OBJ_H

#include "network/udp/udp_socket.h"
#include <linux/if_link.h>
#include <ifaddrs.h>
#include <math.h>

#include <base/Wheels.h>
#include "plc_connection/GetCount.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h> 
#include <std_msgs/UInt32.h> 
#include <base/Angle.h>   

//Struct for receiving Data
struct FromPLC
{
    //Header
    uint32_t MessageID;
    uint32_t Mode;

    //Data
    float Speed[4];
    uint32_t Angle;
    float Voltage;
    uint32_t HomingError;
    uint32_t SpeedError[4];
    uint32_t ResetError[4];
    float Angle_rad;
};

//struct for sending data
struct ToPLC
{
    //Header
    uint32_t MessageID;
    uint32_t Mode;

    //Data
    float Speed[4];
    float Accelleration;
    float Jerk;
    float Dummy[4];
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
class plcConnectionNode
{
    public:
    //Constructor
    plcConnectionNode(OwnUDP::Address* OwnIP);
    plcConnectionNode(OwnUDP::Address* OwnIP, OwnUDP::Address* TargetIP);
    plcConnectionNode();

   
    
    private:
    //Initialize Subscriber/Publisher/Socket
    void Subscribe();
    void CreatePublisher();
    void InitializeSocket();

     //Read ROS-Param
    void ReadParams();

    //Timer Function
    void SendRecv(const ros::TimerEvent &e);
    
    //Send/receive Data
    void SendData();
    bool ReadData();
    void PublishData();

    //Callback for ServiceCount Callback
    bool GetCountService(plc_connection::GetCount::Request &req, plc_connection::GetCount::Response &res);

    //Callback for Speed subscriber 
    void SpeedCallback(const base::Wheels::ConstPtr& msg);

    //Callback for Mode subscriber
    void ModeCallback(const std_msgs::UInt32::ConstPtr& msg);


    //Callback for Torque subscriber
    void TorqueCallback(const std_msgs::Float64::ConstPtr& msg);
    
    //Callback for Accelaration subscriber
    void AccelerationCallback(const std_msgs::Float64::ConstPtr& msg);

    //Member
    //nodehandle
    ros::NodeHandle nh_;
    //Subscriber
    ros::Subscriber SpeedSubscriber_, ModeSubscriber_;
    //Publisher
    ros::Publisher SpeedPublisher_, AnglePublisher_;
    tf2_ros::TransformBroadcaster TFBroadcaster_;

    //Service
    ros::ServiceServer CountServer_;

    ros::Timer SendRecvTimer_;

    //Rosparams
    std::string strTargetIP_, strOwnIP_;
    uint TargetPort_, OwnPort_, ReceiveTimeoutSec_, ReceiveTimeoutUsec_;
    double PLCTimeout_;

    //Duration for Connection Timeout
    ros::Duration ConnectionTimeout_;

    //Params for Angle
    int zeroCount_;
    float countPerRotation_;
    double readWritePeriod_;

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
