#include "plc_connection/plc_connection.h"

//Standard constructor
PlcConnectionNode::PlcConnectionNode()
{
    seq_=0;

    //Read ROSparameter
    ReadParams();

    //Initialize Socket
    InitializeSocket();

    //Subscribe to Topics
    Subscribe();

    //Publish Topics
    CreatePublisher();

    //Run Functions time triggered
    SendRecvTimer_=nh_.createTimer(ros::Duration(0.1), &PlcConnectionNode::SendRecv, this);


    //ToDo: Initialize all
    Data_.From.Speed[0]=0;
    Data_.From.Speed[1]=0;
    Data_.From.Speed[2]=0;
    Data_.From.Speed[3]=0;
    Data_.From.Angle=M_PI;

    Data_.To.Accelleration[0]=0;
    Data_.To.Accelleration[1]=0;
    Data_.To.Accelleration[2]=0;
    Data_.To.Accelleration[3]=0;

    Data_.To.Torque[0]=0;
    Data_.To.Torque[1]=0;
    Data_.To.Torque[2]=0;
    Data_.To.Torque[3]=0;
    
}

//Initialize Socket
void PlcConnectionNode::InitializeSocket()
{
    //temporary variables
    OwnUDP::Address tmpAddress;

    //Set timeout for connection
    ConnectionTimeout_=ros::Duration(PLCTimeout_);
    ROS_INFO("PLC Timeout is set to %f seconds", PLCTimeout_);

    //Set plcmode
    Data_.To.Mode=Mode_;
    ROS_INFO("PLC Mode is %i", Mode_);

    //Set plc Address
    Target_.IP.IP=strTargetIP_;
    Target_.IP.Port=TargetPort_;
    ROS_INFO("PLC IP is %s:%i", strTargetIP_.c_str(), Target_.IP.Port);
    
    //Initialize Target data
    Target_.LastID=0;
    Target_.ComOk=false;
    Target_.LastID=0;
    Target_.LastMsgTime=ros::Time::now();

    tmpAddress.IP.clear();
    tmpAddress.IP=strOwnIP_;
    tmpAddress.Port=OwnPort_;

    //Create UDP Socket
    ROS_INFO("Creating UDP Socket");
    try
    {    
        //set IP-Address
        PLC_Socket_.bindAddress(&tmpAddress);
        ROS_INFO("UDP Socket created on Port %i", OwnPort_);

        //set receive Timeout
        PLC_Socket_.setReceiveTime(ReceiveTimeoutUsec_, ReceiveTimeoutSec_);
        ROS_INFO("Receive Timeout for UDP Socket ist set to %i seconds and %i usec", ReceiveTimeoutSec_, ReceiveTimeoutUsec_);
    }
    catch(const std::runtime_error* e)
    {
        ROS_ERROR("Error while creating UDP Socket on Port %i\n %s", OwnPort_, e->what());
        delete e;
        exit(-1);
    }

}

//Read ROSparameter 
void PlcConnectionNode::ReadParams()
{
    //Get Param from Paramserver or set Default Values
    nh_.param<std::string>("/"+ros::this_node::getName()+"/PLC_IP", strTargetIP_, "192.168.0.43");
    nh_.param<std::string>("/"+ros::this_node::getName()+"/Xavier_IP", strOwnIP_, ""); 

    TargetPort_=nh_.param("/"+ros::this_node::getName()+"/PLC_Port", 5000);
    OwnPort_=nh_.param("/"+ros::this_node::getName()+"/Xavier_Port", 5000);  
    Mode_=nh_.param("/"+ros::this_node::getName()+"/Engine_Mode", 0);
    PLCTimeout_=nh_.param("/"+ros::this_node::getName()+"/PLC_Timeout", 1.5);
    ReceiveTimeoutSec_=nh_.param("/"+ros::this_node::getName()+"/Receive_Timeout_sec", 0);
    ReceiveTimeoutUsec_=nh_.param("/"+ros::this_node::getName()+"/Receive_Timeout_usec", 500);
}

//Subscribe to topics
void PlcConnectionNode::Subscribe()
{
    //Create Subscriber 
    SpeedSubscriber_=nh_.subscribe("Engine/TargetSpeed", 1, &PlcConnectionNode::SpeedCallback, this);
    
    AccelerationSubscriber_=nh_.subscribe("Engine/TargetAcceleration", 1, &PlcConnectionNode::AccelerationCallback, this);
    
    TorqueSubscriber_=nh_.subscribe("Engine/TargetTorque", 1, &PlcConnectionNode::TorqueCallback, this);
}

//create Publisher
void PlcConnectionNode::CreatePublisher()
{
    //create Publisher
    SpeedPublisher_=nh_.advertise<base::Wheels>("Engine/ActualSpeed", 1);
    
    AnglePublisher_=nh_.advertise<base::Angle>("Sensors/BodyAngle", 1);
}

//Callbacks for Subscriber
void PlcConnectionNode::SpeedCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Speed[0]=msg->FrontLeft;
    Data_.To.Speed[1]=msg->FrontRight;
    Data_.To.Speed[2]=msg->RearLeft;
    Data_.To.Speed[3]=msg->RearLeft;
}

void PlcConnectionNode::TorqueCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Torque[0]=msg->FrontLeft;
    Data_.To.Torque[1]=msg->FrontRight;
    Data_.To.Torque[2]=msg->RearLeft;
    Data_.To.Torque[3]=msg->RearLeft;
}

void PlcConnectionNode::AccelerationCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Accelleration[0]=msg->FrontLeft;
    Data_.To.Accelleration[1]=msg->FrontRight;
    Data_.To.Accelleration[2]=msg->RearLeft;
    Data_.To.Accelleration[3]=msg->RearLeft;
}

//Cyclic Function 
//ToDo: Add Error Handling in Protocol
void PlcConnectionNode::SendRecv(const ros::TimerEvent &Time)
{
    //Send and receive Data
    try
    {
        SendData();
        ReadData();    
    }
    catch(std::runtime_error* e)
    {
        ROS_WARN("%s",e->what());
    }

    //Publish data
    PublishData();    
}

//send data
void PlcConnectionNode::SendData()
{
    //prepare data
    PLC_Data tmpData;
    htonPLC(&tmpData, &Data_);

    //send data
    PLC_Socket_.write((uint8_t*) &tmpData.To, sizeof(Data_.To), &Target_.IP);
}

//receive data
//ToDo: Change for not receiving Data
bool PlcConnectionNode::ReadData()
{
    //create teomporary variables
    PLC_Data tmpData;
    OwnUDP::Address tmpAddress;

    //read received data
    PLC_Socket_.read((uint8_t*) &tmpData.From, sizeof(Data_.From), &tmpAddress);
    
    //Check received data
    if(tmpAddress.IP==Target_.IP.IP && tmpAddress.Port==Target_.IP.Port)
    {
        //Check if it is a new message
        if (ntohl(tmpData.From.MessageID)==Target_.LastID)
        {  
            //Check for connection timeout
            if ((ros::Time::now() - Target_.LastMsgTime).toSec()>ConnectionTimeout_.toSec())
            {
                if (Target_.ComOk==true)
                {
                    ROS_ERROR("No Connection to PLC");
                } 
                Target_.ComOk=false;
            }
            return false;
        }

        //write data for host
        ntohPLC(&Data_, &tmpData);

        Target_.ComOk=true;
        Target_.LastID=Data_.From.MessageID;
        Target_.LastMsgTime=ros::Time::now();

        return true;
    }

    return false;
}

//Publish received Data
void PlcConnectionNode::PublishData()
{
    //Create messages
    base::Angle  AngleMsg;
    base::Wheels SpeedMsg;
    geometry_msgs::TransformStamped TransformMsg;
    tf2::Quaternion q;

    TransformMsg.child_frame_id="JointRear";
    TransformMsg.header.frame_id="JointFront";
    TransformMsg.header.seq=seq_;
    TransformMsg.header.stamp=ros::Time::now();

    //write data in messages and publish
    SpeedMsg.header.seq=seq_;
    SpeedMsg.header.stamp=ros::Time::now();

    AngleMsg.header.seq=seq_++;
    AngleMsg.header.stamp=ros::Time::now();

    SpeedMsg.FrontLeft=Data_.From.Speed[0];
    SpeedMsg.FrontRight=Data_.From.Speed[1];
    SpeedMsg.RearLeft=Data_.From.Speed[2];
    SpeedMsg.RearRight=Data_.From.Speed[3];

    AngleMsg.Angle=Data_.From.Angle;

    q.setRPY(0,0,Data_.From.Angle);
    TransformMsg.transform.translation.x=0;
    TransformMsg.transform.translation.y=0;
    TransformMsg.transform.translation.z=0;
    TransformMsg.transform.rotation.w=q.w();
    TransformMsg.transform.rotation.x=q.x();
    TransformMsg.transform.rotation.y=q.y();
    TransformMsg.transform.rotation.z=q.z();


    SpeedPublisher_.publish(SpeedMsg);

    AnglePublisher_.publish(AngleMsg);

    TransformBroadcaster_.sendTransform(TransformMsg);
}

//Write data for network
void ntohPLC(PLC_Data* Host, PLC_Data* Network)
{
    Host->From.MessageID=ntohl(Network->From.MessageID);
    Host->From.Mode=ntohl(Network->From.Mode);
    Host->From.Angle=OwnSocket::ntohf(Network->From.Angle);
    
    for (int i=0;i<4;i++)  Host->From.Speed[i]=OwnSocket::ntohf(Network->From.Speed[i]);
}

//write data for host
void htonPLC(PLC_Data* Network, PLC_Data* Host)
{
    Network->To.MessageID=htonl(Host->To.MessageID++);
    Network->To.Mode=htonl(Host->To.Mode);
    
    for (int i=0; i<4;i++)
    {
        Network->To.Speed[i]=OwnSocket::htonf(Host->To.Speed[i]);
        Network->To.Torque[i]=OwnSocket::htonf(Host->To.Torque[i]);
        Network->To.Accelleration[i]=OwnSocket::htonf(Host->To.Accelleration[i]);
    }
}