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
    SendRecvTimer_=nh.createTimer(ros::Duration(0.1), &PlcConnectionNode::SendRecv, this);


    //ToDo: Initialize all
    Data.From.Speed[0]=0;
    Data.From.Speed[1]=0;
    Data.From.Speed[2]=0;
    Data.From.Speed[3]=0;
    Data.From.Angle=M_PI;

    Data.To.Accelleration[0]=0;
    Data.To.Accelleration[1]=0;
    Data.To.Accelleration[2]=0;
    Data.To.Accelleration[3]=0;

    Data.To.Torque[0]=0;
    Data.To.Torque[1]=0;
    Data.To.Torque[2]=0;
    Data.To.Torque[3]=0;
    
}

//Initialize Socket
void PlcConnectionNode::InitializeSocket()
{
    //temporary variables
    OwnUDP::Address tmpAddress;

    //Set timeout for connection
    ConnectionTimeout=ros::Duration(PLCTimeout);
    ROS_INFO("PLC Timeout is set to %f seconds", PLCTimeout);

    //Set plcmode
    Data.To.Mode=Mode;
    ROS_INFO("PLC Mode is %i", Mode);

    //Set plc Address
    Target.IP.IP=strTargetIP;
    Target.IP.Port=TargetPort;
    ROS_INFO("PLC IP is %s:%i", strTargetIP.c_str(), Target.IP.Port);
    
    //Initialize Target data
    Target.LastID=0;
    Target.ComOk=false;
    Target.LastID=0;
    Target.LastMsgTime=ros::Time::now();

    tmpAddress.IP.clear();
    tmpAddress.IP=strOwnIP;
    tmpAddress.Port=OwnPort;

    //Create UDP Socket
    ROS_INFO("Creating UDP Socket");
    try
    {    
        //set IP-Address
        PLC_Socket.bindAddress(&tmpAddress);
        ROS_INFO("UDP Socket created on Port %i", OwnPort);

        //set receive Timeout
        PLC_Socket.setReceiveTime(ReceiveTimeoutUsec, ReceiveTimeoutSec);
        ROS_INFO("Receive Timeout for UDP Socket ist set to %i seconds and %i usec", ReceiveTimeoutSec, ReceiveTimeoutUsec);
    }
    catch(const std::runtime_error* e)
    {
        ROS_ERROR("Error while creating UDP Socket on Port %i\n %s", OwnPort, e->what());
        delete e;
        exit(-1);
    }

}

//Read ROSparameter 
void PlcConnectionNode::ReadParams()
{
    //Get Param from Paramserver or set Default Values
    nh.param<std::string>("/"+ros::this_node::getName()+"/PLC_IP", strTargetIP, "192.168.0.43");
    nh.param<std::string>("/"+ros::this_node::getName()+"/Xavier_IP", strOwnIP, ""); 

    TargetPort=nh.param("/"+ros::this_node::getName()+"/PLC_Port", 5000);
    OwnPort=nh.param("/"+ros::this_node::getName()+"/Xavier_Port", 5000);  
    Mode=nh.param("/"+ros::this_node::getName()+"/Engine_Mode", 0);
    PLCTimeout=nh.param("/"+ros::this_node::getName()+"/PLC_Timeout", 1.5);
    ReceiveTimeoutSec=nh.param("/"+ros::this_node::getName()+"/Receive_Timeout_sec", 0);
    ReceiveTimeoutUsec=nh.param("/"+ros::this_node::getName()+"/Receive_Timeout_usec", 500);
}

//Subscribe to topics
void PlcConnectionNode::Subscribe()
{
    //Create Subscriber 
    SpeedSubscriber=nh.subscribe("Engine/TargetSpeed", 1, &PlcConnectionNode::SpeedCallback, this);
    
    AccelerationSubscriber=nh.subscribe("Engine/TargetAcceleration", 1, &PlcConnectionNode::AccelerationCallback, this);
    
    TorqueSubscriber=nh.subscribe("Engine/TargetTorque", 1, &PlcConnectionNode::TorqueCallback, this);
}

//create Publisher
void PlcConnectionNode::CreatePublisher()
{
    //create Publisher
    SpeedPublisher=nh.advertise<base::Wheels>("Engine/ActualSpeed", 1);
    
    AnglePublisher=nh.advertise<base::Angle>("Sensors/BodyAngle", 1);
}

//Callbacks for Subscriber
void PlcConnectionNode::SpeedCallback(const base::Wheels::ConstPtr& msg)
{
    Data.To.Speed[0]=msg->FrontLeft;
    Data.To.Speed[1]=msg->FrontRight;
    Data.To.Speed[2]=msg->RearLeft;
    Data.To.Speed[3]=msg->RearLeft;
}

void PlcConnectionNode::TorqueCallback(const base::Wheels::ConstPtr& msg)
{
    Data.To.Torque[0]=msg->FrontLeft;
    Data.To.Torque[1]=msg->FrontRight;
    Data.To.Torque[2]=msg->RearLeft;
    Data.To.Torque[3]=msg->RearLeft;
}

void PlcConnectionNode::AccelerationCallback(const base::Wheels::ConstPtr& msg)
{
    Data.To.Accelleration[0]=msg->FrontLeft;
    Data.To.Accelleration[1]=msg->FrontRight;
    Data.To.Accelleration[2]=msg->RearLeft;
    Data.To.Accelleration[3]=msg->RearLeft;
}

//Operator() 
//ToDo: Add Error Handling in Protocol
void PlcConnectionNode::SendRecv(const ros::TimerEvent &e)
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
        delete e;
    }

    //Publish data
    PublishData();    
}

//send data
void PlcConnectionNode::SendData()
{
    //prepare data
    PLC_Data tmpData;
    htonPLC(&tmpData, &Data);

    //send data
    PLC_Socket.write((uint8_t*) &tmpData.To, sizeof(Data.To), &Target.IP);
}

//receive data
bool PlcConnectionNode::ReadData()
{
    //create teomporary variables
    PLC_Data tmpData;
    OwnUDP::Address tmpAddress;

    //read received data
    PLC_Socket.read((uint8_t*) &tmpData.From, sizeof(Data.From), &tmpAddress);
    
    //Check received data
    if(tmpAddress.IP==Target.IP.IP && tmpAddress.Port==Target.IP.Port)
    {
        //Check if it is a new message
        if (ntohl(tmpData.From.MessageID)==Target.LastID)
        {  
            //Check for connection timeout
            if ((ros::Time::now() - Target.LastMsgTime).toSec()>ConnectionTimeout.toSec())
            {
                if (Target.ComOk==true)
                {
                    ROS_ERROR("No Connection to PLC");
                } 
                Target.ComOk=false;
            }
            return false;
        }

        //write data for host
        ntohPLC(&Data, &tmpData);

        Target.ComOk=true;
        Target.LastID=Data.From.MessageID;
        Target.LastMsgTime=ros::Time::now();

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
    geometry_msgs::TransformStamped TFAngleMsg;
    tf2::Quaternion q;

    //write data in messages and publish
    TFAngleMsg.header.seq=seq_;
    TFAngleMsg.child_frame_id="jointRear";
    TFAngleMsg.header.frame_id="jointFront";
    TFAngleMsg.header.stamp=ros::Time::now();

    SpeedMsg.header.seq=seq_;
    SpeedMsg.header.stamp=ros::Time::now();

    AngleMsg.header.seq=seq_++;
    AngleMsg.header.stamp=ros::Time::now();

    q.setRPY(0,0,Data.From.Angle);
    TFAngleMsg.transform.translation.x=0;
    TFAngleMsg.transform.translation.y=0;
    TFAngleMsg.transform.translation.z=0;

    TFAngleMsg.transform.rotation.x=q.x();
    TFAngleMsg.transform.rotation.y=q.y();
    TFAngleMsg.transform.rotation.z=q.z();
    TFAngleMsg.transform.rotation.w=q.w();


    SpeedMsg.FrontLeft=Data.From.Speed[0];
    SpeedMsg.FrontRight=Data.From.Speed[1];
    SpeedMsg.RearLeft=Data.From.Speed[2];
    SpeedMsg.RearRight=Data.From.Speed[3];

    AngleMsg.Angle=Data.From.Angle;

    TFBroadcaster.sendTransform(TFAngleMsg);

    SpeedPublisher.publish(SpeedMsg);

    AnglePublisher.publish(AngleMsg);
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