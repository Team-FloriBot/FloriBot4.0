#include "network/udp/udp_socket.h"

//Standard constructor
 OwnUDP::UDPSocket::UDPSocket()
{
    //Initialize Socket and get assigned address
    OwnSocket::Socket::init(OwnSocket::Connection::UDP);
}

//Constructor with specified address
 OwnUDP::UDPSocket::UDPSocket(Address* OwnIP)
{
    //Initialize Socket and set IP-Address
    OwnSocket::Socket::init(OwnSocket::Connection::UDP);
    OwnUDP::UDPSocket::bindAddress(OwnIP);
}

//assign IP-Address
 void OwnUDP::UDPSocket::bindAddress(Address* OwnIP)
{
    //Bind address
    OwnSocket::Socket::bindAddress(OwnIP);
}

//Send Data
void OwnUDP::UDPSocket::write(uint8_t* Data, int length, Address* IP)
{
    OwnSocket::Socket::write(Data, length, IP);   
}

//read Data
void OwnUDP::UDPSocket::read(uint8_t* Data, int length, Address* IP)
{
    OwnSocket::Socket::read(Data, length, IP);
}

//Return own IP-Address
void OwnUDP::UDPSocket::getAddress(Address* IP)
{
    IP->IP.clear();
    IP->IP+=OwnAddress_.IP;
    IP->Port=OwnAddress_.Port;
}

void OwnUDP::UDPSocket::setReceiveTime(int usec, int sec)
{
    OwnSocket::Socket::setReceiveTime(usec, sec);
}

//Destructor
OwnUDP::UDPSocket::~UDPSocket()
{}