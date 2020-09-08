#include "network/socket/socket.h"

//Standardconstructor
OwnSocket::Socket::Socket()
{   
    //Initialize Socket
    _SocketID=-1;
    _Passive=false;
    _Connected=false;
}

//Constructor with Connectiontype
OwnSocket::Socket::Socket(Connection Type)
{
    //Initialize Values
    _SocketID=-1;
    _Passive=false;
    _Connected=false;

    //Initialize Socket
    init(Type);
}

//Destructor
OwnSocket::Socket::~Socket()
{
    //close Socket before deleting object
    closeSocket();
}

//initialize Socket
void OwnSocket::Socket::init(Connection Type)
{
    //create temporary variables
    struct sockaddr_in ownAddr;
    socklen_t len;

    //Check if Socket is already initialized
    if(_SocketID>0) closeSocket();

    //Initialize Socket depending on Connectiontype
    switch (Type)
    {
        case TCP:
            _SocketID=socket(AF_INET, SOCK_STREAM, 0);
            break;
        case UDP:
            _SocketID=socket(AF_INET, SOCK_DGRAM, 0);
            break;
    }

    //Check if Socket was created succesfull
    if (_SocketID <= 0) throw new std::runtime_error("Socket is not valid");

    //Get Socket Data from System
    getsockname(_SocketID, (sockaddr*)&ownAddr, &len);

    //Safe Socket Data
    OwnAddress.IP.clear();
    OwnAddress.IP+=inet_ntoa(ownAddr.sin_addr);
    OwnAddress.Port=ntohs(ownAddr.sin_port);

    //Safe Socket Type
    _Type=Type;
}

//Set Timeout for receiving Data
void OwnSocket::Socket::setReceiveTime(int usec, int sec)
{
    //Create temporary Variables
    struct timeval tv;

    //Set Time Variable
    tv.tv_sec = sec;       
    tv.tv_usec = usec; 
    //Set timeout for Socket     
    if (setsockopt(_SocketID, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval))<0) throw new std::runtime_error("Can not set Timeout");

}

//Connect Socket with target
void OwnSocket::Socket::connectTo(Address* IP)
{
    //create temporary variables
    struct sockaddr_in server;

    if (!checkAddress(IP)) throw new std::runtime_error("Invalid Addressparameter");
    //Set Server Connection data
    server.sin_family=AF_INET;
	server.sin_addr.s_addr=inet_addr(IP->IP.c_str());
	server.sin_port=htons(IP->Port);

    //Connect to Server
    if (connect(_SocketID,(struct sockaddr *) &server, sizeof(server)<0)) throw new std::runtime_error("Can not Connect to IP "+IP->IP);
    
    else _Connected=true;
}

//Close socket
void OwnSocket::Socket::closeSocket()
{
    //Check if Socket is connected
    if (_SocketID>0)
    {  
        //close Socket
        close(_SocketID);

        //reset Socket data
        OwnAddress.Port=0;
        OwnAddress.IP.clear();
        _SocketID=0;
	    _Connected=false;
    }
}

//Check if Socket is connected
bool OwnSocket::Socket::connected()
{
    return _Connected;
}

//Check if SocketID is valid
bool OwnSocket::Socket::socketOk()
{
    return (_SocketID>0);
}

//write Data
void OwnSocket::Socket::write(uint8_t* Data, int length, Address* Target=NULL)
{
    //Check Socket status  
    if (_Passive) throw new std::runtime_error("Writing on passive Socket");
    if (!socketOk()) throw new std::runtime_error("Socket is not valid");

    //Check if connected
    if (_Connected)
    {
        //Send Data
        if (send(_SocketID, Data, length,0)<0) throw new std::runtime_error("Sending Data failed");
    }
    else
    {
        //Check if it should be connected
        if (_Type==TCP) throw new std::runtime_error("TCP Socket has to be connected to send Data");

        //Check Address to send to
        if (Target->IP.empty()||!checkAddress(Target)) throw new std::runtime_error("No vaild IP-Address to send Data to");

        struct sockaddr_in target;

        //Set Data for Target
        target.sin_family=AF_INET;
	    target.sin_addr.s_addr=inet_addr(Target->IP.c_str());
	    target.sin_port=htons(Target->Port);

        //Send data
        if (sendto(_SocketID, Data, length,0, (struct sockaddr*)&target, sizeof(target))<0) throw new std::runtime_error("Error while sending Data");
    }   
}

//Read Data on Socket
void OwnSocket::Socket::read(uint8_t* Data, int length, Address* Source=NULL)
{
    //Check Socket
    if (!socketOk()) throw new std::runtime_error("Socket is not valid");

    //Check if an active connection is needed to receive messages
    if (_Type==TCP && !_Connected) throw new std::runtime_error("Socket has to be connected to receive Data");

    socklen_t len;
    struct sockaddr_in source;

    //receive Data and get Sourceaddress
    if (recvfrom(_SocketID, Data, length,0, (struct sockaddr*)&source, &len)<0) throw new std::runtime_error("Error while receiving Data"); 

    //Write Sourceaddress data for further use
    if (Source!=NULL)
    {
        Source->IP.clear();
        Source->IP+=inet_ntoa(source.sin_addr);
        Source->Port=ntohs(source.sin_port);
    }
}

//Get Own Address 
void OwnSocket::Socket::getAddress(Address* Target)
{
    Target->IP.clear();
    Target->IP+=OwnAddress.IP;
    Target->Port=OwnAddress.Port;
}

//Bind specified IP-Address and Port
void OwnSocket::Socket::bindAddress(Address* IPParam)
{
    struct sockaddr_in Addr;

    if (!checkAddress(IPParam)) throw new std::runtime_error("Invalid IP Parameter Address: "+ IPParam->IP+ " Port: "+ std::to_string(IPParam->Port));

    //Prepare Data to set Address data
    Addr.sin_family=AF_INET;

    if(IPParam->IP.empty())
        Addr.sin_addr.s_addr=INADDR_ANY;
    else
	    Addr.sin_addr.s_addr=inet_addr(IPParam->IP.c_str());
    
    Addr.sin_port=htons(IPParam->Port);

    //Bind Address to Socket
    if (bind(_SocketID, (sockaddr*)&Addr, sizeof(Addr))<0) 
                throw new std::runtime_error("Error while binding Address "+ IPParam->IP+ " with Port " + std::to_string(IPParam->Port));
    else
    {
        //Change saved Socket data
        OwnAddress.IP.clear();
        OwnAddress.IP+=IPParam->IP;
        OwnAddress.Port=IPParam->Port;
    }   
}

//Accept incoming connection request
OwnSocket::Socket* OwnSocket::Socket::acceptConnection()
{
    //Create temporary variables
    struct sockaddr_in NewSock;
    socklen_t len=sizeof(NewSock);
    int newID;

    //Accept connection
    if (newID=accept(_SocketID, (sockaddr*)&NewSock, &len)<0) throw new std::runtime_error("Socket can not accept connections");

    //Return Socketobject for the new connection
    return new Socket(newID, _Type);
}

//Set Socket to listening Socket 
void OwnSocket::Socket::listenPort(int queuesSize)
{
    //Set Socket
    if (listen(_SocketID, queuesSize)<0) throw new std::runtime_error("Socket can not listen");
    _Passive=true;
}

//internal Constructor for accepting connections
OwnSocket::Socket::Socket(int ID, Connection Type)
{
    //temporary variables
    struct sockaddr_in OwnSock;
    socklen_t len;

    //Write Data to Object
    _SocketID=ID;
    getsockname(_SocketID, (sockaddr*)&OwnSock, &len);

    OwnAddress.IP.clear();
    OwnAddress.IP+=inet_ntoa(OwnSock.sin_addr);
    OwnAddress.Port=ntohs(OwnSock.sin_port);
    _Connected=true;
    _Type=Type;
}

//return the connection type of the socket
OwnSocket::Connection OwnSocket::Socket::getConnectionType()
{
    return _Type;
}

//Check the IP-address and Port number returns true if Address is ok
bool OwnSocket::checkAddress(Address* Addr)
{
    //std::cout<<Addr->IP;
    //Iniitalize regex for checking IP-Address
    std::regex regtest("^(?:[0-9]{1,3}[.]){3}[0-9]{1,3}$");
    //Check Addres data
    if(!std::regex_match(Addr->IP.begin(), Addr->IP.end(), regtest) && Addr->IP.empty()) return false;
    if (Addr->Port<=0) return false;

    return true;
}

//Change Float from host to network byte order !!Works only for 32Bit float Type!!
float OwnSocket::ntohf(float In)
{
    //Create temporary Variables
    uint32_t tmp;
    float out;

    //Copy float to int, change Byteorder and copy back
    memcpy(&tmp, &In, sizeof(float));
    tmp=ntohl(tmp);
    memcpy(&out, &tmp, sizeof(float));
    return out;
}

//Change float from network to host byte order !!Works only for 32Bit float Type!!
float OwnSocket::htonf(float In)
{
    //Create temporary Variables
    uint32_t tmp;
    float out;

    //change byte order
    memcpy(&tmp, &In, sizeof(float));
    tmp=ntohl(tmp);
    memcpy(&out, &tmp, sizeof(float));
    return out;
}