#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <regex>

namespace OwnSocket
{

    //change order of bytes for float
    float ntohf(float In);
    float htonf( float In);

    //enum for the connection type
    enum Connection
    {
        TCP,
        UDP
    };

    //struct for IP-Address
    struct Address
    {
        std::string IP;
        int Port;
    };

    //Function to check if the Address is valid
    bool checkAddress(Address* Addr);

    //Socket class
    class Socket
    {
        public:
        //Constructor/Destructor
        Socket();
        Socket(Connection Type);
        ~Socket();

        //open/close/initialize connection
        void connectTo(Address* IP);
        void closeSocket();
        void init(Connection Type);
        
        //read/write data on socket
        void write(uint8_t* Data, int length, Address* Target);
        void read(uint8_t* Data, int length, Address* Source);
        
        //check Socket
        bool socketOk();
        bool connected();

        //misc Functions for Socket
        void bindAddress(Address* IP);
        Socket* acceptConnection();        
        void listenPort(int queuesSize);
        void getAddress(Address* Target);
        void setReceiveTime(int usec, int sec);
        Connection getConnectionType();

        private:
        //internal Constructor for programming servers 
        Socket(int ID, Connection Type);
        
        protected:
        //Class member
        Connection Type_;
        int SocketID_;
        bool Connected_;
        bool Passive_;
        Address OwnAddress_;

    };


}

#endif