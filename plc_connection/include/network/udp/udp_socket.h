#ifndef UDP_SOCKET_H
#define UDP_SOCkET_H

#include "network/udp/udp.h"

namespace OwnUDP
{
    class UDPSocket:private OwnSocket::Socket
    {
        public:
        //Constructors and Destructors
        UDPSocket();
        UDPSocket(Address* OwnIP);
        ~UDPSocket();
        
        //Set/Get Address
        void bindAddress(Address* OwnIP);
        void getAddress(Address* IP);

        //Set Timeout
        void setReceiveTime(int usec, int sec);

        //Write and read data
        void write(uint8_t* Data, int length, Address* IP);
        void read(uint8_t* Data, int length, Address* IP);

        private:

    };

}

#endif