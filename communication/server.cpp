#include <iostream>
//#include <WS2tcpip.h>

//#pragma comment (lib, "ws2_32.lib")

using namespace std;

void main(){

    // initialize winsock
    WSADATA wsData;
    WORD ver = MAKEWORD(2,2);

    int ws0k = WSAStartup(ver, &wsData);

    // Create a socket

    //Bind the socket to an IP address and port

    // wait for a connection

    // close listening socket

    // while loop: accapt and echo message back to client

    //close sock

    // shutdown winsock

}