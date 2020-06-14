#include <iostream>
#include <WS2tcpip.h>

#pragma comment (lib, "ws2_32.lib")

using namespace std;

void main(){

    // initialize winsock
    WSADATA wsData;
    WORD ver = MAKEWORD(2,2);

    int ws0k = WSAStartup(ver, &wsData);
    if (ws0k !=0 )
    {
        cerr << "Can't Initialize winsock! Quiting" << endl;
        return;
    }

    // Create a socket
    SOCKET listening = socket(AF_INET, SOCK_STREAM, 0);
    if (listening == INVALID_SOCKET)
    {
        cerr << "Can't Initialize winsock! Quiting" << endl;
        return;
    }

    //Bind the socket to an IP address and port
    sockaddr_in central;
    central.sin_family = AF_INET;
    central.sin_port =htons(54000)
    central.sin_addr.s_un.s_addr = INADDR_ANY; // Could also use inet_pton...

    bind(listening, (sockaddr*)&central, sizeof(central));

    //tell winsock the socket is listening for
    listen(listening, SOMAXCONN);

    // wait for a connection
    sockaddr_in robot;
    int robotSize = sizeof(robot);

    SOCKET robotSocket = accept(listening, (sockaddr*)&robot, &robotSize);
    if (robotSocket == INVALID_SOCKET)
    {
        cerr << "Can't Initialize winsock! Quiting" << endl;
        return;
    }

    char host[NI_MAXHOST]; // clients remote name
    char service[NI_MAXHOST]; // service (i.e. port) client is connected on

    ZeroMemory(host, NI_MAXHOST); // same as memset(host, 0, NI_MAXHOST);
    ZeroMemory(service, NI_MAXHOST);

    if (getnameinfo((sockaddr*)&robot, sizeof(robot), host, NI_MAXHOST, service, NI_MAXHOST, 0) == 0)
    {
        cout << host << "connected on port"<< service << endl;
    }
    else
    {
        inet_ntop(AF_INET, &robot.sin_addr, host, NI_MAXHOST);
        cout << host << "connected on port" << ntohs(robot.sin_port) << endl;
    }

    // close listening socket
    closesocket(listening);

    // while loop: accapt and echo message back to client
    char buffer[4096];

    while(true)
    {
        ZeroMemory(buffer, 4096);
        // wait for client to send data
        receive
        //Echo Message back to client
    }

    //close sock

    // shutdown winsock

}