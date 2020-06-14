#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#define PORT 8080

bool condition;

int main(int argc, char const *argv[]) {

	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = {0};

    server_fd = socket(AF_INET, SOCK_STREAM,0);

    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);
    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    while(condition=true){
        valread = read( new_socket , buffer, 1024); 
        if(buffer < 0){
            condition = false;
        }
        else{
            printf("%s\n",buffer);
            bzero(buffer,256);
            send(new_socket , "This is a test" , strlen("This is a test") , 0 ); 
        }
        
    }
        
} 