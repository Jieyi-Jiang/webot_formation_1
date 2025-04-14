#include <iostream>
#include <winsock2.h>

// #pragma comment(lib, "Ws2_32.lib")

#define BUFFER_SIZE 1024

int main() {
    WSADATA wsaData;
    SOCKET listeningSocket, newConnection;
    sockaddr_in serverAddr, clientAddr;
    int clientAddrSize = sizeof(clientAddr);
    char buffer[BUFFER_SIZE];
    int bytesReceived;

    // ��ʼ��Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed: " << WSAGetLastError() << std::endl;
        return 1;
    }

    // ��������Socket
    listeningSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (listeningSocket == INVALID_SOCKET) {
        std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // ���÷�������ַ�ṹ
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(8888); // ����8888�˿�

    // ��Socket
    if (bind(listeningSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
        closesocket(listeningSocket);
        WSACleanup();
        return 1;
    }

    // ��ʼ����
    if (listen(listeningSocket, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Listen failed: " << WSAGetLastError() << std::endl;
        closesocket(listeningSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Waiting for a connection..." << std::endl;

    // ���ܿͻ�������
    newConnection = accept(listeningSocket, (sockaddr*)&clientAddr, &clientAddrSize);
    if (newConnection == INVALID_SOCKET) {
        std::cerr << "Accept failed: " << WSAGetLastError() << std::endl;
        closesocket(listeningSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Client connected!" << std::endl;

    // ��������
    bytesReceived = recv(newConnection, buffer, BUFFER_SIZE, 0);
    if (bytesReceived == SOCKET_ERROR) {
        std::cerr << "Receive failed: " << WSAGetLastError() << std::endl;
    } else {
        buffer[bytesReceived] = '\0'; // ȷ���ַ�����null��β
        std::cout << "Received: " << buffer << std::endl;
    }

    // ��������
    const char* message = "Hello from server!";
    if (send(newConnection, message, strlen(message), 0) == SOCKET_ERROR) {
        std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
    }

    // �ر�Socket
    closesocket(newConnection);
    closesocket(listeningSocket);

    // ����Winsock
    WSACleanup();

    return 0;
}
