#include <iostream>
#include <string>
#include <thread>
#include <cstring>
#include <thread>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <Synchapi.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")

#include <common/mavlink.h>

#include "mavlink_udp.hpp"
using namespace std;

MavlinkUDP::MavlinkUDP(const string multicast_ip, const string multicast_port, const string bind_port)
{
    // 启动 WSA 库
    int result = WSAStartup(MAKEWORD(2, 2), &(this->_wsa_data));
    if (result != NO_ERROR) {
        fprintf(stderr, "WSAStartup failed: %d\n", result);
        throw EXIT_FAILURE;
    }

    //  创建 socket 套接字，相当于文件描述符
    this->_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->_sock == INVALID_SOCKET)
    {
        fprintf(stderr, "socket function failed with error: %d\n", WSAGetLastError());
        throw EXIT_FAILURE;
    }

    // 设置 多播TTL时间
    DWORD ttl_value = 32;
    if (SOCKET_ERROR == setsockopt(this->_sock, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl_value, sizeof(ttl_value)))
    {
        fprintf(stderr, "setsockopt IPPROTO_IP IP_MULTICAST_TTL error: %d\n", WSAGetLastError());
        throw EXIT_FAILURE;
    }
    
    // 开启允许地址重用 - 有风险 
    // 关闭了之后，不同程序不能绑定同一端口
    boolean reuse = true;
    if (SOCKET_ERROR == setsockopt(this->_sock, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse)))
    {
        fprintf(stderr, "setsockopt SOL_SOCKET SO_REUSEADDR error: %d\n", WSAGetLastError());
        throw EXIT_FAILURE;
    }

    // 设置绑定的地址
    memset(&this->_my_addr, 0, sizeof(this->_my_addr));
    this->_my_addr.sin_family = AF_INET; // PF_INET = AF_INET
    // this->_my_addr.sin_addr.S_un.S_addr = inet_addr("192.168.31.85");
    this->_my_addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    this->_my_addr.sin_port = htons(atol(bind_port.c_str()));
    // this->_my_addr.sin_port = htons(8080);
    // printf("my addr: %s\n", inet_ntoa(this->_my_addr.sin_addr));
    
    // 也可以不 bind
    if (SOCKET_ERROR == bind(this->_sock, (sockaddr*)(&this->_my_addr), sizeof(this->_my_addr)))
    {
        fprintf(stderr, "bind error: %d\n", WSAGetLastError());
        throw EXIT_FAILURE;
    }

    // 关闭多播回环(multicast loopback)，不接收自己发的 datagram 
    // boolean loopback = false;
    // if (SOCKET_ERROR == setsockopt(this->_sock, IPPROTO_IP, IP_MULTICAST_LOOP, (char*)&loopback, sizeof(loopback)))
    // {
    //     fprintf(stderr, "setsockopt IPPROTO_IP IP_MULTICAST_LOOP error: %d\n", WSAGetLastError());
    //     throw EXIT_FAILURE;
    // }

    // 加入组播
    this->_mreq.imr_multiaddr.S_un.S_addr = inet_addr(multicast_ip.c_str());
    // this->_mreq.imr_multiaddr.S_un.S_addr = inet_addr("239.255.255.250");
    printf("multicast_ip:%s\n", multicast_ip.c_str());
    // this->_mreq.imr_interface.S_un.S_addr = inet_addr("192.168.31.85");
    this->_mreq.imr_interface.S_un.S_addr = htonl(INADDR_ANY);
    result = setsockopt(this->_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)(&this->_mreq), sizeof(this->_mreq));
    if(SOCKET_ERROR == result)
    {
        fprintf(stderr, "setsockapt IPPROTO_IP IP_ADD_MEMBERSHIP error: %d\n", result);
        throw EXIT_FAILURE;
    }

    memset(&this->_multicast_addr, 0, sizeof(this->_multicast_addr));
    this->_multicast_addr.sin_family = AF_INET;
    this->_multicast_addr.sin_addr.S_un.S_addr = inet_addr(multicast_ip.c_str());
    this->_multicast_addr.sin_port = htons(atol(multicast_port.c_str()));
    // this->_multicast_addr.sin_port = htons(8080);
    printf("_multicast_addr port: %d\n", atol(multicast_port.c_str()));
    printf("_multicast_addr addr: %s\n", inet_ntoa(this->_multicast_addr.sin_addr));
    this->_recive_buff = new char[this->_recive_buff_size];
    if (this->_recive_buff == nullptr)
    {
        fprintf(stderr, "new _recive_buff error");
        throw EXIT_FAILURE;
    }
}

MavlinkUDP::~MavlinkUDP()
{
    if (this->_recive_buff != nullptr)
    {
        delete[] this->_recive_buff;
    }
    closesocket(this->_sock);
    WSACleanup();
    printf("clean\n");
}

int MavlinkUDP::mavudp_send_string(const string str)
{
    const char *send_str = "test multicast";
    cout << "send: " << str << endl;
    int str_len = strlen(send_str);
    printf("ste length: %d\n", str_len);
    if (SOCKET_ERROR == sendto(this->_sock, send_str, str_len, 0, (sockaddr*)(&this->_multicast_addr), sizeof(this->_multicast_addr)))
    {
        cerr << "fail to send" << endl;
    }
    return 0;
    // cout << "send: " << str << endl;
    // printf("ste length: %d\n", str.length());
    // if (SOCKET_ERROR == sendto(this->_sock, str.c_str(), str.length(), 0, (sockaddr*)(&this->_multicast_addr), sizeof(this->_multicast_addr)))
    // {
    //     cerr << "fail to send" << endl;
    // }
    // return 0;
}


int MavlinkUDP::mavudp_receive_string(string &str)
{
    int ret = 0; 
    int addr_size = sizeof(this->_multicast_addr);
    // ret = recvfrom(this->_sock, this->_recive_buff, this->_recive_buff_size, 
    //     0, (sockaddr*)(&this->_multicast_addr), &addr_size);
    ret = recvfrom(this->_sock, this->_recive_buff, this->_recive_buff_size, 0, NULL, NULL);
    if (ret == SOCKET_ERROR) 
    {
        cerr << "fail to recive" << endl;
        return 0;
    }
    if (ret > 0)
    {
        printf("str size: %d\n", ret);
        this->_recive_buff[ret] = '\0';
        str = string(this->_recive_buff);
        cout << "receive: " << str << endl;
        ret = 0;
    }
    return ret;
}


int MavlinkUDP::mavudp_send_bytes(const char *buff, const size_t send_size)
{
    long long ret_val =  sendto(this->_sock, buff, send_size, 0, (sockaddr*)(&this->_multicast_addr), sizeof(this->_multicast_addr));
    if (ret_val < 0)
    {
        cerr << "fail to send" << endl;
    }
    if (ret_val == 0)
    {
        cerr << "send nothing" << endl;
    }
    return ret_val;
}

int MavlinkUDP::mavudp_receive_bytes(char *buff, const size_t buff_size)
{
    // 在非抢占式的windows老古董才需要这个东西，win95之后就没什么用
    // if (WSAIsBlocking()) 
    // {
    //     cerr << "block" << endl;
    //     return -1;
    // }
    int addr_size = sizeof(this->_multicast_addr);
    long long ret_val = 0;
    ret_val = recvfrom(this->_sock, buff, buff_size, 0, NULL, NULL);
    // ret_val = recvfrom(this->_sock, buff, buff_size, 0, (sockaddr*)(&this->_multicast_addr), &addr_size);
    if (ret_val < 0)
    {
        fprintf(stderr, "SOCKET_ERROR: %u\n | from mavudp_receive_bytes", WSAGetLastError());
    }
    if (ret_val == 0)
    {
        cerr << "disconnected" << endl;
        
    }
    return ret_val;
}


int MavlinkUDP::mavudp_recive_message(uint8_t channel ,mavlink_message_t *message, mavlink_status_t *status)
{
    char buff[MAVLINK_MAX_PACKET_LEN];
    int buff_size = MAVLINK_MAX_PACKET_LEN;
    // 接收数据
    int ret = recvfrom(this->_sock, buff, buff_size, 0, NULL, NULL);
    if (ret < 0)
    {
        cerr << "receive error: " << WSAGetLastError() << endl;
        return -1;
    }
    if (ret == 0)
    {
        cerr << "disconnected" << endl;
        return -1;
    }
    // printf("receive %d bytes\n", ret);
    // 反序列化
    int mav_ret = 0;
    for (int i = 0; i < ret; ++ i)
    {
        mav_ret = mavlink_parse_char(channel, buff[i], message, status);
        // cout << mav_ret;
    }
    // cout << endl;
    if (mav_ret == 0)
    {
        cerr << "message could be decoded or bad CRC" << endl;
        return -1;
    }
    return 0;
}

int MavlinkUDP::mavudp_send_message(mavlink_message_t *message)
{
    char buff[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer((uint8_t*)buff, message);
    int ret = this->mavudp_send_bytes(buff, len);
    if (ret != len) 
    {
        cerr << "send error: " << WSAGetLastError() << endl;
        return -1;
    }
    return 0;
}