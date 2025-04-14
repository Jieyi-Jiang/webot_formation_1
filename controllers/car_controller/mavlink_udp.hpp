#ifndef MAVLINK_UDP_HPP
#define MAVLINK_UDP_HPP

#include <iostream>
#include <string>
#include <cstdio>

#include <common/mavlink.h>
#include <windows.h>

using namespace std;

class MavlinkUDP
{

public:
	MavlinkUDP(const string multicast_ip, const string multicast_port, const string bind_port);
	~MavlinkUDP();
	int mavudp_recive_message(uint8_t channel ,mavlink_message_t *message, mavlink_status_t *status);
	int mavudp_send_message(mavlink_message_t *message);
	int mavudp_send_string(const string str);
	int mavudp_receive_string(string &str);
	int mavudp_send_bytes(const char *buff, const size_t send_size);
	int mavudp_receive_bytes(char *buff, const size_t buff_size);
	int mavudp_send_heartbeat();
protected:
	SOCKET _sock;
	WSADATA _wsa_data;
	SOCKADDR_IN _my_addr; 
	SOCKADDR_IN _multicast_addr;
	SOCKADDR_IN _reciveform_addr;
	int _reciveform_addr_len;
	ip_mreq _mreq;
	char *_recive_buff;
	int _recive_buff_size = 2048;
};

#endif // MAVLINK_UDP_HPP