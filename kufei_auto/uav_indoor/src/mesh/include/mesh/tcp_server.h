#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <mutex>
#include <atomic>
#include <queue>
#include <thread>

class TcpServer {
private:
    int server_fd;
    int client_socket;
    struct sockaddr_in address;
    int addrlen;
    std::mutex mtx_; // 保护socket访问的互斥锁
    std::atomic<bool> is_connected; // 连接状态
    std::string host_ip;
    int port;
    
    // 新增接收缓冲区相关成员
    std::queue<std::vector<uint8_t>> receive_queue_;  // 接收数据队列
    mutable std::mutex receive_queue_mutex_;          // 保护接收队列的互斥锁 (注意这里加了mutable)
    std::thread receive_thread_;                      // 接收数据线程
    std::atomic<bool> running_;                       // 服务器运行状态

    void acceptConnections();
    void receiveDataLoop();                           // 新增：持续接收数据的函数

public:
    TcpServer(const std::string& ip = "0.0.0.0", int p = 10000);
    ~TcpServer();

    bool startServer();
    void stopServer();
    
    // 发送buffer函数
    bool sendBuffer(const std::vector<uint8_t>& buffer);
    bool sendBuffer(const uint8_t* buffer, size_t length);
    
    // 接收buffer函数
    bool receiveBuffer(std::vector<uint8_t>& buffer, size_t max_size = 1024);
    bool receiveBuffer(uint8_t* buffer, size_t buffer_size, size_t& bytes_received);
    
    // 新增：获取队列中的数据
    bool getReceivedData(std::vector<uint8_t>& data); // 非阻塞获取接收数据
    size_t getReceiveQueueSize() const;               // 获取接收队列大小
    
    // 获取连接状态
    bool isConnected() const;
    
    // 清理接收队列
    void clearReceiveQueue();
};

#endif // TCP_SERVER_H