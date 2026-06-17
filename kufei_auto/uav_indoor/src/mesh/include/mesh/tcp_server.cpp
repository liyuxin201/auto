#include "tcp_server.h"
#include <thread>
#include <chrono>

TcpServer::TcpServer(const std::string& ip, int p) 
    : server_fd(-1), client_socket(-1), host_ip(ip), port(p), is_connected(false), running_(false) {
    addrlen = sizeof(address);
}

TcpServer::~TcpServer() {
    stopServer();
}

bool TcpServer::startServer() {
    // 创建socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        ROS_ERROR("Socket creation failed");
        return false;
    }

    // 设置socket选项
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        ROS_ERROR("Setsockopt failed");
        close(server_fd);
        return false;
    }

    // 配置地址
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    
    if (host_ip == "0.0.0.0") {
        address.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, host_ip.c_str(), &address.sin_addr) <= 0) {
            ROS_ERROR("Invalid address");
            close(server_fd);
            return false;
        }
    }

    // 绑定
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        ROS_ERROR("Bind failed on %s:%d", host_ip.c_str(), port);
        close(server_fd);
        return false;
    }

    // 监听
    if (listen(server_fd, 1) < 0) { // 只接受一个连接
        ROS_ERROR("Listen failed");
        close(server_fd);
        return false;
    }

    ROS_INFO("TCP Server started on %s:%d", host_ip.c_str(), port);

    // 设置运行状态
    running_ = true;

    // 启动接受连接的线程
    std::thread accept_thread(&TcpServer::acceptConnections, this);
    accept_thread.detach();

    // 启动接收数据线程
    receive_thread_ = std::thread(&TcpServer::receiveDataLoop, this);
    receive_thread_.detach();

    return true;
}

void TcpServer::acceptConnections() {
    while (server_fd >= 0 && running_) {
        int new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        
        if (new_socket < 0) {
            if (server_fd >= 0 && running_) { // 只有在服务器还在运行时才报错
                ROS_ERROR("Accept failed");
            }
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (client_socket >= 0) {
                close(client_socket); // 关闭旧连接
            }
            client_socket = new_socket;
            is_connected = true;
        }

        // 获取客户端地址
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
        ROS_INFO("Client connected from %s:%d", client_ip, ntohs(address.sin_port));
    }
}

void TcpServer::stopServer() {
    running_ = false;  // 先设置停止标志
    
    {
        std::lock_guard<std::mutex> lock(mtx_);
        is_connected = false;
        
        if (client_socket >= 0) {
            close(client_socket);
            client_socket = -1;
        }
    }
    
    if (server_fd >= 0) {
        close(server_fd);
        server_fd = -1;
    }
    
    ROS_INFO("TCP Server stopped");
}

bool TcpServer::sendBuffer(const std::vector<uint8_t>& buffer) {
    return sendBuffer(buffer.data(), buffer.size());
}

bool TcpServer::sendBuffer(const uint8_t* buffer, size_t length) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    if (client_socket < 0 || !is_connected) {
        ROS_WARN("No client connected");
        return false;
    }
    
    ssize_t bytes_sent = send(client_socket, buffer, length, 0);
    
    if (bytes_sent < 0) {
        ROS_ERROR("Failed to send buffer");
        // 连接可能已断开
        close(client_socket);
        client_socket = -1;
        is_connected = false;
        return false;
    }
    
    if (static_cast<size_t>(bytes_sent) != length) {
        ROS_WARN("Only sent %zd bytes out of %zu", bytes_sent, length);
    }
    
    // ROS_INFO("Sent %zd bytes", bytes_sent);
    return true;
}

bool TcpServer::receiveBuffer(std::vector<uint8_t>& buffer, size_t max_size) {
    buffer.resize(max_size);
    size_t bytes_received;
    
    bool result = receiveBuffer(buffer.data(), max_size, bytes_received);
    
    if (result) {
        buffer.resize(bytes_received); // 调整到实际接收的大小
    } else {
        buffer.clear();
    }
    
    return result;
}

bool TcpServer::receiveBuffer(uint8_t* buffer, size_t buffer_size, size_t& bytes_received) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    if (client_socket < 0 || !is_connected) {
        ROS_WARN("No client connected");
        bytes_received = 0;
        return false;
    }
    
    ssize_t bytes_read = recv(client_socket, buffer, buffer_size, 0);
    
    if (bytes_read < 0) {
        ROS_ERROR("Failed to receive buffer");
        bytes_received = 0;
        return false;
    }
    
    if (bytes_read == 0) {
        ROS_INFO("Client disconnected");
        close(client_socket);
        client_socket = -1;
        is_connected = false;
        bytes_received = 0;
        return false;
    }
    
    bytes_received = static_cast<size_t>(bytes_read);
    ROS_INFO("Received %zu bytes", bytes_received);
    return true;
}

bool TcpServer::isConnected() const {
    return is_connected.load();
}

// 新增：持续接收数据的函数
void TcpServer::receiveDataLoop() {
    const size_t FRAME_SIZE = 61; // 根据主程序中的定义，一帧数据大小为14字节
    std::vector<uint8_t> buffer;
    buffer.reserve(1024); // 预留空间
    
    while (running_) {
        if (isConnected()) {
            uint8_t temp_buffer[1024];
            ssize_t bytes_read = recv(client_socket, temp_buffer, sizeof(temp_buffer), MSG_DONTWAIT);
            
            if (bytes_read > 0) {
                // 将新接收到的数据添加到缓冲区
                for (ssize_t i = 0; i < bytes_read; ++i) {
                    buffer.push_back(temp_buffer[i]);
                }
                
                // 处理缓冲区中的完整帧
                size_t pos = 0;
                while (pos + FRAME_SIZE <= buffer.size()) {
                    // 检查帧头和帧尾是否正确（假设帧头是0xAA，帧尾是0xFF）
                    if (buffer[pos] == 0xAA && buffer[pos + FRAME_SIZE - 1] == 0xFF) {
                        // 找到一个完整且有效的帧
                        std::vector<uint8_t> data(buffer.begin() + pos, buffer.begin() + pos + FRAME_SIZE);
                        {
                            std::lock_guard<std::mutex> lock(receive_queue_mutex_);
                            receive_queue_.push(data);
                        }
                        ROS_INFO("Received valid frame: %zd bytes", data.size());
                        pos += FRAME_SIZE;
                    } else {
                        // 跳过无效字节
                        pos++;
                    }
                }
                
                // 移除已处理的数据
                if (pos > 0) {
                    buffer.erase(buffer.begin(), buffer.begin() + pos);
                }
            } else if (bytes_read == 0) {
                // 连接断开
                ROS_INFO("Client disconnected");
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    close(client_socket);
                    client_socket = -1;
                    is_connected = false;
                }
            } else {
                // 没有数据可读或其他错误，短暂休眠
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } else {
            // 没有连接，短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

// 新增：非阻塞获取接收数据
bool TcpServer::getReceivedData(std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(receive_queue_mutex_);
    
    if (receive_queue_.empty()) {
        return false;  // 队列为空
    }
    
    data = receive_queue_.front();
    receive_queue_.pop();
    return true;
}

// 新增：获取接收队列大小
size_t TcpServer::getReceiveQueueSize() const {
    std::lock_guard<std::mutex> lock(receive_queue_mutex_);  // 现在可以正常工作了
    return receive_queue_.size();
}