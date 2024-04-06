#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <lifecycle_msgs/srv/change_state.hpp>
#include "robot_interfaces/srv/sensor.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <array>
#include <sstream>
#include <iostream>
#include <atomic>
#include <memory>
#include <thread>
#include <functional>

using namespace std::chrono_literals;
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

class sensor_lifecyle_node : public LifecycleNode
{
public:
  sensor_lifecyle_node()
  : LifecycleNode("sensor_server_node"), sock_fd(-1), sensor_vals(nullptr)
  {
        this->declare_parameter<std::string>("address", "127.0.0.3");
        this->get_parameter("Address", this->address);

        this->declare_parameter<int>("port", 10000);
        this->get_parameter("Port", this->port);

        this->declare_parameter<std::string>("sample_num", "10");
        this->get_parameter("sample_num", this->numOfSamples);

        
  }

  ~sensor_lifecyle_node() 
  {
    
    if (sock_fd != -1) {
      close(sock_fd);
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(get_logger(), "Configuring:");
    
    if (connect_to_sensor(this->address.c_str(), this->port)) { 
      RCLCPP_INFO(get_logger(), "Successfully connected to the sensor.");
      this->sensor_vals = std::make_unique<float[]>(6);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to connect to the sensor.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(get_logger(), "Activating...");

    service_ = this->create_service<robot_interfaces::srv::Sensor>(
    "sensor_info", 
    std::bind(
        &sensor_lifecyle_node::node_service_function, 
        this, 
        std::placeholders::_1, 
        std::placeholders::_2
      )
    );
    try 
    {
      this->sensor_thread = create_receiving_thread();
    } 
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create thread: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  void node_service_function(const std::shared_ptr<robot_interfaces::srv::Sensor::Request> _, std::shared_ptr<robot_interfaces::srv::Sensor::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
    for(int i = 0; i < size(response->data); i++){
      response->data[i] = this->sensor_vals[i];
    }

  }
  


private:

  int sock_fd;
  int port;
  std::string address;
  std::string numOfSamples;

  std::atomic<bool> stopSignal{false};
  std::unique_ptr<float[]> sensor_vals;
  std::unique_ptr<std::thread> sensor_thread;
  rclcpp::Service<robot_interfaces::srv::Sensor>::SharedPtr service_;

  bool connect_to_sensor(const char* address, int port) 
  {
    struct sockaddr_in serv_addr;
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Socket creation error");
      return false;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if(inet_pton(AF_INET, address, &serv_addr.sin_addr)<=0) {
      RCLCPP_ERROR(get_logger(), "Invalid address");
      return false;
    }

    if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Connection Failed");
      return false;
    }
    return true;
  }

  std::unique_ptr<std::thread> create_receiving_thread() 
  {
    auto task = [this]() {

      char buffer[1024] = {0};
      this->stopSignal.store(false);
      while (!this->stopSignal) {

        write(this->sock_fd, this->numOfSamples.c_str(), this->numOfSamples.size());

        int bytesReceived = read(sock_fd, buffer, 1024);

        if (bytesReceived > 0) {

          buffer[bytesReceived] = '\0';
          std::string receivedData(buffer, bytesReceived);

          receivedData.erase(0, 1); 
          receivedData.pop_back(); 

          std::stringstream ss(receivedData);
          std::vector<int> tempData;
          std::string item;
          int count = 0;

          while(std::getline(ss, item, ',')){
            this->sensor_vals[count] = std::stof(item);
          }
        }
      }
    };
    return std::make_unique<std::thread>(task);
  }

  void stopThread(std::unique_ptr<std::thread> thread)
  {
    this->stopSignal.store(true);
    if (thread && thread->joinable()){
      thread->join();
    }
    
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<sensor_lifecyle_node> lc_node = std::make_shared<sensor_lifecyle_node>();

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  
  return 0;
}
