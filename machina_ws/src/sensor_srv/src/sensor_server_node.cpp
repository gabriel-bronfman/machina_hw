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
  : LifecycleNode("sensor_server_node"), sock_fd(-1), sensor_thread(nullptr)
  {
        this->declare_parameter<std::string>("address", "127.0.0.1");
        this->get_parameter("address", this->address);

        this->declare_parameter<int>("port", 10000);
        this->get_parameter("port", this->port);

        this->declare_parameter<std::string>("sample_num", "5");
        this->get_parameter("sample_num", this->numOfSamples);

        this->sensor_vals.resize(5 * 6);
  }

  ~sensor_lifecyle_node() 
  {
    
    if (sock_fd != -1) {
      close(sock_fd);
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(get_logger(), "Configuring sensor to address %s and port %d", this->address.c_str(), this->port);
    
    if (connect_to_sensor(this->address.c_str(), this->port)) { 
      RCLCPP_INFO(get_logger(), "Successfully connected to the sensor.");
      
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to connect to the sensor.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
  {
      RCLCPP_INFO(get_logger(), "Deactivating...");

      
      if(service_ != nullptr) {
          
          service_.reset();
          RCLCPP_INFO(get_logger(), "Service deleted successfully.");
      } else {
          RCLCPP_WARN(get_logger(), "Service was already null.");
      }
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(get_logger(), "Shutting Down...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
      if (this->sensor_thread == nullptr)
      {
        this->sensor_thread = create_receiving_thread();
      }
    } 
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create thread: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Activation Successful");
    return CallbackReturn::SUCCESS;
  }

  void node_service_function(const std::shared_ptr<robot_interfaces::srv::Sensor::Request> request, std::shared_ptr<robot_interfaces::srv::Sensor::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Incoming request");

    
    std::lock_guard<std::mutex> guard(sensor_vals_mutex);

    response->data.layout.dim.resize(2); 


    response->data.layout.dim[0].label = "rows";
    response->data.layout.dim[0].size = 5;
    response->data.layout.dim[0].stride = 5 * 6;

    // Dimension for columns
    response->data.layout.dim[1].label = "cols";
    response->data.layout.dim[1].size = 6; 
    response->data.layout.dim[1].stride = 6;


    for (size_t i = 0; i < this->sensor_vals.size(); ++i) {
      response->data.data.push_back(this->sensor_vals[i]);
    }
  }
  


private:

  int sock_fd;
  int port;
  std::string address;
  std::string numOfSamples;

  std::atomic<bool> stopSignal{false};
  std::vector<double> sensor_vals;
  std::unique_ptr<std::thread> sensor_thread;
  rclcpp::Service<robot_interfaces::srv::Sensor>::SharedPtr service_;
  std::mutex sensor_vals_mutex;

  bool connect_to_sensor(const char* address, int port) 
  {
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
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

        int bytesReceived = read(sock_fd, buffer, sizeof(buffer));
        if (bytesReceived > 0) {
          int numDoubles = bytesReceived / sizeof(double); 

          
          std::lock_guard<std::mutex> guard(sensor_vals_mutex);

         
          this->sensor_vals.clear();

          
          for(int i = 0; i < numDoubles; ++i) {
            double value;
            std::memcpy(&value, &buffer[i * sizeof(double)], sizeof(double));
            this->sensor_vals.push_back(value);
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
