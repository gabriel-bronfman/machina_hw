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
#include <vector>

using namespace std::chrono_literals;
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

class sensor_lifecyle_node : public LifecycleNode {
public:
  sensor_lifecyle_node()
  : LifecycleNode("sensor_server_node") {
    this->declare_parameter<int>("num_sensors", 1);
    this->get_parameter("num_sensors", this->num_sensors);

    this->declare_parameter<std::vector<std::string>>("sensor_addresses", std::vector<std::string>{"127.0.0.1"});
    this->get_parameter("sensor_addresses", this->sensor_addresses);

    this->declare_parameter<int>("port", 10000);
    this->get_parameter("port", this->port);

    this->declare_parameter<std::string>("sample_num", "5");
    this->get_parameter("sample_num", this->numOfSamples);

    // Resize to accommodate values for all sensors
    this->sensor_vals.resize(num_sensors, std::vector<double>(5 * 6));
    this->sock_fds.resize(num_sensors, -1); 

    this->sensor_threads.resize(num_sensors);
  }

  ~sensor_lifecyle_node() {
    for (auto& socket : sock_fds) {
      if (socket != -1) {
        close(socket);
      }
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    for (int i = 0; i < num_sensors; ++i) {
      RCLCPP_INFO(get_logger(), "Configuring sensor %d to address %s and port %d", i, this->sensor_addresses[i].c_str(), this->port);
      
      if (connect_to_sensor(i, this->sensor_addresses[i].c_str(), this->port)) { 
        RCLCPP_INFO(get_logger(), "Successfully connected to sensor %d.", i);
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to connect to sensor %d.", i);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "Activating...");

    for (int i = 0; i < num_sensors; ++i) {
      std::string service_name = "sensor_info_" + std::to_string(i);
      auto service = this->create_service<robot_interfaces::srv::Sensor>(
          service_name, 
          [this, i](const std::shared_ptr<robot_interfaces::srv::Sensor::Request> request,
                    std::shared_ptr<robot_interfaces::srv::Sensor::Response> response) {
              this->node_service_function(i, request, response);
          });
      services_.push_back(service);
    }

    RCLCPP_INFO(get_logger(), "Finished starting services");
    for (int i = 0; i < num_sensors; ++i) {
      try {
        if (sensor_threads[i] == nullptr) {
          sensor_threads[i] = create_receiving_thread(i);
        }
      } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(get_logger(), "Failed to create thread for sensor %d: %s", i, e.what());
        return CallbackReturn::FAILURE;
      }
    }

    RCLCPP_INFO(get_logger(), "Activation Successful");
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    for (auto& service : services_) {
      if(service != nullptr) {
        service.reset();
      }
    }
    RCLCPP_INFO(get_logger(), "Services deactivated successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "Shutting Down...");
    stop_all_threads();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  int num_sensors;
  std::vector<std::string> sensor_addresses;
  int port;
  std::string numOfSamples;
  std::vector<int> sock_fds;
  std::atomic<bool> stopSignal{false};
  std::vector<std::vector<double>> sensor_vals;
  std::vector<std::unique_ptr<std::thread>> sensor_threads{num_sensors};
  std::vector<rclcpp::Service<robot_interfaces::srv::Sensor>::SharedPtr> services_;
  std::mutex sensor_vals_mutex;

  bool connect_to_sensor(int index, const char* address, int port) {
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    sock_fds[index] = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fds[index] < 0) {
      RCLCPP_ERROR(get_logger(), "Socket creation error for sensor %d", index);
      return false;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if(inet_pton(AF_INET, address, &serv_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid address for sensor %d", index);
      return false;
    }

    if (connect(sock_fds[index], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Connection Failed for sensor %d", index);
      return false;
    }
    return true;
  }

  std::unique_ptr<std::thread> create_receiving_thread(int index) {
    return std::make_unique<std::thread>([this, index]() {
      char buffer[1024] = {0};
      while (!this->stopSignal) {
        write(sock_fds[index], this->numOfSamples.c_str(), this->numOfSamples.size());

        int bytesReceived = read(sock_fds[index], buffer, sizeof(buffer));
        if (bytesReceived > 0) {
          int numDoubles = bytesReceived / sizeof(double);
          std::lock_guard<std::mutex> guard(sensor_vals_mutex);
          sensor_vals[index].clear();
          for(int i = 0; i < numDoubles; ++i) {
            double value;
            std::memcpy(&value, &buffer[i * sizeof(double)], sizeof(double));
            sensor_vals[index].push_back(value);
          }
        }
      }
    });
  }

  void stop_all_threads() {
    stopSignal.store(true);
    for (auto& thread : sensor_threads) {
      if (thread && thread->joinable()) {
        thread->join();
      }
    }
  }

  void node_service_function(int index, const std::shared_ptr<robot_interfaces::srv::Sensor::Request> request, std::shared_ptr<robot_interfaces::srv::Sensor::Response> response) {
    RCLCPP_INFO(get_logger(), "Incoming request for sensor %d", index);
    std::lock_guard<std::mutex> guard(sensor_vals_mutex);

    response->data.layout.dim.resize(2);
    response->data.layout.dim[0].label = "rows";
    response->data.layout.dim[0].size = 5;
    response->data.layout.dim[0].stride = 5 * 6;

    response->data.layout.dim[1].label = "cols";
    response->data.layout.dim[1].size = 6;
    response->data.layout.dim[1].stride = 6;

    for (size_t i = 0; i < sensor_vals[index].size(); ++i) {
      response->data.data.push_back(sensor_vals[index][i]);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto lc_node = std::make_shared<sensor_lifecyle_node>();
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
