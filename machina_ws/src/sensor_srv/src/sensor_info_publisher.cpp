#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "robot_interfaces/srv/sensor.hpp" 
#include <chrono>
#include <queue>

class SensorInfoPublisher : public rclcpp::Node
{
public:
  SensorInfoPublisher()
  : Node("sensor_info_publisher"), sensor_data_(std::make_shared<std_msgs::msg::Float32MultiArray>())
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sensor_info_topic", 10);
    
    client_ = this->create_client<robot_interfaces::srv::Sensor>("/sensor_info_service");
    

    queue_of_readings = std::make_unique<std::queue<std::array<float, 6>>>();

    request_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SensorInfoPublisher::call_service, this));
    
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&SensorInfoPublisher::publish_data, this));
  }

private:

  rclcpp::TimerBase::SharedPtr request_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Client<robot_interfaces::srv::Sensor>::SharedPtr client_;
  std::shared_ptr<std_msgs::msg::Float32MultiArray> sensor_data_;

  std::unique_ptr<std::queue<std::array<float,6>>> queue_of_readings;
  std::mutex queue_mutex;
  
  // std::chrono::time_point<std::chrono::high_resolution_clock> start;
  // std::chrono::time_point<std::chrono::high_resolution_clock> end;

  void call_service()
  {
    if (!this->client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available.");
      return;
    }
    
    auto request = std::make_shared<robot_interfaces::srv::Sensor::Request>();
    // this->start = std::chrono::high_resolution_clock::now();
    auto result_future = this->client_->async_send_request(request,
      std::bind(&SensorInfoPublisher::handle_service_response, this, std::placeholders::_1));
  }

  void handle_service_response(const rclcpp::Client<robot_interfaces::srv::Sensor>::SharedFuture future)
  {
    auto response = future.get();
    
    
    std::lock_guard<std::mutex> guard(queue_mutex);

    for (int col = 0; col < 5; ++col){ 
        std::array<float, 6> columnReadings; 
        for (int row = 0; row < 6; ++row) 
        {
            int index = row * 5 + col;
            columnReadings[row] = response->data.data[index];
        }
        queue_of_readings->push(columnReadings);
    }
  }

  void publish_data()
  {
    std::lock_guard<std::mutex> guard(queue_mutex); 

    
    if (!queue_of_readings->empty()) {
      auto readings = queue_of_readings->front();
      queue_of_readings->pop();

      
      sensor_data_->data.clear();
      for (const auto& value : readings) {
        sensor_data_->data.push_back(value);
      }

      publisher_->publish(*sensor_data_);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
