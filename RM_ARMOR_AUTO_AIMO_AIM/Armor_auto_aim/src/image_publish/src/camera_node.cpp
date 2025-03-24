//std
#include<functional>
#include<chrono>
#include<iostream>

//rclcpp
#include<rclcpp/rclcpp.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/msg/image.h>

//opencv
#include<opencv2/opencv.hpp>

using namespace std::chrono_literals;

//发布图像
class Camera_node_publisher:public rclcpp::Node
{
   public:
      Camera_node_publisher():Node("camera")
      {
         //发布节点创建
         RCLCPP_INFO(this->get_logger(),"camera publish...");
         pub_=this->create_publisher<sensor_msgs::msg::Image>("camera/image",10);
         
         cv::Mat image;
         std::string path = "/home/x/visionlib/vision.mp4";
         cap_ = cv::VideoCapture(path);
         
         //发布图像
         while(rclcpp::ok())
         {
           cap_>>image;
           std_msgs::msg::Header header;
           header.stamp = this->now();
           header.frame_id = "camera";
           auto src = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
           pub_->publish(*src);
           rclcpp::sleep_for(7ms);
         }
      }
    private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera_node_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}