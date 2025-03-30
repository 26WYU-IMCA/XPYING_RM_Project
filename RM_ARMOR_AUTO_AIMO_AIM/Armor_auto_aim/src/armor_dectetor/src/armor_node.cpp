// 装甲板识别节点   接收图像 识别装甲板  pnp解算 x为前 y为左 z为上

#include "Armor_match.h"
#include "PnP_Solver.h"
#include "armor_parameter/msg/armor.hpp"

// std
#include <iostream>
#include <functional>
#include <algorithm>

// opencv
#include <opencv2/opencv.hpp>

// rclcpp
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

// 装甲板识别节点
class Armor_Node : public rclcpp::Node
{
public:
    Armor_Node() : Node("armor_dectetor")
    {
        RCLCPP_INFO(this->get_logger(), "Armor Node working...");

        // 创建接受节点
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image", 10,
                                                                  std::bind(&Armor_Node::ArmorMark, this, std::placeholders::_1));

        // 创建图像发送节点
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_track", 10);

        //创建坐标发送节点
        pub_x_y_z_ = this->create_publisher<armor_parameter::msg::Armor>("camera/xyz",10);

        // 类初始化
        pnp_solver_ = std::make_shared<PnPSolver>();
        armorDetector_ = std::make_shared<armorDetector>(10, 15, 2.5);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // 接受未处理图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;    // 发送处理完的图像
    rclcpp::Publisher<armor_parameter::msg::Armor>::SharedPtr pub_x_y_z_;  //发送坐标
    
    // 图像处理
    void ArmorMark(const sensor_msgs::msg::Image::ConstSharedPtr &image);           // 接受图像并显示装甲板
    void DrawArmor(const cv::Mat &image, armorDetector::Armor &target_Armor);       // 装甲板标记
    void ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold); // 图像处理
    void SendImage(const cv::Mat &src);                                             // 图像发送
    void SendXYZ(const armorDetector::Armor &target_armor);                      //坐标发送

    // pnp解算类
    std::shared_ptr<PnPSolver> pnp_solver_;

    // 装甲板类
    std::shared_ptr<armorDetector> armorDetector_;
};

// 图像处理
void Armor_Node::ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold)
{
    cv::Mat thr, gray, dil, temp;
    cv::Mat ken = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    double threshold1 = 3;
    double threshold2 = 9;
    cv::undistort(image, temp, armorDetector_->camera_matrix, armorDetector_->distortion_coefficients);
    cv::cvtColor(temp, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thr, threshold, 255, cv::THRESH_BINARY);
    cv::morphologyEx(thr, dil, cv::MORPH_OPEN, ken, cv::Point(-1, -1));
    cv::Canny(dil, edges, threshold1, threshold2);
}

// 装甲板标记
void Armor_Node::DrawArmor(const cv::Mat &image, armorDetector::Armor &target_Armor)
{
    cv::line(image, target_Armor.left_light.top, target_Armor.right_light.bottom, cv::Scalar(0, 255, 0), 3);
    cv::line(image, target_Armor.left_light.bottom, target_Armor.right_light.top, cv::Scalar(0, 255, 0), 3);
}

// 装甲板排序
bool compare(armorDetector::Light left_light, armorDetector::Light right_light)
{
    return right_light.light.center.x > left_light.light.center.x;
}

// 图像发送
void Armor_Node::SendImage(const cv::Mat &image)
{
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";
    auto src = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pub_->publish(*src);
}

//坐标发送
void Armor_Node::SendXYZ(const armorDetector::Armor &target_armor)
{
  auto msg = std::make_shared<armor_parameter::msg::Armor>();
  msg->pix_x = target_armor.x;
  msg->pix_y = target_armor.y;
  msg->x = target_armor.position_y;   //左右
  msg->y = target_armor.position_z;   //上下
  msg->z = target_armor.position_x;   //前后
  pub_x_y_z_->publish(*msg);
}

// 装甲板标记
void Armor_Node::ArmorMark(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
    cv::Mat src, edges, rvec, tvec;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<armorDetector::Light> light_Rects;

    // 图像处理
    auto src_ = cv_bridge::toCvShare(image, "bgr8");
    src = src_->image;
    ProcessImage(src, edges, 150);
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 寻找灯条
    for (int i = 0; i < contours.size(); ++i)
    {
        cv::Point2f vec[4];
        cv::RotatedRect minRects = cv::minAreaRect(cv::Mat(contours[i]));
        minRects.points(vec);

        // 判断是否是符合条件的灯条
        if (armorDetector_->IsLight(minRects)) 
        {
            armorDetector::Light minRects_ = {minRects, minRects.size.width * minRects.size.height, minRects.angle};
            light_Rects.push_back(minRects_);
        }
    }
    
    // 灯条按x的大小降序排序
    std::sort(light_Rects.begin(), light_Rects.end(), compare);

    // 灯条匹配
    if (light_Rects.size() >= 2)
    {
        for (int i = 0; i + 1 < light_Rects.size(); i++)
        {
            armorDetector::Light left_light = light_Rects[i];
            armorDetector::Light right_light = light_Rects[i + 1];

            // 装甲板匹配
            if (armorDetector_->MatchArmor(left_light, right_light))
            {
                armorDetector::Armor target_armor = {left_light, right_light};
                armorDetector_->GetArmorXYZ(target_armor);

                // pnp解算
                bool success = pnp_solver_->solvePnP(target_armor, rvec, tvec);
                if (success)
                {
                    target_armor.position_x = tvec.at<double>(0);    //原来为x——到图像的距离(前后)                      
                    target_armor.position_y = tvec.at<double>(1);    //原来为y——平行于水平线且垂直于图像中心到相机中心(左右)
                    target_armor.position_z = tvec.at<double>(2);    //原来为z——垂直于地面(上下)

                    std::cout << "x:" << target_armor.position_x << " y:" << target_armor.position_y << " z:" << target_armor.position_z << std::endl;
                    // 转换为3×3旋转矩阵
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvec, rotation_matrix);
                    // 旋转矩阵转换为四元数
                    tf2::Matrix3x3 tf2_rotation_matrix(
                        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                        rotation_matrix.at<double>(2, 2));
                    tf2::Quaternion tf2_q;
                    tf2_rotation_matrix.getRotation(tf2_q);
                    // target_armor.tf2_q = tf2_q;
                    //发送坐标
                    SendXYZ(target_armor);
                }
                // 显示装甲板
                DrawArmor(src, target_armor);
                i++;
            }
        }
    }
    // 图像显示
    SendImage(src);// 图像发送
    //cv::imshow("video", src);
    //cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Armor_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
