// 装甲板预测  标记当前装甲板与下一时刻装甲板的位置 发送装甲板坐标

#include "Kalman_predict.h"
#include "armor_parameter/msg/armor.hpp"

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"
// opencv
#include "opencv2/opencv.hpp"

// std
#include <thread>

// 装甲板预测
class Armor_Predict_Node : public rclcpp::Node
{
public:
    Armor_Predict_Node() : Node("predict")
    {
        RCLCPP_INFO(this->get_logger(), "Armor_Predict_Node working...");
        // 创建坐标接收节点
        sub_x_y_z = this->create_subscription<armor_parameter::msg::Armor>("camera/xyz", 10,
                                                                           std::bind(&Armor_Predict_Node::GetXYZ, this, std::placeholders::_1));
        // 创建图像接收节点
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_track", 10,
                                                                  std::bind(&Armor_Predict_Node::ArmorPredict, this, std::placeholders::_1));
        // 创建图像发布节点
        image_pub_ = image_transport::create_publisher(this, "camera/result");
        // 初始化卡尔曼类
        kalman_ = std::make_shared<kalman_armor>();
        kalman_->Init(3, 1, 0);
        /// singer模型
        kalman_->SingerInit(5, 5, 1, 0.01, 1);
        kalman_->KfinitSinger();

        // thread
        image_thread = std::thread(&Armor_Predict_Node::ImageThread, this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;          // 接收图像
    rclcpp::Subscription<armor_parameter::msg::Armor>::SharedPtr sub_x_y_z; // 接收坐标
    image_transport::Publisher image_pub_;                                  // 图像发布 使用rqt接收

    // 图像处理
    void ArmorPredict(const sensor_msgs::msg::Image::ConstSharedPtr &image);

    // 坐标获取
    void GetXYZ(const armor_parameter::msg::Armor::ConstSharedPtr &xyz);

    // 卡尔曼类
    std::shared_ptr<kalman_armor> kalman_;

    // thread
    std::thread image_thread;

    void ImageThread();

    cv::Mat result;

    // 互斥锁
    std::mutex result_mutex;
};

void Armor_Predict_Node::GetXYZ(const armor_parameter::msg::Armor::ConstSharedPtr &xyz)
{
    cv::Mat measurement;
    if (kalman_->update_armor_ == 0)
    {
        //          初始化装甲板
        //      new_armor = target_armor;
        kalman_->new_armor.x = xyz->pix_x;
        kalman_->new_armor.y = xyz->pix_y;
        kalman_->new_armor.position_x = xyz->x;
        kalman_->new_armor.position_y = xyz->y;
        kalman_->new_armor.position_z = xyz->z;
        //      last_armor = target_armor;
        kalman_->last_armor = kalman_->new_armor;

        kalman_->kalman.statePost = (cv::Mat_<float>(3, 1) << kalman_->new_armor.y, 0, 0);
        kalman_->update_armor_ = 1;
    }
    else
    {
        //           更新装甲板
        //    last_armor =   new_armor;
        kalman_->last_armor = kalman_->new_armor;

        //     new_armor =   target_armor;
        kalman_->new_armor.x = xyz->pix_x;
        kalman_->new_armor.y = xyz->pix_y;
        kalman_->new_armor.position_x = xyz->x;
        kalman_->new_armor.position_y = xyz->y;
        kalman_->new_armor.position_z = xyz->z;
        kalman_->new_armor.armor_speed = kalman_->kalman.statePost.at<float>(1);

        if (!kalman_->IsSameArmor())
        {
            // last_armor = target_armor;
            kalman_->last_armor.x = xyz->pix_x;
            kalman_->last_armor.y = xyz->pix_y;
            kalman_->last_armor.position_x = xyz->x;
            kalman_->last_armor.position_y = xyz->y;
            kalman_->last_armor.position_z = xyz->z;
            // 重置卡尔曼滤波
            kalman_->kalman.statePost.at<float>(0) = kalman_->new_armor.x; // 更新位置
            kalman_->kalman.statePost.at<float>(1) = 0;                    // 重置速度
        }
    }
    measurement = (cv::Mat_<float>(1, 1) << kalman_->new_armor.x);
    kalman_->KalmanPredict();
    kalman_->KalmanUpdata(measurement);
    kalman_->new_armor.pre_x = kalman_->kalman.statePost.at<float>(0);
    kalman_->new_armor.pre_y = kalman_->new_armor.y;
}

void Armor_Predict_Node::ArmorPredict(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
    cv::Mat src;
    // 图像处理
    auto src_ = cv_bridge::toCvShare(image, "bgr8");
    src = src_->image;

    std::lock_guard<std::mutex> lock(result_mutex);
     
    if (kalman_->new_armor.pre_x != 0)
    {
        cv::circle(src, cv::Point(kalman_->new_armor.pre_x, kalman_->new_armor.pre_y), 10, cv::Scalar(0, 0, 255), 5);
    }
    result = src.clone(); //确保深拷贝
    //cv::imshow("video", src);
    //cv::waitKey(1);
}

void Armor_Predict_Node::ImageThread()
{
    rclcpp::Rate rate(120);
    while (rclcpp::ok()) {
        cv::Mat current_result;
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            if (result.empty()) continue;
            current_result = result.clone();
        }
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";
        auto msg = cv_bridge::CvImage(header, "bgr8", current_result).toImageMsg();
        image_pub_.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Armor_Predict_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}