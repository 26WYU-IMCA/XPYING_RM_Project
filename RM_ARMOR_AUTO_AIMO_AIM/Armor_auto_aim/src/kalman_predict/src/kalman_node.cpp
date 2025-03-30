// 装甲板预测  标记当前装甲板与下一时刻装甲板的位置 发送装甲板坐标

#include "armor_parameter/msg/armor.hpp"
#include "Kalman_Filter.h"
#include "Armor_match.h"

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
#include <eigen3/Eigen/Eigen>

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

        //初始化装甲板类
        armor_ = std::make_shared<armorDetector>(10, 15, 2.5);

        // 初始化卡尔曼类
        //armor_ = std::make_shared<kalman_armor>();
        // armor_->Init(3, 1, 0);

        kalman_filter_ = std::make_shared<Kalman_Filter>();
        kalman_filter_->Init(3,1,1);
        kalman_filter_->SingerInit(5,5,1,0.01,1);
        /// singer模型
        // armor_->SingerInit(5, 5, 1, 0.01, 1);
        // armor_->KfinitSinger();
          
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
    
    //装甲板类
    std::shared_ptr<armorDetector>armor_;

    // 卡尔曼类
    //std::shared_ptr<kalman_armor> armor_;

    std::shared_ptr<Kalman_Filter> kalman_filter_;

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
    if (armor_->update_armor_ == 0)
    {
        //          初始化装甲板
        //      new_armor = target_armor;
        armor_->new_armor.x = xyz->pix_x;
        armor_->new_armor.y = xyz->pix_y;
        armor_->new_armor.position_x = xyz->x;
        armor_->new_armor.position_y = xyz->y;
        armor_->new_armor.position_z = xyz->z;
        //      last_armor = target_armor;
        armor_->last_armor = armor_->new_armor;

        // armor_->kalman.statePost = (cv::Mat_<float>(3, 1) << armor_->new_armor.y, 0, 0);

        kalman_filter_->stateOpt<<armor_->new_armor.y, 0, 0;

        armor_->update_armor_ = 1;
    }
    else
    {
        //           更新装甲板
        //    last_armor =   new_armor;
        armor_->last_armor = armor_->new_armor;

        //     new_armor =   target_armor;
        armor_->new_armor.x = xyz->pix_x;
        armor_->new_armor.y = xyz->pix_y;
        armor_->new_armor.position_x = xyz->x;
        armor_->new_armor.position_y = xyz->y;
        armor_->new_armor.position_z = xyz->z;
        // armor_->new_armor.armor_speed = armor_->kalman.statePost.at<float>(1);
        armor_->new_armor.armor_speed =kalman_filter_->stateOpt(1);
        
        if (!armor_->IsSameArmor())
        {
            // last_armor = target_armor;
            armor_->last_armor.x = xyz->pix_x;
            armor_->last_armor.y = xyz->pix_y;
            armor_->last_armor.position_x = xyz->x;
            armor_->last_armor.position_y = xyz->y;
            armor_->last_armor.position_z = xyz->z;
                                 // 重置卡尔曼滤波
            // 更新位置
            // armor_->kalman.statePost.at<float>(0) = armor_->new_armor.x; 
            kalman_filter_->stateOpt(0) = armor_->new_armor.x;
            // 重置速度
            // armor_->kalman.statePost.at<float>(1) = 0;                    
            kalman_filter_->stateOpt(1) = 0;
        }
    }
    // measurement = (cv::Mat_<float>(1, 1) << armor_->new_armor.x);
    // armor_->KalmanPredict();
    // armor_->KalmanUpdata(measurement);
    // armor_->new_armor.pre_x = armor_->kalman.statePost.at<float>(0);
    armor_->new_armor.pre_y = armor_->new_armor.y;

    kalman_filter_->measurement << armor_->new_armor.x;
    kalman_filter_->Predict();
    kalman_filter_->Update();

}

void Armor_Predict_Node::ArmorPredict(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
    cv::Mat src;
    // 图像处理
    auto src_ = cv_bridge::toCvShare(image, "bgr8");
    src = src_->image;

    std::lock_guard<std::mutex> lock(result_mutex);
     
    if (kalman_filter_->stateOpt(0)!=0)
    {
        // cv::circle(src, cv::Point(armor_->new_armor.pre_x, armor_->new_armor.pre_y), 10, cv::Scalar(0, 0, 255), 5);
        cv::circle(src, cv::Point(kalman_filter_->stateOpt(0), armor_->new_armor.pre_y), 10, cv::Scalar(0, 0, 255), 5);
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