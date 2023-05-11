#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

//include all messages for syncing
#include "sensor_msgs/msg/image.hpp"


#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace cv;


class CameraLeptonSynchroniserNode : public rclcpp::Node 
{
public:
    CameraLeptonSynchroniserNode() : Node("camera_sync_node_lepton") 
    {
    RCLCPP_INFO(this->get_logger(), "Camera Sync Node started");

    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // Define quality of service: all messages that you want to receive must have the same
    //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    //custom_qos_profile.depth = 1;
    //custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    //custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
   
    
    // Initialise subscribers to the topics of interest
    lepton_sub_.subscribe(this, "/lepton1/lepton/image_raw", rmw_qos_profile);
    realsense_rgb_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile);
    realsense_depth_sub_.subscribe(this, "/camera/depth/image_rect_raw", rmw_qos_profile);

   

    // Individual subscribers
    lepton_sub_.registerCallback(&CameraLeptonSynchroniserNode::lepton_callback, this);
    realsense_rgb_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_rgb_callback, this);
    realsense_depth_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_depth_callback, this);

    
    
    // Create an approximate time filter
    syncApproximate.reset(new ApproxSync(approximate_policy(100), realsense_rgb_sub_, realsense_depth_sub_, lepton_sub_));  

    // Register the approximate time callback.
    syncApproximate->registerCallback(&CameraLeptonSynchroniserNode::approximate_sync_callback, this);
    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0,10000000)); 

    }
 
private:
    

    
    void lepton_callback(const sensor_msgs::msg::Image::SharedPtr &lepton_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following LEPTON message: %u. %u", lepton_msg->header.stamp.sec, lepton_msg->header.stamp.nanosec);  
    }

    void realsense_rgb_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense RGB message: %u. %u", realsense_rgb_msg->header.stamp.sec, realsense_rgb_msg->header.stamp.nanosec);
    }
    
   void realsense_depth_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_depth_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Depth message: %u. %u", realsense_depth_msg->header.stamp.sec, realsense_depth_msg->header.stamp.nanosec);
    }

    void approximate_sync_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg, const sensor_msgs::msg::Image::SharedPtr &realsense_depth_msg, const sensor_msgs::msg::Image::SharedPtr &lepton_msg)
    {   

        RCLCPP_INFO(this->get_logger(), "Received APPROXIMATE msgs");
        
        // write to image using opencv
        cv_bridge::CvImagePtr cv_ptr_rs_rgb, cv_ptr_rs_depth, cv_ptr_lepton;
        cv_ptr_lepton = cv_bridge::toCvCopy(lepton_msg, lepton_msg->encoding);
        cv_ptr_rs_rgb = cv_bridge::toCvCopy(realsense_rgb_msg, realsense_rgb_msg->encoding);
        cv_ptr_rs_depth = cv_bridge::toCvCopy(realsense_depth_msg, realsense_depth_msg->encoding);

        std::string img_name_lepton = ("/home/orin/m2s2_ws/Experiments/20230509/lepton_realsense/lepton/" + std::to_string(count) + ".png");
        std::string img_name_rs_rgb = ("/home/orin/m2s2_ws/Experiments/20230509/lepton_realsense/realsense_rgb/" + std::to_string(count) + ".png");        //std::string img_name_ximea = ("/home/orin/m2s2_ws/Experiments/20230426/ximea/" + std::to_string(count) + ".png");
        std::string img_name_rs_depth = ("/home/orin/m2s2_ws/Experiments/20230509/lepton_realsense/realsense_depth/" + std::to_string(count) + ".png");        //std::string img_name_ximea = ("/home/orin/m2s2_ws/Experiments/20230426/ximea/" + std::to_string(count) + ".png");

        cv::imwrite(img_name_lepton, cv_ptr_lepton->image);
        cv::imwrite(img_name_rs_rgb, cv_ptr_rs_rgb->image);
        cv::imwrite(img_name_rs_depth, cv_ptr_rs_depth->image);
    
        count = count+1;


    }

    // Subscribers to individual topics
    message_filters::Subscriber<sensor_msgs::msg::Image> lepton_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_depth_sub_;

    // Message filter stuff
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
    typedef message_filters::Synchronizer<approximate_policy> ApproxSync;
    std::shared_ptr<ApproxSync> syncApproximate;

    int count = 0;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraLeptonSynchroniserNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

