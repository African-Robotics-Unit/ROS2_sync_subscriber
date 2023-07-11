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
    lepton1_sub_.subscribe(this, "/lepton1/lepton/image_raw", rmw_qos_profile);
    lepton2_sub_.subscribe(this, "/lepton2/lepton2/image_raw", rmw_qos_profile);
    realsense_rgb_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile);
    //realsense_depth_sub_.subscribe(this, "/camera/depth/image_rect_raw", rmw_qos_profile);
    ximea_sub_.subscribe(this, "/ximea/image_raw", rmw_qos_profile);
    realsense_infra1_sub_.subscribe(this, "/camera/infra1/image_rect_raw", rmw_qos_profile);
    realsense_infra2_sub_.subscribe(this, "/camera/infra2/image_rect_raw", rmw_qos_profile);
    boson_sub_.subscribe(this, "/boson/image_raw", rmw_qos_profile);

   

    // Individual subscribers
    lepton1_sub_.registerCallback(&CameraLeptonSynchroniserNode::lepton1_callback, this);
    lepton2_sub_.registerCallback(&CameraLeptonSynchroniserNode::lepton2_callback, this);
    realsense_rgb_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_rgb_callback, this);
    //realsense_depth_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_depth_callback, this);
    ximea_sub_.registerCallback(&CameraLeptonSynchroniserNode::ximea_callback, this);
    realsense_infra1_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_infra1_callback, this);
    realsense_infra2_sub_.registerCallback(&CameraLeptonSynchroniserNode::realsense_infra2_callback, this);
    boson_sub_.registerCallback(&CameraLeptonSynchroniserNode::boson_callback, this);

    
    
    // Create an approximate time filter
    syncApproximate.reset(new ApproxSync(approximate_policy(100), boson_sub_, realsense_rgb_sub_, ximea_sub_, lepton1_sub_, lepton2_sub_, realsense_infra1_sub_, realsense_infra2_sub_));  

    // Register the approximate time callback.
    syncApproximate->registerCallback(&CameraLeptonSynchroniserNode::approximate_sync_callback, this);
    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0,100000000)); 

    }
 
private:
    

    
    void lepton1_callback(const sensor_msgs::msg::Image::SharedPtr &lepton1_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following LEPTON message: %u. %u", lepton_msg->header.stamp.sec, lepton_msg->header.stamp.nanosec);  
    }

    void lepton2_callback(const sensor_msgs::msg::Image::SharedPtr &lepton2_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following LEPTON message: %u. %u", lepton_msg->header.stamp.sec, lepton_msg->header.stamp.nanosec);  
    }

    void realsense_rgb_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense RGB message: %u. %u", realsense_rgb_msg->header.stamp.sec, realsense_rgb_msg->header.stamp.nanosec);
    }
    
   /*void realsense_depth_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_depth_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Depth message: %u. %u", realsense_depth_msg->header.stamp.sec, realsense_depth_msg->header.stamp.nanosec);
    }*/

    void ximea_callback(const sensor_msgs::msg::Image::SharedPtr &ximea_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Depth message: %u. %u", realsense_depth_msg->header.stamp.sec, realsense_depth_msg->header.stamp.nanosec);
    }

    void realsense_infra1_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_infra1_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Infra1 message: %u. %u", realsense_infra1_msg->header.stamp.sec, realsense_infra1_msg->header.stamp.nanosec);

    }

    void realsense_infra2_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_infra2_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Infra2 message: %u. %u", realsense_infra2_msg->header.stamp.sec, realsense_infra2_msg->header.stamp.nanosec);

    }

    void boson_callback(const sensor_msgs::msg::Image::SharedPtr &boson_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Infra2 message: %u. %u", realsense_infra2_msg->header.stamp.sec, realsense_infra2_msg->header.stamp.nanosec);

    }

    void approximate_sync_callback(const sensor_msgs::msg::Image::SharedPtr &boson_msg, const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg, const sensor_msgs::msg::Image::SharedPtr &ximea_msg, const sensor_msgs::msg::Image::SharedPtr &lepton1_msg, const sensor_msgs::msg::Image::SharedPtr &lepton2_msg, const sensor_msgs::msg::Image::SharedPtr &realsense_infra1_msg, const sensor_msgs::msg::Image::SharedPtr &realsense_infra2_msg)
    {   

        RCLCPP_INFO(this->get_logger(), "Received APPROXIMATE msgs");
        
        // write to image using opencv
        if (count%10==0){
            cv_bridge::CvImagePtr cv_ptr_lepton1, cv_ptr_lepton2, cv_ptr_rs_rgb, cv_ptr_ximea, cv_ptr_rs_infra1, cv_ptr_rs_infra2, cv_ptr_boson;
            cv_ptr_lepton1 = cv_bridge::toCvCopy(lepton1_msg, lepton1_msg->encoding);
            cv_ptr_lepton2 = cv_bridge::toCvCopy(lepton2_msg, lepton2_msg->encoding);
            cv_ptr_rs_rgb = cv_bridge::toCvCopy(realsense_rgb_msg, realsense_rgb_msg->encoding);
            cv_ptr_ximea = cv_bridge::toCvCopy(ximea_msg, ximea_msg->encoding);
            //cv_ptr_rs_depth = cv_bridge::toCvCopy(realsense_depth_msg, realsense_depth_msg->encoding);
            cv_ptr_rs_infra1 = cv_bridge::toCvCopy(realsense_infra1_msg, realsense_infra1_msg->encoding);
            cv_ptr_rs_infra2 = cv_bridge::toCvCopy(realsense_infra2_msg, realsense_infra2_msg->encoding);
            cv_ptr_boson = cv_bridge::toCvCopy(boson_msg, boson_msg->encoding);


            std::string path = "/home/m2s2/Nutramax_Data/Calibration_Data/20230619/test2/";
            

            std::string img_name_lepton1 =   (path + "lepton1/" + std::to_string(frame_no) + ".png");
            std::string img_name_lepton2 =   (path + "lepton2/" + std::to_string(frame_no) + ".png");
            std::string img_name_rs_rgb =    (path + "rs_rgb/" + std::to_string(frame_no) + ".png");
            std::string img_name_ximea  =    (path + "ximea/" + std::to_string(frame_no) + ".png");
            //std::string img_name_rs_depth =  (path + "rs_depth/" + std::to_string(frame_no) + ".png");        
            std::string img_name_rs_infra1 = (path + "rs_infra1/" + std::to_string(frame_no) + ".png");
            std::string img_name_rs_infra2 = (path + "rs_infra2/" + std::to_string(frame_no) + ".png");
            std::string img_name_boson = (path + "boson/" + std::to_string(frame_no) + ".png");


            cv::imwrite(img_name_lepton1, cv_ptr_lepton1->image);
            cv::imwrite(img_name_lepton2, cv_ptr_lepton2->image);
            cv::imwrite(img_name_rs_rgb, cv_ptr_rs_rgb->image);
            cv::imwrite(img_name_ximea, cv_ptr_ximea->image);
            //cv::imwrite(img_name_rs_depth, cv_ptr_rs_depth->image);
            cv::imwrite(img_name_rs_infra1, cv_ptr_rs_infra1->image);
            cv::imwrite(img_name_rs_infra2, cv_ptr_rs_infra2->image);
            cv::imwrite(img_name_boson, cv_ptr_boson->image);

            frame_no = frame_no+1;
        }

        count = count+1;


    }

    // Subscribers to individual topics
    message_filters::Subscriber<sensor_msgs::msg::Image> lepton1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> lepton2_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> ximea_sub_;
    //message_filters::Subscriber<sensor_msgs::msg::Image> realsense_depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_infra1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_infra2_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> boson_sub_;


    // Message filter stuff
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
    typedef message_filters::Synchronizer<approximate_policy> ApproxSync;
    std::shared_ptr<ApproxSync> syncApproximate;

    int count = 0;
    int frame_no = 0;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraLeptonSynchroniserNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

