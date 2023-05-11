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


class CameraSynchroniserNode : public rclcpp::Node 
{
public:
    CameraSynchroniserNode() : Node("camera_sync_node") 
    {
    RCLCPP_INFO(this->get_logger(), "Camera Sync node started");

    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // Define quality of service: all messages that you want to receive must have the same
    //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    //custom_qos_profile.depth = 1;
    //custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    //custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
   
    
    // Initialise subscribers to the topics of interest
    boson_sub_.subscribe(this, "/boson/image_raw", rmw_qos_profile);
    //lepton_sub_.subscribe(this, "/lepton1/lepton/image_raw", rmw_qos_profile);
    realsense_rgb_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile);
    realsense_depth_sub_.subscribe(this, "/camera/depth/image_rect_raw", rmw_qos_profile);

    //ximea_sub_.subscribe(this, "/ximea/image_raw", rmw_qos_profile);
    //realsense_infra1_sub_.subscribe(this, "/camera/infra1/image_rect_raw", rmw_qos_profile);
    //realsense_infra2_sub_.subscribe(this, "/camera/infra2/image_rect_raw", rmw_qos_profile);


    // Individual subscribers
    boson_sub_.registerCallback(&CameraSynchroniserNode::boson_callback, this);
    //lepton_sub_.registerCallback(&CameraSynchroniserNode::lepton_callback, this);
    realsense_rgb_sub_.registerCallback(&CameraSynchroniserNode::realsense_rgb_callback, this);
    realsense_depth_sub_.registerCallback(&CameraSynchroniserNode::realsense_depth_callback, this);

    //ximea_sub_.registerCallback(&CameraSynchroniserNode::ximea_callback, this);
    //realsense_infra1_sub_.registerCallback(&CameraSynchroniserNode::realsense_infra1_callback, this);
    //realsense_infra2_sub_.registerCallback(&CameraSynchroniserNode::realsense_infra2_callback, this);
    

    
    
    // Create an approximate time filter
    syncApproximate.reset(new ApproxSync(approximate_policy(100), realsense_rgb_sub_, realsense_depth_sub_, boson_sub_));  

    // Register the approximate time callback.
    syncApproximate->registerCallback(&CameraSynchroniserNode::approximate_sync_callback, this);
    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0,10000000)); 
    

    // Publish synced image messages  
    //this->publisher_image_raw_synced_   = this->create_publisher<sensor_msgs::msg::Image>("synced_image", qos);
    //this->publisher_thermal_raw_synced_ = this->create_publisher<flir_lepton_msgs::msg::TemperatureRaw>("synced_thermal", qos);

    }
 
private:
    
    
    void boson_callback(const sensor_msgs::msg::Image::SharedPtr &boson_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following BOSON message: %u. %u", boson_msg->header.stamp.sec, boson_msg->header.stamp.nanosec);
        // Convert RAW16 data to 8 bits to show as ROS2 Image msg
        //this->thermal16 = cv::Mat(thermal_img_msg->height, thermal_img_msg->width, CV_16U, &thermal_img_msg->data[0]);
        //normalize(this->thermal16, this->thermal8, 0, 65535, cv::NORM_MINMAX);
        //this->thermal8.convertTo(this->thermal8,CV_8U,1./256.);
        
        //cv::bitwise_not(this->thermal8, this->thermal8);
        //std::string timestr = std::to_string(thermal_img_msg->header.stamp.sec) + std::to_string(thermal_img_msg->header.stamp.nanosec);
           
        //std::string img_name_thermal = ("/home/orin/m2s2_ws/cam_calib_parameters/boson_ximea/boson_imgs/" + timestr + ".png");
        //std::string img_name_thermal_raw = ("/home/orin/m2s2_ws/cam_calib_parameters/boson_ximea/boson_raw_imgs/" + timestr + ".png");
        
        //cv::imwrite(img_name_thermal, this->thermal8);
        //cv::imwrite(img_name_thermal_raw, this->thermal16);

        
    }

    /*
    void ximea_callback(const sensor_msgs::msg::Image::SharedPtr &ximea_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following XIMEA message: %u. %u", ximea_msg->header.stamp.sec, ximea_msg->header.stamp.nanosec);

        //write to image using opencv
        //cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_thermal;
        //cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img_msg, rgb_img_msg->encoding);
        //std::string timestr = std::to_string(rgb_img_msg->header.stamp.sec) + std::to_string(rgb_img_msg->header.stamp.nanosec);

        //std::string img_name_rgb = ("/home/orin/m2s2_ws/cam_calib_parameters/boson_ximea/ximea_imgs/" + timestr + ".png");
        //cv::imwrite(img_name_rgb, cv_ptr_rgb->image);



    }
    */

    
    /*void lepton_callback(const sensor_msgs::msg::Image::SharedPtr &lepton_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following LEPTON message: %u. %u", lepton_msg->header.stamp.sec, lepton_msg->header.stamp.nanosec);

        //write to image using opencv
        //cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_thermal;
        //cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img_msg, rgb_img_msg->encoding);
        //std::string timestr = std::to_string(rgb_img_msg->header.stamp.sec) + std::to_string(rgb_img_msg->header.stamp.nanosec);

        //std::string img_name_rgb = ("/home/orin/m2s2_ws/cam_calib_parameters/boson_ximea/ximea_imgs/" + timestr + ".png");
        //cv::imwrite(img_name_rgb, cv_ptr_rgb->image);
    }*/

    void realsense_rgb_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense RGB message: %u. %u", realsense_rgb_msg->header.stamp.sec, realsense_rgb_msg->header.stamp.nanosec);

    }

    void realsense_depth_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_depth_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Depth message: %u. %u", realsense_depth_msg->header.stamp.sec, realsense_depth_msg->header.stamp.nanosec);

    }

    /*void realsense_infra1_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_infra1_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Infra1 message: %u. %u", realsense_infra1_msg->header.stamp.sec, realsense_infra1_msg->header.stamp.nanosec);

    }

    void realsense_infra2_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_infra2_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I heard the following RealSense Infra2 message: %u. %u", realsense_infra2_msg->header.stamp.sec, realsense_infra2_msg->header.stamp.nanosec);

    }*/


    void approximate_sync_callback(const sensor_msgs::msg::Image::SharedPtr &realsense_rgb_msg, const sensor_msgs::msg::Image::SharedPtr &realsense_depth_msg, const sensor_msgs::msg::Image::SharedPtr &boson_msg)
    {   

        RCLCPP_INFO(this->get_logger(), "Received APPROXIMATE msgs");

        // Convert RAW16 data to 8 bits to show as ROS2 Image msg
        //this->thermal16 = cv::Mat(thermal_img_msg->height, thermal_img_msg->width, CV_16U, &thermal_img_msg->data[0]);
        //normalize(this->thermal16, this->thermal8, 0, 65535, cv::NORM_MINMAX);
        //this->thermal8.convertTo(this->thermal8,CV_8U,1./256.);
        //cv::bitwise_not(this->thermal8, this->thermal8);
        
        // write to image using opencv
        cv_bridge::CvImagePtr cv_ptr_rs_rgb, cv_ptr_rs_depth, cv_ptr_boson;
        cv_ptr_boson = cv_bridge::toCvCopy(boson_msg, boson_msg->encoding);
        //cv_ptr_lepton = cv_bridge::toCvCopy(lepton_msg, lepton_msg->encoding);
        cv_ptr_rs_rgb = cv_bridge::toCvCopy(realsense_rgb_msg, realsense_rgb_msg->encoding);
        cv_ptr_rs_depth = cv_bridge::toCvCopy(realsense_depth_msg, realsense_depth_msg->encoding);
        
        //cv_ptr_ximea = cv_bridge::toCvCopy(ximea_msg, ximea_msg->encoding);
        //cv_ptr_rs_infra1 = cv_bridge::toCvCopy(realsense_infra1_msg, realsense_infra1_msg->encoding);
        //cv_ptr_rs_infra2 = cv_bridge::toCvCopy(realsense_infra2_msg, realsense_infra2_msg->encoding);





        std::string img_name_boson = ("/home/orin/m2s2_ws/Experiments/20230509/boson_realsense/boson/" + std::to_string(count) + ".png");
        //std::string img_name_lepton = ("/home/orin/m2s2_ws/Experiments/20230426/lepton/" + std::to_string(count) + ".png");
        std::string img_name_rs_rgb = ("/home/orin/m2s2_ws/Experiments/20230509/boson_realsense/realsense_rgb/" + std::to_string(count) + ".png");        //std::string img_name_ximea = ("/home/orin/m2s2_ws/Experiments/20230426/ximea/" + std::to_string(count) + ".png");
        std::string img_name_rs_depth = ("/home/orin/m2s2_ws/Experiments/20230509/boson_realsense/realsense_depth/" + std::to_string(count) + ".png");        //std::string img_name_ximea = ("/home/orin/m2s2_ws/Experiments/20230426/ximea/" + std::to_string(count) + ".png");

        //std::string img_name_ximea = ("/home/orin/m2s2_ws/Experiments/20230426/ximea/" + std::to_string(count) + ".png");
        //std::string img_name_rs_infra1 = ("/home/orin/m2s2_ws/Experiments/20230426_realsense/rs_infra1/" + std::to_string(count) + ".png");
        //std::string img_name_rs_infra2 = ("/home/orin/m2s2_ws/Experiments/20230426_realsense/rs_infra2/" + std::to_string(count) + ".png");

        cv::imwrite(img_name_boson, cv_ptr_boson->image);
        //cv::imwrite(img_name_lepton, cv_ptr_lepton->image);
        cv::imwrite(img_name_rs_rgb, cv_ptr_rs_rgb->image);
        cv::imwrite(img_name_rs_depth, cv_ptr_rs_depth->image);


        //cv::imwrite(img_name_ximea, cv_ptr_ximea->image);
        //cv::imwrite(img_name_rs_infra1, cv_ptr_rs_infra1->image);
        //cv::imwrite(img_name_rs_infra2, cv_ptr_rs_infra2->image);
            

    
        
    
        count = count+1;


    }

    // Subscribers to individual topics
    message_filters::Subscriber<sensor_msgs::msg::Image> boson_sub_;
    //message_filters::Subscriber<sensor_msgs::msg::Image> lepton_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> realsense_depth_sub_;

    //message_filters::Subscriber<sensor_msgs::msg::Image> ximea_sub_;
    //message_filters::Subscriber<sensor_msgs::msg::Image> realsense_infra1_sub_;
    //message_filters::Subscriber<sensor_msgs::msg::Image> realsense_infra2_sub_;


    // Publishers for synced image messages
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_raw_synced_;
    //rclcpp::Publisher<flir_lepton_msgs::msg::TemperatureRaw>::SharedPtr publisher_thermal_raw_synced_;

    // Message filter stuff
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
    typedef message_filters::Synchronizer<approximate_policy> ApproxSync;
    std::shared_ptr<ApproxSync> syncApproximate;

    int count = 0;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSynchroniserNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

