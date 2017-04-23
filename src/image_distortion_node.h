#ifndef image_distortion_node_H
#define image_distortion_node_H

//std
#include <iostream>
#include <sstream>

//std ros dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ros/package.h>

//openCV
#include "opencv2/highgui/highgui.hpp"

/** \brief Image distortion experiments
 * 
 * Image distortion experiments
 * 
 **/
class ImageDistortionNode
{
    protected:                    
        //ros node handle
        ros::NodeHandle nh_;
        
        //image transport, publisher and message
        image_transport::ImageTransport image_tp_;
        image_transport::Publisher image_publisher_;      
        cv_bridge::CvImage image_;        
        
        //camera info publisher and message
        ros::Publisher camera_info_publisher_;
        sensor_msgs::CameraInfo camera_info_msg_;        
        
        //node configuration parameters
        double rate_; //loop rate
        std::string image_input_file_; //name of the inputimage file
        std::string camera_info_file_; //name of the camera info file. Where calibration data is stored
        
        //Calibration data and yaml file to store it
        cv::Mat paramsD; //distortion params
        cv::Mat matrixK; //intrinsics matrix
        cv::Mat matrixP; //projective matrix
        cv::FileStorage calibration_file_; //calibration file object 
        
    public:
        //constructor
        ImageDistortionNode();
        
        //destructor
        ~ImageDistortionNode();
        
        //returns rate_ value
        double rate() const; 
        
        //Publish the input image, the camera info and the manually distorted image
        void publish();
                        
    protected: 
        //sets calibration data from file. Retunrns false if file does not exist
        bool setCalibrationFromFile();

};
#endif
