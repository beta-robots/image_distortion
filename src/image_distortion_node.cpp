#include "image_distortion_node.h"

ImageDistortionNode::ImageDistortionNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_), 
    paramsD(5,1,CV_64FC1),
    matrixK(3,3,CV_64FC1),
    matrixP(3,4,CV_64FC1), 
    calibration_file_()
{
    //local vars
    std::ostringstream sstream; 
    std::string package_path;
    
    //node params & input files
    rate_ = 1;
    package_path = ros::package::getPath("image_distortion");
    sstream << package_path << "/image/image_source.png";
    image_input_file_ = sstream.str(); 
    sstream.str(""); //clear previous content
    sstream << package_path << "/calibration/calibration_data_01.yaml";
    camera_info_file_ = sstream.str(); 
    
    //get image from file
    image_.image = cv::imread(image_input_file_, cv::IMREAD_GRAYSCALE);

    //init the image publisher
    image_publisher_ = image_tp_.advertise("image_raw", 1);
    
    //init the camera_info publisher
    camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    
    //sets calibration data from calibration file
    if ( setCalibrationFromFile() )
    {
        std::cout << "Calibration data load from file. camera_info topic will publish these data." << std::endl;
    }
    else //some error occurred
    {
        std::cout << "WARNING! Calibration file not found. camera_info topic will publish incorrect data." << std::endl;
    }
    
}
        
ImageDistortionNode::~ImageDistortionNode()
{

}
        
double ImageDistortionNode::rate() const
{
    return rate_; 
}
        
void ImageDistortionNode::publish()
{
    //Get timestamp and fill the header
    ros::Time ts = ros::Time::now();
    image_.header.seq ++;
    image_.header.stamp = ts;
    image_.header.frame_id = "camera"; 
    image_.encoding = sensor_msgs::image_encodings::MONO8;

    //publish the image    
    image_publisher_.publish(image_.toImageMsg());        
    
    //fill the camera info message
    camera_info_msg_.header.seq ++;
    camera_info_msg_.header.stamp = ts;
    camera_info_msg_.header.frame_id = "camera"; 
    camera_info_msg_.height = image_.image.rows; 
    camera_info_msg_.width = image_.image.cols;     
    camera_info_msg_.distortion_model = "plumb_bob"; 
    camera_info_msg_.D.resize(5); 
    for(unsigned int ii=0; ii<5; ii++) camera_info_msg_.D[ii] = paramsD.at<double>(ii);
    for(unsigned int ii=0; ii<3; ii++)
        for(unsigned int jj=0; jj<3; jj++) camera_info_msg_.K[ii*3+jj] = matrixK.at<double>(ii,jj);
    for(unsigned int ii=0; ii<3; ii++)
        for(unsigned int jj=0; jj<4; jj++) camera_info_msg_.P[ii*4+jj] = matrixP.at<double>(ii,jj);        
    camera_info_msg_.R[0] = 1.; //R is the identity (no rotation)
    camera_info_msg_.R[4] = 1.; //R is the identity (no rotation)
    camera_info_msg_.R[8] = 1.; //R is the identity (no rotation)
    camera_info_msg_.binning_x = 1;
    camera_info_msg_.binning_y = 1;
    camera_info_msg_.roi.width = 0;
    camera_info_msg_.roi.height = 0; 
    camera_info_msg_.roi.do_rectify = false; 

    //publish the camera info
    camera_info_publisher_.publish(camera_info_msg_);
}
                        
bool ImageDistortionNode::setCalibrationFromFile()
{
    //open calibration data file
    calibration_file_.open(camera_info_file_, cv::FileStorage::READ); 
    if ( calibration_file_.isOpened() )
    {
        calibration_file_["paramsD"] >> paramsD;
        calibration_file_["matrixK"] >> matrixK;
        calibration_file_["matrixP"] >> matrixP;
        calibration_file_.release(); 
        return true; 
    }
    else //file not opened
    {
        calibration_file_.release(); 
        return false; 
    }

}
