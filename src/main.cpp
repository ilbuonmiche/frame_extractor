#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
// #include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
#include <sstream>
#include <iomanip>
#include <frame_extractor/GlobalPose.h>

static int im_num = 1;

static std::string enc = sensor_msgs::image_encodings::BGR8;
static std::string path = "/home/michele/Pictures/Images";
static std::string img_topic = "/FLUENCE/front/right/color/image_raw";
// static std::string path = "/media/michele/TOSHIBA/Nantes_city_centre_dataset/Images";

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;
    // cv_bridge::CvImageConstPtr cv_ptr;
    
    std::stringstream sstring;
    
    sstring << path << "/" << "Image" << std::setfill('0') << std::setw(5) << im_num << ".png";
    im_num++;
    
    try
    {
	  cv_ptr = cv_bridge::toCvCopy(msg, enc);
      // cv_ptr = cv_bridge::toCvShare(msg, enc);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    cv::Rect myROI(0, 0, 1280, 850);
    
    cv::Mat croppedImage = cv_ptr->image(myROI);
    

    // Update GUI Window
    // cv::imshow("Image window", cv_ptr->image);
    cv::imshow("Image window", croppedImage);
    cv::waitKey(3);

    // cv::imwrite( sstring.str(), cv_ptr->image );
    cv::imwrite( sstring.str(), croppedImage );
    
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_extractor");
  // ImageConverter ic;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  image_sub_ = it_.subscribe(img_topic, 100, &imageCb);
  
  cv::namedWindow("Image window");
  
  ros::spin();
  
  cv::destroyWindow("Image window");
  
  return 0;
}

