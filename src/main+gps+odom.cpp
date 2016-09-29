#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <frame_extractor/GlobalPose.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// #include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <stdlib.h>
#include <pthread.h>

static int im_num = 0;
// static int im_num = 1;
// static int im_num = 6556;
// static int im_num = 11616;
// static int im_num = 7141;
// static int im_num = 6849;    // cloudy run

static std::string enc = sensor_msgs::image_encodings::BGR8;
static std::string path = "/home/michele/Pictures/Images";
// static std::string path = "/media/michele/TOSHIBA EXT1/Nantes_city_centre_dataset/Images2";
static std::string img_topic = "/FLUENCE/front/right/color/image_raw";				// normal lens
// static std::string img_topic = "/FLUENCE/front/left/color/image_raw";           // fisheye
// static std::string img_topic = "/FLUENCE/front/left/color/image_rectified";     // rectified channel
static std::string gt_topic = "/FLUENCE/position/GroundTruth";
static std::string odom_topic = "/FLUENCE/odometry/attitude";
static double lat;
static double lon;
static double vy;
static double x;
static double y;
static std::ofstream myfile1;
static std::ofstream myfile2;
static std::ofstream myfile3;
static std::ofstream myfile4;
static std::ifstream readfile;

// initialization of mutexes and conditions
pthread_mutex_t mutex_imageCb = PTHREAD_MUTEX_INITIALIZER;


  void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
	// std::cout << "Ciao sono un callback! " << im_num << std::endl;
	  
	  
	cv_bridge::CvImagePtr cv_ptr;
    // cv_bridge::CvImageConstPtr cv_ptr;
    
    std::stringstream sstring;
    
    sstring << path << "/" << "Image" << std::setfill('0') << std::setw(5) << im_num << ".png";
    im_num++;
    
    try
    {
	  cv_ptr = cv_bridge::toCvCopy(image_msg, enc);
      // cv_ptr = cv_bridge::toCvShare(image_msg, enc);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    cv::Rect myROI(0, 0, 1280, 850);           // perfect for normal lens
    // cv::Rect myROI(0, 0, 1280, 750);              // quite good for fisheye but still a part of the car is visible
    // cv::Rect myROI(0, 0, 1280, 620);			  // completely without car in fisheye
    
    cv::Mat croppedImage = cv_ptr->image(myROI);
    

    // Update GUI Window
    // cv::imshow("Image window", cv_ptr->image);
    // cv::imshow("Image window", croppedImage);
    // cv::waitKey(3);

    // cv::imwrite( sstring.str(), cv_ptr->image );
    cv::imwrite( sstring.str(), croppedImage );
    
    pthread_mutex_lock(&mutex_imageCb);
    
    myfile1 << im_num-1 << "\t";
    myfile1 << std::setprecision(17) << lat << "\n";
    myfile2 << im_num-1 << "\t";
    myfile2 << std::setprecision(17) << lon << "\n";
    myfile3 << im_num-1 << "\t";
    myfile3 << std::setprecision(17) << vy << "\n";
    myfile4 << im_num-1 << "\t";
    myfile4 << std::setprecision(17) << x << "\t" << y << "\n";
    
    pthread_mutex_unlock(&mutex_imageCb);
    
  }
  
  
  
  void groundTruthCB(const frame_extractor::GlobalPose::ConstPtr& gps_msg)
  {
	  lat = gps_msg->latitude;
	  lon = gps_msg->longitude;
  }
  
  
  
  void odometryCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
	  vy = odom_msg->twist.twist.linear.y;
	  x = odom_msg->pose.pose.position.x;
	  y = odom_msg->pose.pose.position.y;
  }
  
  
  
  
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_extractor");
  // ImageConverter ic;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  
  readfile.open("/home/michele/Pictures/gt_latitude.txt", std::ios_base::in);
  
  int n = 0;
  std::string s;
  if(readfile.is_open()){
	while( std::getline( readfile, s ) ) 
	{
		n++;
	}
	im_num = n;
  }
  else
    im_num = 1;
	
  std::cout << "Starting from image num: " << im_num << std::endl;
  
  
  readfile.close();
  
  
  // open the files for the gps in append mode
  myfile1.open("/home/michele/Pictures/gt_latitude.txt", std::ios_base::app);
  myfile2.open("/home/michele/Pictures/gt_longitude.txt", std::ios_base::app);
  myfile3.open("/home/michele/Pictures/velocity.txt", std::ios_base::app);
  myfile4.open("/home/michele/Pictures/positions.txt", std::ios_base::app);
  
  myfile1 << "% sequence number	% latitude\n";
  myfile2 << "% sequence number	% longitude\n";
  myfile3 << "% sequence number	% linear_velocity\n";
  myfile4 << "% sequence number	% x  % y  \n";
  
  image_sub_ = it_.subscribe(img_topic, 10, &imageCb);
  
  ros::Subscriber sub_gt = nh_.subscribe(gt_topic, 1, &groundTruthCB);
  ros::Subscriber sub_odom = nh_.subscribe(odom_topic, 1, &odometryCB);  

  
  // cv::namedWindow("Image window");
  
  ros::spin();
  
  // cv::destroyWindow("Image window");
  
  myfile1.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();
  
  return 0;
}

