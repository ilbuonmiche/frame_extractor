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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sys/types.h>
#include <sys/stat.h>

// This numeber is used to name the images that are extracted
// When a dataset is composed by more than one bag file be ware of
// carefully setting this numebr not to overwrite anything.
static int im_num = 0;
// static int im_num = 6556;
// static int im_num = 11616;
// static int im_num = 7141;


// Image encoding in the case of the FLUENCE car and the camera ...
static std::string enc = sensor_msgs::image_encodings::BGR8;

static std::string writePath = "/home/michele/Pictures";

/* Choose the image topic from which extract the images
	In our work just the right images of the stereo couple were used and
	recorded without any rectification
	In the image acquired it can be chosen to rectify the image directly
	before writing it on the disk.
*/
static std::string img_topic = "/FLUENCE/front/right/color/image_raw";				// normal lens
// static std::string img_topic = "/FLUENCE/front/left/color/image_raw";           // fisheye
// static std::string img_topic = "/FLUENCE/front/left/color/image_rectified";     // rectified channel

/* 
 * Along with the images and poses we also want to extract the ground 
 * truth and the IMU measurements
*/
static std::string gt_topic = "/FLUENCE/position/GroundTruth";
static std::string odom_topic = "/FLUENCE/odometry/attitude";

/* 
 * 
 * 
*/
static double im_secs;
static double gt_secs;
static double lat;
static double lon;
static double odom_secs;

static double vy;
static double x;
static double y;
static double a;
static double b;
static double c;
static double d;

static std::ofstream myfile0;
static std::ofstream myfile1;
static std::ofstream myfile2;
static std::ofstream myfile3;
static std::ofstream myfile4;
static std::ofstream myfile5;
static std::ifstream readfile;


void callback(const sensor_msgs::Image::ConstPtr& image_msg, const frame_extractor::GlobalPose::ConstPtr& gps_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	// std::cout << "Debug: callback is working, first im_num is " << im_num << std::endl;
	
	cv_bridge::CvImagePtr cv_ptr;
    // cv_bridge::CvImageConstPtr cv_ptr;
    
    // String used to name the images
    std::stringstream sstring;
    
    std::string images_path = writePath + "/Images";
    mkdir( images_path.c_str(), 0777 );
    
    sstring << images_path << "/" << "Image" << std::setfill('0') << std::setw(5) << im_num << ".png";
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
    
    im_secs = image_msg->header.stamp.toSec();
    
    gt_secs = gps_msg->header.stamp.toSec();
    lat = gps_msg->latitude;
	lon = gps_msg->longitude;
	
	
	odom_secs = odom_msg->header.stamp.toSec();
	
	vy = odom_msg->twist.twist.linear.y;
	x = odom_msg->pose.pose.position.x;
	y = odom_msg->pose.pose.position.y;
	
	a = odom_msg->pose.pose.orientation.x;
	b = odom_msg->pose.pose.orientation.y;
	c = odom_msg->pose.pose.orientation.z;
	d = odom_msg->pose.pose.orientation.w;
    
    // Resize the image in order to cut or reduce the visibility of the
    // car.
    cv::Rect myROI(0, 0, 1280, 850);           // perfect for normal lens
    // cv::Rect myROI(0, 0, 1280, 750);        // quite good for fisheye but still a part of the car is visible
    // cv::Rect myROI(0, 0, 1280, 620);		   // completely without car in fisheye
    
    // Apply the crop
    cv::Mat croppedImage = cv_ptr->image(myROI);
    

    // Update GUI Window
    // cv::imshow("Image window", cv_ptr->image);
    // cv::imshow("Image window", croppedImage);
    // cv::waitKey(3);

    // Writing on the disk
    // cv::imwrite( sstring.str(), cv_ptr->image );
    cv::imwrite( sstring.str(), croppedImage );
    
    myfile0 << im_num-1 << "\t";
    myfile0 << im_secs << "\n";
    
    myfile1 << im_num-1 << "\t";
    myfile1 << gt_secs << "\t";
    myfile1 << lat << "\n";
    
    myfile2 << im_num-1 << "\t";
    myfile2 << gt_secs << "\t";
    myfile2 << lon << "\n";
    
    myfile3 << im_num-1 << "\t";
    myfile3 << odom_secs << "\t";
    myfile3 << vy << "\n";
    
    myfile4 << im_num-1 << "\t";
    myfile4 << odom_secs << "\t";
    myfile4 << x << "\t" << y << "\n";
    
    myfile5 << im_num-1 << "\t";
    myfile5 << odom_secs << "\t";
    myfile5 << a << "\t" << b << "\t" << c << "\t" << d << "\n";


}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_extractor");
  // ImageConverter ic;
  
  ros::NodeHandle nh_;
  
  // Attempt to automate the process of image number setting in the case
  // of multiple bag made dataset
  std::string path = writePath + "/im_stamps.txt";
  readfile.open( path.c_str(), std::ios_base::in);
  
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
  path = writePath + "/im_stamps.txt";
  myfile0.open( path.c_str(), std::ios_base::app);
  path = writePath + "/gt_latitude.txt";
  myfile1.open( path.c_str(), std::ios_base::app);
  path = writePath + "/gt_longitude.txt";
  myfile2.open( path.c_str(), std::ios_base::app);
  path = writePath + "/velocity.txt";
  myfile3.open( path.c_str(), std::ios_base::app);
  path = writePath + "/positions.txt";
  myfile4.open( path.c_str(), std::ios_base::app);
  path = writePath + "/orientations.txt";
  myfile5.open( path.c_str(), std::ios_base::app);
  
  myfile0 << "% sequence number \t % time stamp \n";
  myfile1 << "% sequence number \t % time stamp \t % latitude \n";
  myfile2 << "% sequence number \t % time stamp \t % longitude \n";
  myfile3 << "% sequence number \t % time stamp \t % linear_velocity \n";
  myfile4 << "% sequence number \t % time stamp \t % x \t % y \n";
  myfile5 << "% sequence number \t % time stamp \t % x \t % y \t % z \t % w - quaternion \n";
  
  // It is really important to write data with their full precision especially for the GPS measurements
  myfile0 << std::setprecision(17);
  myfile1 << std::setprecision(17);
  myfile2 << std::setprecision(17);
  myfile3 << std::setprecision(17);
  myfile4 << std::setprecision(17);
  myfile5 << std::setprecision(17);
  
  
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, img_topic, 1);
  message_filters::Subscriber<frame_extractor::GlobalPose> sub_gt(nh_, gt_topic, 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh_, odom_topic, 1);
  
  // message_filters::TimeSynchronizer<sensor_msgs::Image, frame_extractor::GlobalPose, nav_msgs::Odometry> sync(image_sub, sub_gt, sub_odom, 10);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, frame_extractor::GlobalPose, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, sub_gt, sub_odom);
  
  sync.registerCallback( boost::bind(&callback, _1, _2, _3) );
  
  // cv::namedWindow("Image window");
  
  ros::spin();
  
  // cv::destroyWindow("Image window");
  
  myfile0.close();
  myfile1.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();
  myfile5.close();
  
  return 0;
}

