//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>
  
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
  
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

ros::Publisher pub;

void deletetopblob(cv::Mat *src, cv::Mat *dest) {
	uchar *p, *q;
	for(int i=0; i < src->rows; i++) {
		p = src->ptr<uchar>(i);
		q = dest->ptr<uchar>(i);
		if (p[0] && p[src->cols-1]) {
			break; // stop at first non-black line
		}
		for(int j=0; j < src->cols; j++) {	// left to right
			if (!p[j]) {
				for (int k=0; k < dest->channels(); k++) {
					//ROS_INFO("%d\n", p[j+k]);
					q[j+k] = 255;
				}
			} else {
				break; // stop at first non-black pixel in column
			}
		}
		for(int j = src->cols-1; j>=0; j--) {   // right to left (shameless code-duplication)
			if (!p[j]) {
				for (int k=0; k < dest->channels(); k++) {
					//ROS_INFO("%d\n", p[j+k]);
					q[j+k] = 255;
				}
			} else {
				break;  // stop at first non-black pixel in column
			}
		}
	}
}
  
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}

	// Rotate 90 deg CCW
	cv::transpose(cv_ptr->image, cv_ptr->image);  
    	cv::flip(cv_ptr->image, cv_ptr->image, 0);

	int width = cv_ptr->image.size().width;
	int height = cv_ptr->image.size().height;

	// The code below is from: http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
	cv::Mat dst, cdst, wew, gray;

	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	cv::threshold(gray, wew, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	deletetopblob(&wew, &gray);
	
	cv::Canny(gray, dst, 50, 200, 3);
	cv::cvtColor(dst, cdst, CV_GRAY2BGR);

	
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(dst, lines, 1, CV_PI/180, 20, 20, 5 );
	long int sum = 0;
	float avg;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Vec4i l = lines[i];
		line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
		sum += ((l[0] + l[2]) - width) / 2;
	}
	if (lines.size() > 0) {
		avg = (sum / (float)lines.size()) / (float)(width / 2);
		//ROS_INFO("angle = %f\n", avg);
		geometry_msgs::Twist msg;
		msg.linear.x = 0.6;
		msg.angular.z = avg * 0.5;
		pub.publish(msg);
	}
	
	cv::imshow("source", cv_ptr->image);
	cv::imshow("detected lines", cdst);
	cv::imshow("wew", wew);
	cv::imshow("gray", gray);
  
    //Display the image using OpenCV
    //cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);
}
  
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
	ros::init(argc, argv, "image_processor");

	ros::NodeHandle nh;

	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    //Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	image_transport::TransportHints hints("compressed", ros::TransportHints());
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, hints);
    

//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);

    /**
    * In this application all user callbacks will be called from within the ros::spin() call. 
    * ros::spin() will not return until the node has been shutdown, either through a call 
    * to ros::shutdown() or a Ctrl-C.
    */
	ros::spin();
    //ROS_INFO is the replacement for printf/cout.
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
  
}
