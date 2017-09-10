#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"

#include <image_transport/image_transport.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
//#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\core\core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
//#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel cam_model_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_cam_frame(new pcl::PointCloud<pcl::PointXYZ>);
double DEPTH_RANGE = 40; // FIXME:: 40 meters range for pointcloud

//tf::TransformListener* p_listener;
tf::StampedTransform trans_lidToCam;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{

    pcl::fromROSMsg(*cloud_msg, *cloud);   	

	try{
	    pcl_ros::transformPointCloud(*cloud,*cloud_in_cam_frame,trans_lidToCam);
	   }
	catch(...){
    	    ROS_INFO_STREAM("Pointcloud transformation failed");
	   }	  

    //ROS_INFO_STREAM(" Width = "<<cloud_msg->width<<" Height = "<<cloud_msg->height);    
    
}


void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
{

	ROS_INFO_STREAM("Cloud Data Size =" <<cloud_in_cam_frame->points.size());

	cam_model_.fromCameraInfo(info_msg);
	
	int height = image_msg->height;
	int width = image_msg->width;

	cv::Mat depth_image; // cv::Mat depth_image(height,width,CV_8UC1);
	depth_image = cv::Mat::zeros(height,width,CV_8UC1);
	
	
	if (cloud_in_cam_frame->points.size() > 0)
	{ 	

	  for(size_t i = 0; i < cloud_in_cam_frame->points.size(); i+=100)//cloud->points.size(); i++)
	  {
		if(cloud_in_cam_frame->points[i].z>=0)
		{
		cv::Point3d pt_cv(cloud_in_cam_frame->points[i].x,cloud_in_cam_frame->points[i].y, cloud_in_cam_frame->points[i].z);
		cv::Point2d uv;
		uv = cam_model_.project3dToPixel(pt_cv);

		if((uv.x>=0) && (uv.x<width) && (uv.y>=0) && (uv.y<height)) // if pixel is within camera frame size
		{
		//depth_image[int(uv.y)][int(uv.x)] = cloud->points[i].z;
		double depth = 255*cloud_in_cam_frame->points[i].z/DEPTH_RANGE;
		//ROS_INFO_STREAM_THROTTLE(1,"depth = "<<depth);
		//ROS_INFO_STREAM("depth = "<<depth);	
		try{	
			depth_image.at<double>(int(uv.y),int(uv.x)) = (depth<255)?depth:255;
		}
		catch(...){
		ROS_INFO_STREAM("Erroneous projection onto image");
		return;
		}
		}

		ROS_INFO_STREAM("Projection of x ="<< cloud_in_cam_frame->points[i].x<<" y = "<< cloud_in_cam_frame->points[i].y<<" z = "<< cloud_in_cam_frame->points[i].z<<" is at  u = "<<uv.x<<" v = "<<uv.y);
		}
		//break;		
	  }
	cv::namedWindow( "Depth Image", CV_WINDOW_AUTOSIZE );// Create a window for display.
    	cv::imshow( "Depth Image", depth_image );                   // Show our image inside it.
	cv::waitKey(10);                                          // Wait for a keystroke in the window
	}
	
}

int main(int argc, char* argv[])
{
	ros::init (argc, argv, "project3dtouv");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it_(nh);	
    	
	tf::TransformListener listener;


	// Get the transform between LIDAR and camera
	while (nh.ok()){
		try{
	      	    listener.lookupTransform("bumblebee_optical","velodyne", ros::Time(0), trans_lidToCam);
		    //p_listener = &listener;
		   }
		catch (tf::TransformException ex){
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
		    continue;
		   }		

	//ROS_INFO_STREAM("Transform"<<transform.getOrigin().x());
	
	// Get point cloud data (x,y,z)	
	ros::Subscriber subCloud = nh.subscribe("velodyne_points", 100, cloud_cb);

	// Project the point cloud data to pixel position using camera model (Projection is currently for a random single point)
	image_transport::CameraSubscriber subCam = it_.subscribeCamera("camera/image_raw", 100,imageCb);

	// Display a range image   
	// in progress ....
    

	ros::spin();
	}
}





