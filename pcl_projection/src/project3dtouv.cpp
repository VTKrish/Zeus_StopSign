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

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <tf/transform_listener.h>
//#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel cam_model_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{

    pcl::fromROSMsg(*cloud_msg, *cloud);

    //ROS_INFO_STREAM(" Width = "<<cloud_msg->width<<" Height = "<<cloud_msg->height);    
    
}


void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
{

	ROS_INFO_STREAM("Cloud Data Size =" <<cloud->points.size());

	cam_model_.fromCameraInfo(info_msg);
	
	
	if (cloud->points.size() > 0)
	{ 	

	  for(size_t i = 0; i < cloud->points.size(); i++)
	  {
		if(cloud->points[i].z!=0)
		{
		cv::Point3d pt_cv(cloud->points[i].x,cloud->points[i].y, cloud->points[i].z);
		cv::Point2d uv;
		uv = cam_model_.project3dToPixel(pt_cv);

		ROS_INFO_STREAM("Projection of x ="<< cloud->points[i].x<<" y = "<< cloud->points[i].y<<" z = "<< cloud->points[i].z<<" is at  u = "<<uv.x<<" v = "<<uv.y);
		}
		break;
	  }
	}
	
}

int main(int argc, char* argv[])
{
	ros::init (argc, argv, "project3dtouv");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it_(nh);		
	
	// Get point cloud data (x,y,z)	
	ros::Subscriber subCloud = nh.subscribe("velodyne_points", 100, cloud_cb);

	// Project the point cloud data to pixel position using camera model (Projection is currently for a random single point)
	image_transport::CameraSubscriber subCam = it_.subscribeCamera("camera/image_raw", 100,imageCb);

	// Display a range image   
	// in progress ....
    

	ros::spin();
}





