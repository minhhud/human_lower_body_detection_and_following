#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>	
#include <pthread.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"

#include "message_filters/subscriber.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Leg Detector & Tracker
#include "detector/extraction_processor.h"
#include "detector/HOG_processor.h"
#include <leg_msgs/PositionMeasurement.h>
#include <leg_msgs/Legs_Position.h>
#include <leg_msgs/Legs_Distance.h>
#include "tracker/track_feature.h"
#include "tracker/tracker_kalman.h"

#define RGB_TOPIC "/camera/rgb/image_color"
#define PCL_TOPIC "/camera/depth/points"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudT_Ptr;
PointCloudT::Ptr cloud_ (new PointCloudT);

// Directory containing positive sample images
static std::string posSamplesDir = ros::package::getPath("leg_detector")+"/../database/leg_detection/train_data/pos/";
// Directory containing negative sample images
static std::string negSamplesDir = ros::package::getPath("leg_detector")+"/../database/leg_detection/train_data/neg/";
// Set the file to write the features to
static std::string featuresFile = ros::package::getPath("leg_detector")+"/../database/leg_detection/train_data/genfiles/features.dat";
// Set the file to write the SVM model to
static std::string svmModelFile = ros::package::getPath("leg_detector")+"/../database/leg_detection/train_data/genfiles/svmlightmodel.dat";
// Set the file to write the resulting detecting descriptor vector to
static std::string descriptorVectorFile = ros::package::getPath("leg_detector")+"/../database/leg_detection/train_data/genfiles/descriptorvector.dat";

candidates_retrieve::Retrieve candidates_ret;
extraction_process::Extraction ext_proc;

hog_detector::Detector hog_ld(svmModelFile);

std::list<LegsFeature*> tracked_features_;

int LegsFeature::nextid = 0;
ros::Publisher global_leg_result_pub, dist_leg_result_pub;

static pthread_t p_thread;
static bool thread_flag = true;
static int thread_id;

cv::Scalar Colors[] =
{
		cv::Scalar(100,0,0),
		cv::Scalar(0,100,0),
		cv::Scalar(0,0,100),
		cv::Scalar(155,0,100),
		cv::Scalar(155,100,0),
		cv::Scalar(0,100,155),
		cv::Scalar(100,0,155),
		cv::Scalar(0,155,100),
		cv::Scalar(100,155,0),
		cv::Scalar(100,155,100)
};

cv::Rect id_Rect[] =
{
		cv::Rect(0,510,50,90),
		cv::Rect(50,510,50,90),
		cv::Rect(100,510,50,90),
		cv::Rect(150,510,50,90),
		cv::Rect(200,510,50,90),
		cv::Rect(250,510,50,90),
		cv::Rect(300,510,50,90),
		cv::Rect(350,510,50,90),
		cv::Rect(400,510,50,90),
		cv::Rect(450,510,50,90)
};
