#ifndef DETECT_LASER_HH
#define DETECT_LASER_HH

#include <leg_detector/LegDetectorConfig.h>
#include <dynamic_reconfigure/server.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "tracker/track_feature.h"

/////////////레이저 센서를 이용한 1차 다리 검출////////////////////
namespace candidates_retrieve
{
	/////////////다리 후보군 검출 클래스////////////////////
	class Retrieve
	{
	public:
		Retrieve();
		~Retrieve();

		void Get_Configure(leg_detector::LegDetectorConfig &config, uint32_t level);
		void Init_Conf();

		/////////////Random Tree 학습 데이터 Load////////////////////
		void Load_RT(int argc_, char **argv_);
		/////////////Scan Data 저장////////////////////
		void Set_Scan(const sensor_msgs::LaserScan& scan);
		/////////////1차 다리 후보군 검출//////////////////
		void Classify_RT(cv::Mat& viz_img, std::list<SingleLegFeature*>& new_features);

	public:
		/////////////다리의 임계값 설정////////////////////
		float connected_thresh_;
		sensor_msgs::LaserScan scan_;

		int mask_count_;
		CvRTrees forest_;
		int feat_count_;
	};
};

namespace extraction_process
{
	/////////////다리 후보군중 다리 추출 클래스////////////////////
	class Extraction
	{
	public:
		Extraction();
		~Extraction();

		typedef pcl::PointXYZ PointT;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef PointCloudT::Ptr PointCloudT_Ptr;

		void Get_Configure(leg_detector::LegDetectorConfig &config, uint32_t level);
		void Init_Conf();

		/////////////PCL Data 저장////////////////////
		void Set_Cloud(PointCloudT_Ptr& cloud);
		/////////////두 다리사이의 거리 계산////////////////////
		double Distance( std::list<SingleLegFeature*>::iterator it1, std::list<SingleLegFeature*>::iterator it2);
		/////////////Random Tree를 이용한 다리 분류////////////////////
		void New_PairLegs(std::list<SingleLegFeature*>& features);
		/////////////Euclidean Distance를 이용하여 후처리////////////////////
		std::list<LegsFeature*> Euclidean_Clustering(std::list<SingleLegFeature*> features, cv::Mat& viz_img, geometry_msgs::Point* leg_joint, double* odom_pose);
		std::string Calc_Direction(cv::Mat& viz_img, tf::Point leg1, tf::Point leg2, geometry_msgs::Point* leg_joint);

	public:
		double no_observation_timeout_s_;
		/////////////두 다리 사이의 최소 간격 설정////////////////////
		float leg_pair_separation_;
		float max_track_jump_m_;

		PointCloudT::Ptr cloud_ ;
		int next_p_id_;
	};
};

#endif
