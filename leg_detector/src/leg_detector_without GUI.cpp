#include "leg_detector.h"
#include <sensor_msgs/RegionOfInterest.h>

cv::Mat path_img(cv::Size(500, 600), CV_8UC3, cv::Scalar::all(255));
cv::Mat pp_img[10];

struct TDI{
	bool flag;
	tf::Vector3 pos, leg1, leg2;
	int	*hist_val;
};

// actual legdetector node
class LegDetector
{
public:
	ros::NodeHandle 		nh_;
	tf::TransformListener 	tfl_;
	int 					feature_id_;
	int 					next_p_id_;
	bool 					rgb_flag_;
	bool 					pcl_flag_;
	cv::Mat 				rgb_img;
	double odom_pose[3];

	message_filters::Subscriber<sensor_msgs::Image> 		rgb_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> 	pcl_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> 	lrf_sub_;
	ros::Subscriber 	odom_sub_;
	tf::MessageFilter<sensor_msgs::Image>					rgb_notifier_;
	tf::MessageFilter<sensor_msgs::PointCloud2>				pcl_notifier_;
	tf::MessageFilter<sensor_msgs::LaserScan> 				lrf_notifier_;
	ros::Publisher vis_pub_;

	LegDetector(ros::NodeHandle nh) :
		nh_(nh),
		feature_id_(0),
		next_p_id_(0),
		rgb_sub_(nh_,RGB_TOPIC,1),
		pcl_sub_(nh_,PCL_TOPIC,1),
		lrf_sub_(nh_,"base_scan",1),
		rgb_notifier_(rgb_sub_,tfl_,"camera_rgb_frame",1),
		pcl_notifier_(pcl_sub_,tfl_,"camera_depth_frame",1),
		lrf_notifier_(lrf_sub_,tfl_,"camera_depth_frame",1),
		rgb_flag_(false),
		pcl_flag_(false)
	{
		rgb_img = cv::Mat::zeros(480, 640, CV_8UC3);

		rgb_notifier_.registerCallback(boost::bind(&LegDetector::RGB_Callback, this, _1));
		rgb_notifier_.setTolerance(ros::Duration(0.01));

		pcl_notifier_.registerCallback(boost::bind(&LegDetector::PCL_Callback, this, _1));
		pcl_notifier_.setTolerance(ros::Duration(0.01));

		lrf_notifier_.registerCallback(boost::bind(&LegDetector::LRF_Callback, this, _1));
		lrf_notifier_.setTolerance(ros::Duration(0.01));

		odom_sub_ = nh.subscribe("/odom", 1, &LegDetector::OD_Callback, this);
		vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	}

	~LegDetector()
	{
	}

	void OD_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
	{
		odom_pose[0] = pose_msg->pose.pose.position.y*0.1; // robot_x
		odom_pose[1] = pose_msg->pose.pose.position.x*0.1; // robot_z
		odom_pose[2] = atan2(pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w)*180/M_PI*2;
	}

	void RGB_Callback(const sensor_msgs::Image::ConstPtr& rgb_msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv_ptr->image.copyTo(rgb_img);
		if(!rgb_flag_) rgb_flag_ = true;
	}

	void PCL_Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
	{
		pcl::fromROSMsg(*cloud_msg, *cloud_);
		ext_proc.Set_Cloud(cloud_);

		if(!pcl_flag_) pcl_flag_ = true;
	}

	void LRF_Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	{
		if(rgb_flag_ && pcl_flag_)
		{
			cv::Mat viz_proc_img;
			rgb_img.copyTo(viz_proc_img);
			candidates_ret.Set_Scan(*scan_msg);

			std::list<SingleLegFeature*> new_single_features;
			std::list<LegsFeature*> propagated, new_legs_features;
			std::multiset<MatchedFeature> matches;

			// tracked_features_ -> pre_frame_track features
			// Filter_Update -> Track Erase(Time Over)
			// propagated -> updated_track
			propagated = Filter_Update(tracked_features_);

			// candidates_leg_retrieving in 2Dimensions
			candidates_ret.Classify_RT(viz_proc_img, new_single_features);

			// if find pairlegs, make id
			// new_single_features -> paired legs among candidates_legs
			ext_proc.New_PairLegs(new_single_features);

			// masking with Lower Body Part using Euclidean_

			// new_legs_features -> detected Lower body parts
			geometry_msgs::Point leg_joint[6];
			new_legs_features = ext_proc.Euclidean_Clustering(new_single_features, viz_proc_img, leg_joint, odom_pose);	

			int cnt=0;
			for(std::list<LegsFeature*>::iterator nlf_iter=new_legs_features.begin(); nlf_iter!=new_legs_features.end(); nlf_iter++,cnt++)
			{
				cv::Mat found_img = rgb_img(cv::Rect((*nlf_iter)->pixel_.x+10, (*nlf_iter)->pixel_.y, (*nlf_iter)->pixel_.width-20, (*nlf_iter)->pixel_.height));
				cv::resize(found_img, found_img,  cv::Size(64, 128));
				cv::HOGDescriptor hog;
				std::vector<float> descriptors;
				cv::cvtColor(found_img, found_img, CV_BGR2GRAY);
				hog.compute(found_img, descriptors, cv::Size(8, 8), cv::Size(0, 0));
				

				int pt1_x = (*nlf_iter)->pixel_.x > 14?(*nlf_iter)->pixel_.x:14;
				int pt1_y = (*nlf_iter)->pixel_.y > 14?(*nlf_iter)->pixel_.y:14;

				int pt2_x = ((*nlf_iter)->pixel_.x+(*nlf_iter)->pixel_.width) < 640?((*nlf_iter)->pixel_.x+(*nlf_iter)->pixel_.width):639;
				int pt2_y = ((*nlf_iter)->pixel_.y+(*nlf_iter)->pixel_.height) < 480?((*nlf_iter)->pixel_.y+(*nlf_iter)->pixel_.height):479;
				cv::rectangle(viz_proc_img, cv::Point(pt1_x, pt1_y), cv::Point(pt2_x, pt2_y), cv::Scalar(255,255,255), 2);

				cv::rectangle(viz_proc_img, cv::Point((*nlf_iter)->pixel_.x, (*nlf_iter)->pixel_.y), cv::Point((*nlf_iter)->pixel_.x+(*nlf_iter)->pixel_.width, (*nlf_iter)->pixel_.y+(*nlf_iter)->pixel_.height), cv::Scalar(255,255,255), 2);

				char id[10];
				sprintf(id, "%d", cnt);
//				cv::putText(viz_proc_img, id, cv::Point((*nlf_iter)->pixel_.x, (*nlf_iter)->pixel_.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
				int* color_val = Calculate_Color_Histogram((*nlf_iter)->pixel_, (*nlf_iter)->center_.getZ(), rgb_img);
				float color_diff = 0;
				int* closest_color;
				std::list<LegsFeature*>::iterator closest = propagated.end();
				float closest_dist = ext_proc.max_track_jump_m_;

				// associate with pre_track_features
				float similarity = 0;

				for (std::list<LegsFeature*>::iterator pf_iter = propagated.begin(); pf_iter != propagated.end(); pf_iter++)
				{
					// find the closest distance between candidate and trackers
					float dist = (*nlf_iter)->center_.distance((*pf_iter)->center_);

					// calculate similarity about color
					double color = 0;
					for( int i = 0; i < 16; i++ )
					{
						if(abs(color_val[i] - (*pf_iter)->hist_[i])<10)	;
						else
							color += ((color_val[i] - (*pf_iter)->hist_[i]) * (color_val[i] - (*pf_iter)->hist_[i]));
					}
					color = sqrt(color)/10;

					color = std::max(8, (int)color);
					color = std::min(30, (int)color);
					float sim_ = calculate_similarity(dist, (int)color, (*pf_iter)->undetected_time_);

					if((sim_ > 0.7) && (sim_ > similarity))
					{
						similarity = sim_;
						color_diff = color;
						closest_color = color_val;
						closest = pf_iter;
						closest_dist = dist;
					}
				}
				// if there is similar track, then update it
				if (closest != propagated.end())
				{
					(*nlf_iter)->hist_ = closest_color;
					(*nlf_iter)->undetected_time_ = 0;
					(*closest)->update((*nlf_iter));
					propagated.erase(closest);
				}
				// else find new leg using HOG+SVM
				else
				{
					cv::Mat img;
					cv::Mat img_roi = viz_proc_img((*nlf_iter)->pixel_);
					img_roi.copyTo(img);

					if(hog_ld.Classify_HOG_SVM(img, viz_proc_img, (*nlf_iter)->pixel_))
					{
						int pt1_x = (*nlf_iter)->pixel_.x > 14?(*nlf_iter)->pixel_.x:14;
						int pt1_y = (*nlf_iter)->pixel_.y > 14?(*nlf_iter)->pixel_.y:14;

						int pt2_x = ((*nlf_iter)->pixel_.x+(*nlf_iter)->pixel_.width) < 640?((*nlf_iter)->pixel_.x+(*nlf_iter)->pixel_.width):639;
						int pt2_y = ((*nlf_iter)->pixel_.y+(*nlf_iter)->pixel_.height) < 480?((*nlf_iter)->pixel_.y+(*nlf_iter)->pixel_.height):479;
						cv::rectangle(viz_proc_img, cv::Point(pt1_x, pt1_y), cv::Point(pt2_x, pt2_y), cv::Scalar(0,0,255), 2);

						(*nlf_iter)->hist_ = color_val;
						std::list<LegsFeature*>::iterator new_saved = tracked_features_.insert(tracked_features_.end(), (*nlf_iter));
					}
					img.release();
					img_roi.release();
				}
			}

			for(std::list<LegsFeature*>::iterator remain = propagated.begin(); remain != propagated.end(); remain++)
				(*remain)->undetected_time_ = std::min(100, ++(*remain)->undetected_time_);

			Update_ID();

			cv::imshow("Processing", viz_proc_img);
			cv::waitKey(1);
			viz_proc_img.release();
		}

		leg_msgs::Legs_Position global_legs_pos;
		leg_msgs::Legs_Distance dist_robot_leg;
		ros::Time stamp_t;
		for(std::list<LegsFeature*>::iterator tf_iter = tracked_features_.begin(); tf_iter != tracked_features_.end(); tf_iter++)
		{
			leg_msgs::PositionMeasurement global_pos_meas;
			global_pos_meas.object_id = (*tf_iter)->object_id;

			geometry_msgs::Point local_pos, global_pos;
			global_pos.x = (*tf_iter)->leg1_->loc_.getX();	global_pos.y = (*tf_iter)->leg1_->loc_.getY();	global_pos.z = (*tf_iter)->leg1_->loc_.getZ();
			global_pos_meas.leg1 = global_pos;
			global_pos.x = (*tf_iter)->leg2_->loc_.getX();	global_pos.y = (*tf_iter)->leg2_->loc_.getY();	global_pos.z = (*tf_iter)->leg2_->loc_.getZ();
			global_pos_meas.leg2 = global_pos;
			global_pos.x = (*tf_iter)->center_.getX();	global_pos.y = (*tf_iter)->center_.getY();	global_pos.z = (*tf_iter)->center_.getZ();
			global_pos_meas.cen = global_pos;
			(*tf_iter)->time_;
			global_pos_meas.dir = (*tf_iter)->direction_;
			sensor_msgs::RegionOfInterest roi_;
			roi_.width = (*tf_iter)->pixel_.width;
			roi_.height = (*tf_iter)->pixel_.height;
			roi_.x_offset = (*tf_iter)->pixel_.x;
			roi_.y_offset = (*tf_iter)->pixel_.y;
			global_pos_meas.leg_roi = roi_;

			global_legs_pos.legs.push_back(global_pos_meas);
			float d_x = (odom_pose[0] - global_pos.x);
			float d_z = (odom_pose[1] - global_pos.z);

			float dist = sqrt((d_x * d_x) + (d_z * d_z));
			
			dist_robot_leg.dist.push_back(dist);
			dist_robot_leg.dir.push_back((*tf_iter)->direction_);

			dist_robot_leg.leg_roi.push_back(roi_);

			stamp_t = std::max(stamp_t, (*tf_iter)->time_);
		}
		global_legs_pos.header.stamp = stamp_t;
		global_legs_pos.header.frame_id = "leg_detector";
		global_leg_result_pub.publish(global_legs_pos);

		dist_robot_leg.header.stamp = stamp_t;
		dist_robot_leg.header.frame_id = "leg_detector";

		dist_leg_result_pub.publish(dist_robot_leg);
	}

	std::list<LegsFeature*> Filter_Update(std::list<LegsFeature*>& features)
	{
	    ros::Time purge = candidates_ret.scan_.header.stamp + ros::Duration().fromSec(-ext_proc.no_observation_timeout_s_);
	    std::list<LegsFeature*>::iterator sf_iter = features.begin();
	    while (sf_iter != features.end())
	    {
			if ((*sf_iter)->meas_time_ < purge)
			{
				if( (*sf_iter)->other )
					(*sf_iter)->other->other = NULL;
				delete (*sf_iter);
				features.erase(sf_iter++);
			}
			else
				++sf_iter;
	    }
	    // System update of trackers, and copy updated ones in propagate list
		std::list<LegsFeature*> propagated;
		for (std::list<LegsFeature*>::iterator sf_iter = features.begin(); sf_iter != features.end(); sf_iter++)
		{
			(*sf_iter)->propagate(candidates_ret.scan_.header.stamp);
			propagated.push_back(*sf_iter);

			cv::Mat histimg  = cv::Mat::zeros(100, 320, CV_8UC3);
			histimg = cv::Scalar::all(0);
			int binW = histimg.cols / 16;
			cv::Mat buf(1, 16, CV_8UC3);
			for( int i = 0; i < 16; i++ )
				buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./16), 255, 255);
			cvtColor(buf, buf, CV_HSV2BGR);

			char val_text[10];

			for( int i = 0; i < 16; i++ )
			{
				int sat_val = cv::saturate_cast<int>((*sf_iter)->hist_[i]*histimg.rows/255);
		//		cv::rectangle( histimg, cv::Point(i*binW,histimg.rows),
		//				cv::Point((i+1)*binW,histimg.rows - sat_val),
		//				cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
			}
			char hist_text[10];
			sprintf(hist_text, "%s Histogram", (*sf_iter)->object_id.c_str());
//			cv::imshow( hist_text, histimg );
		}
		return propagated;
	}

	void Update_ID()
	{
		// Deal With legs that already have ids
		std::list<LegsFeature*>::iterator begin = tracked_features_.begin();
		std::list<LegsFeature*>::iterator end = tracked_features_.end();
		std::list<LegsFeature*>::iterator leg_iter, tracked_leg, best, it;

		int id_list[10] = {0,1,2,3,4,5,6,7,8,9};

	    for (leg_iter = begin; leg_iter != end; ++leg_iter)
	    {
	    	if((*leg_iter)->object_id != "")
	    	{
	    		id_list[atoi((*leg_iter)->object_id.data())] = 10;
	    	}
	    }

	    for (leg_iter = begin; leg_iter != end; ++leg_iter)
	    {
	    	if((*leg_iter)->object_id == "")
			{
	    		for(int i=0; i<10; i++)
	    		{
	    			if(id_list[i]!=10)
	    			{
	    				char id[100];
						snprintf(id,10,"%d",id_list[i]);
	    				(*leg_iter)->object_id = std::string(id);
	    				id_list[i] = 10;
	    				cv::Mat tmp_img = rgb_img((*leg_iter)->pixel_);
						cv::resize(tmp_img, tmp_img,cv::Size(50, 90));
						tmp_img.copyTo(pp_img[i]);
						tmp_img.release();
	    				break;
	    			}
	    		}
			}
	    }
	}

	int* Calculate_Color_Histogram(cv::Rect rect, double depth, cv::Mat src)
	{
		cv::Mat mask, hue, hsv_img, hist;
		cv::Mat dst = cv::Mat::zeros(480, 640, CV_8UC3);;

		int ch[] = {0, 0};
		int hsize = 16;
		float hranges[] = {0,180};
		const float* phranges = hranges;

		int w1 = rect.x;
		int h1 = rect.y;
		int w2 = rect.x + rect.width;
		int h2 = rect.y + rect.height;

		cv::Mat img;
		cv::Mat img_roi = src(rect);
		img_roi.copyTo(img);

		cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

		cv::inRange(hsv_img, cv::Scalar(0, 10, 10),
		cv::Scalar(180, 255, 255), mask);

		hue.create(hsv_img.size(), hsv_img.depth());
		cv::mixChannels(&hsv_img, 1, &hue, 1, ch, 1);

		cv::calcHist(&hue, 1, 0, mask, hist, 1, &hsize, &phranges);
		cv::normalize(hist, hist, 0, 255, CV_MINMAX);

		int* hist_val = (int*)calloc(16, sizeof(int));
		for( int i = 0; i < 16; i++ )
		{
			int val = cv::saturate_cast<int>(hist.at<float>(i));
			hist_val[i] = val;
		}
		return hist_val;
	}

	float calculate_similarity(float dist, int color, int lost_time)
	{
		float sim_dist = 1-(dist*dist);

		float sim_color = (-1.0/22.0)*(float)color + (15.0/11.0);

		float w_d = 0.3 + 0.4*exp(-(lost_time*lost_time)/(40*40));
		float w_c = 1-w_d;

		float similarity = w_d * sim_dist + w_c * sim_color;

		//return similarity; // dist+color
		return sim_dist;     // only dist
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv,"leg_detector");
	ros::NodeHandle nh;
	global_leg_result_pub = nh.advertise<leg_msgs::Legs_Position>("/global_leg_detection", 1);
	dist_leg_result_pub = nh.advertise<leg_msgs::Legs_Distance>("/leg_detection", 1);
	cv::namedWindow("Processing GUI", 0);
	cvResizeWindow( "Processing GUI", 1090, 960 );

	if(!strcmp(argv[1],"train"))
		hog_detector::Trainer hog_train(featuresFile, svmModelFile, posSamplesDir, negSamplesDir);
	else
	{
		candidates_ret.Init_Conf();
		candidates_ret.Load_RT(argc, argv);

		ext_proc.Init_Conf();

		hog_ld.Set_SVM();

		LegDetector ld(nh);

		thread_flag = false;
		ros::spin();
	}
	return 0;
}

