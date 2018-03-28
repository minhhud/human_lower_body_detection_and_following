#include "detector/extraction_processor.h"
#include "detector/laser_processor.h"
#include "detector/calc_leg_features.h"

using namespace candidates_retrieve;
using namespace extraction_process;
using namespace laser_processor;

Retrieve::Retrieve()
{
}

Retrieve::~Retrieve()
{
}

void Retrieve::Init_Conf()
{
	dynamic_reconfigure::Server<leg_detector::LegDetectorConfig> srv_;
	dynamic_reconfigure::Server<leg_detector::LegDetectorConfig>::CallbackType f;
	f = boost::bind(&Retrieve::Get_Configure, this, _1, _2);
	srv_.setCallback(f);
}

void Retrieve::Get_Configure(leg_detector::LegDetectorConfig &config, uint32_t level)
{
	connected_thresh_       	= config.connection_threshold;
}

void Retrieve::Load_RT(int argc_, char **argv_)
{
	if (argc_ > 1)
	{
		forest_.load(argv_[1]);
		feat_count_ = forest_.get_active_var_mask()->cols;
		printf("Loaded forest_ with %d features: %s\n", feat_count_, argv_[1]);
	}
	else
	{
		printf("Please provide a trained random forest_s classifier as an input.\n");
		ros::shutdown();
	}
}

void Retrieve::Set_Scan(const sensor_msgs::LaserScan& scan)
{
	scan_ = scan;
}

void Retrieve::Classify_RT(cv::Mat& viz_img, std::list<SingleLegFeature*>& new_features)
{
	std::multiset<MatchedFeature> matches;
	ScanMask mask_;
	ScanProcessor processor(scan_, mask_);
	processor.splitConnected(connected_thresh_);
	processor.removeLessThan(5);

	CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

	std::list<tf::Point> leg_tmp;

	/////////////Scan Data로 Cluster된 Data를 Random Tree를 이용하여 개별 다리 분류////////////////////
	for (std::list<SampleSet*>::iterator i = processor.getClusters().begin(); i != processor.getClusters().end(); i++)
	{
		std::vector<float> f = calcLegFeatures(*i, scan_);

		for (int k = 0; k < feat_count_; k++)
			tmp_mat->data.fl[k] = (float)(f[k]);

		float probability = forest_.predict_prob( tmp_mat );
		tf::Stamped<tf::Point> loc((*i)->center(), scan_.header.stamp, scan_.header.frame_id);
		/////////////다리일 확율이 90%이상인 값에 대해서만 중심값 저장////////////////////
		if(probability>0.9)
		{
			std::list<SingleLegFeature*>::iterator new_saved = new_features.insert(new_features.end(), new SingleLegFeature(loc, probability));

	        int n = int(-((*i)->center().getY()/(*i)->center().getX())*525 + 320)-10;
	        //cv::rectangle(viz_img, cv::Point(n, 340), cv::Point(n, 343), cv::Scalar(64, 255, 64), 2);
	        char tmp[50];
	        sprintf(tmp, "x:%.3f", (*i)->center().getY());
//	        cv::putText(viz_img, tmp, cv::Point(n,310), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,0),1,1);
	        sprintf(tmp, "z:%.3f", (*i)->center().getX());
//	        cv::putText(viz_img, tmp, cv::Point(n,325), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,0),1,1);
		}
	}
	cvReleaseMat(&tmp_mat); tmp_mat = 0;
}

Extraction::Extraction()
{
}

Extraction::~Extraction()
{
}

void Extraction::Init_Conf()
{
	dynamic_reconfigure::Server<leg_detector::LegDetectorConfig> srv_;
	dynamic_reconfigure::Server<leg_detector::LegDetectorConfig>::CallbackType f;
	f = boost::bind(&Extraction::Get_Configure, this, _1, _2);
	srv_.setCallback(f);
}

void Extraction::Get_Configure(leg_detector::LegDetectorConfig &config, uint32_t level)
{
	no_observation_timeout_s_	= config.no_observation_timeout;
	leg_pair_separation_    	= config.leg_pair_separation;
	max_track_jump_m_			= config.max_track_jump;
}

void Extraction::Set_Cloud(PointCloudT_Ptr& cloud)
{
	cloud_ = cloud;
}

double Extraction::Distance( std::list<SingleLegFeature*>::iterator it1, std::list<SingleLegFeature*>::iterator it2)
{
    tf::Stamped<tf::Point> one = (*it1)->loc_, two = (*it2)->loc_;
    double dx = one[0]-two[0], dy = one[1]-two[1], dz = one[2]-two[2];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

void Extraction::New_PairLegs(std::list<SingleLegFeature*>& features)
{
	int next_id=0;
	// Deal With legs that already have ids
	std::list<SingleLegFeature*>::iterator begin = features.begin();
	std::list<SingleLegFeature*>::iterator end = features.end();
	std::list<SingleLegFeature*>::iterator leg1, leg2, best, it;

	// Attempt to pair up legs with no id
	for(;;)
	{
		std::list<SingleLegFeature*>::iterator best1 = end, best2 = end;
		double closest_dist = leg_pair_separation_;

		for(leg1 = begin; leg1 != end; ++leg1)
		{
		// If this leg has low Probability, skip
			if (((*leg1)->object_id_ != "")||(*leg1)->getProbability() < 0.9)
				continue;

			for(leg2 = begin; leg2 != end; ++leg2)
			{
				if(((*leg2)->object_id_ != "")||((*leg2)->getProbability() < 0.9) || (leg1==leg2) ) continue;
				double d = Distance(leg1, leg2);
				if(d<closest_dist)
				{
					best1 = leg1;
					best2 = leg2;
					closest_dist = d;
				}
			}
		}
		if(best1 != end)
		{
			char id[100];
			snprintf(id,100,"tmp%d", next_id++);
			(*best1)->object_id_ = std::string(id);
			(*best2)->object_id_ = std::string(id);
			(*best1)->other_ = *best2;
			(*best2)->other_ = *best1;
		}
		else
		{
			break;
		}
	}
}

std::list<LegsFeature*> Extraction::Euclidean_Clustering(std::list<SingleLegFeature*> features, cv::Mat& viz_img, geometry_msgs::Point* leg_joint, double* odom_pose)
{
	std::list<LegsFeature*> legs_features;
	/////////////Laser Scan을 통해 분류된 다리 후보군을 대상으로 Euclidean Distance를 이용하여 Clustering////////////////////
	if(features.size() > 1)
	{
	    std::list<SingleLegFeature*>::iterator begin = features.begin();
	    std::list<SingleLegFeature*>::iterator end = features.end();
	    std::list<SingleLegFeature*>::iterator leg1, leg2;

		for(leg1 = begin; leg1 != end; ++leg1)
		{
			if ((*leg1)->object_id_ == "")
				continue;
		    for(leg2 = leg1; leg2 != end; ++leg2)
		    {
		    	if((leg1!=leg2)&&((*leg1)->object_id_ == (*leg2)->object_id_))
		    	{
					/////////////다리 좌, 우에 대한 Pixel값 추출////////////////////
					int leg1_pixel = int(-((*leg1)->loc_.getX()/(*leg1)->loc_.getZ())*525 + 320)-10;
					int leg2_pixel = int(-((*leg2)->loc_.getX()/(*leg2)->loc_.getZ())*525 + 320)-10;

//					cv::rectangle(viz_img, cv::Point(leg1_pixel-10, 330), cv::Point(leg2_pixel-10, 350), cv::Scalar(255, 255, 255), 2);

					/////////////Depth값을 기반으로 5cm 폭 계산////////////////////
					int inter = int((0.05/(*leg1)->loc_.getZ())*525);

					int pre_w1 = leg1_pixel;
					int pre_w2 = leg2_pixel;

					int now_w1 = pre_w1;
					int now_w2 = pre_w2;

					double pre_leg1_x = -(*leg1)->loc_.getX();
					double pre_leg1_z = (*leg1)->loc_.getZ();
					double pre_leg2_x = -(*leg2)->loc_.getX();
					double pre_leg2_z = (*leg2)->loc_.getZ();

					int cen_w1 = now_w1;
					int cen_w2 = now_w2;

					int max_w1 = (cen_w1+inter) < 640?(cen_w1+inter):639;
					int min_w1 = (cen_w1-inter) > 14?(cen_w1-inter):14;

					int max_w2 = (cen_w2+inter) < 640?(cen_w2+inter):639;
					int min_w2 = (cen_w2-inter) > 14?(cen_w2-inter):14;

					int b_left = std::min(min_w1, min_w2);
					int b_right = std::max(max_w1, max_w2);

					for(int i=(340-inter); i>0; i-=inter)
					{
						cen_w1 = now_w1;
						cen_w2 = now_w2;

						max_w1 = (cen_w1+inter) < 640?(cen_w1+inter):639;
						min_w1 = (cen_w1-inter) > 14?(cen_w1-inter):14;

						max_w2 = (cen_w2+inter) < 640?(cen_w2+inter):639;
						min_w2 = (cen_w2-inter) > 14?(cen_w2-inter):14;

						double distance = 5;
						pre_w1 = now_w1;
						for(int w=min_w1; w<max_w1; w++)
						{
							double dx = cloud_->points[w+640*i].x - pre_leg1_x;
							double dz = cloud_->points[w+640*i].z - pre_leg1_z;

							/////////////거리값이 5센티 미만이면 Cluster////////////////////
							if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
							{
								distance = std::min(distance, sqrt(dx*dx + dz*dz));
								now_w1 = w;
							}
						}
						//cv::rectangle(viz_img, cv::Point(now_w1-2, i-2), cv::Point(now_w1+2, i+2), cv::Scalar(0, 255, 255), 2);
						pre_leg1_x =  cloud_->points[now_w1+640*i].x;
						pre_leg1_z =  cloud_->points[now_w1+640*i].z;

						if(distance == 5)
							break;

						distance = 5;
						pre_w2 = now_w2;
						for(int w=min_w2; w<max_w2; w++)
						{
							double dx = cloud_->points[w+640*i].x - pre_leg2_x;
							double dz = cloud_->points[w+640*i].z - pre_leg2_z;

							if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
							{
								distance = std::min(distance, sqrt(dx*dx + dz*dz));
								now_w2 = w;
							}
						}
						//cv::rectangle(viz_img, cv::Point(now_w2-2, i-2), cv::Point(now_w2+2, i+2), cv::Scalar(255, 0, 255), 2);
						pre_leg2_x =  cloud_->points[now_w2+640*i].x;
						pre_leg2_z =  cloud_->points[now_w2+640*i].z;

						if(distance == 5)
							break;

						/////////////Euclidean Distance를 이용하여 Cluster한 높이가 70cm이상인 경우////////////////////
						if((-cloud_->points[now_w1+640*i].y+0.6>0.7)&&(-cloud_->points[now_w2+640*i].y+0.6>0.7))
						{
							bool connect_pair=true;
							int pre_w = std::min(now_w1, now_w2);
							/////////////최대 높이에서 두 다리 사이 값이 모두 동일한 깊이값을 가지는지 확인(두 다리가 이어져 있는지를 확인하기 위함)////////////////////
							for(int w=std::min(now_w1, now_w2)+1; w<std::max(now_w1, now_w2); w++)
							{
								double dz_cw = cloud_->points[w+640*i].z - cloud_->points[pre_w+640*i].z;

								cv::rectangle(viz_img, cv::Point(w-12, i-2), cv::Point(w-8, i+2), cv::Scalar(0, 0, 255), 2);
								if(dz_cw > 0.05 || isnan(dz_cw))
								{
									connect_pair = false;
									break;
								}
								pre_w = w;
							}
							/////////////최종적으로 다리로 확정////////////////////
							if(connect_pair)
							{
								std::string dir = Calc_Direction(viz_img, tf::Point(pre_leg1_x, -cloud_->points[now_w1+640*i].y-0.2, pre_leg1_z), tf::Point(pre_leg2_x, -cloud_->points[now_w2+640*i].y-0.2, pre_leg2_z), leg_joint);

								int u_left = std::min(min_w1, min_w2);
								int u_right = std::max(max_w1, max_w2);
								
								double global1[2], global2[2];
								//std::cout << "local_1" << (*leg1)->loc_.getX() << ", " << (*leg1)->loc_.getZ() << std::endl;
								double local_ori = atan2((*leg1)->loc_.getX(), (*leg1)->loc_.getZ()) * 180/ M_PI;

								global1[0] = odom_pose[0] + sqrt(((*leg1)->loc_.getX()*(*leg1)->loc_.getX()) + ((*leg1)->loc_.getZ()*(*leg1)->loc_.getZ())) * sin((odom_pose[2] + local_ori) * M_PI/180);
								global1[1] = odom_pose[1] + sqrt(((*leg1)->loc_.getX()*(*leg1)->loc_.getX()) + ((*leg1)->loc_.getZ()*(*leg1)->loc_.getZ())) * cos((odom_pose[2] + local_ori) * M_PI/180);
		
								//global_x = (*leg1)->loc_.getZ() * cos(odom_pose[2]+ 
								tf::Stamped<tf::Point> loc1(tf::Point(global1[0], -cloud_->points[now_w1+640*i].y, global1[1]), (*leg1)->loc_.stamp_, (*leg1)->loc_.frame_id_);
								(*leg1)->loc_ = loc1;
								(*leg1)->pixel_ = cv::Rect((leg1_pixel+(now_w1 - leg1_pixel)/2)-((max_w1-min_w1)/4), (i+((340-inter)-i)/2)-((max_w1-min_w1)/4), (max_w1-min_w1)/2, (max_w1-min_w1)/2);
//								cv::rectangle(viz_img, cv::Point((leg1_pixel+(now_w1 - leg1_pixel)/2)-((max_w1-min_w1)/4), (i+((340-inter)-i)/2)-((max_w1-min_w1)/4)), cv::Point((leg1_pixel+(now_w1 - leg1_pixel)/2)+((max_w1-min_w1)/4), (i+((340-inter)-i)/2)+((max_w1-min_w1)/4)), cv::Scalar(255, 0, 255), 3);

								local_ori = atan2((*leg2)->loc_.getX(), (*leg2)->loc_.getZ()) * 180/ M_PI;
								global2[0] = odom_pose[0] + sqrt(((*leg2)->loc_.getX()*(*leg2)->loc_.getX()) + ((*leg2)->loc_.getZ()*(*leg2)->loc_.getZ())) * sin((odom_pose[2] + local_ori) * M_PI/180);
								global2[1] = odom_pose[1] + sqrt(((*leg2)->loc_.getX()*(*leg2)->loc_.getX()) + ((*leg2)->loc_.getZ()*(*leg2)->loc_.getZ())) * cos((odom_pose[2] + local_ori) * M_PI/180);

								//std::cout << "local_2" << (*leg2)->loc_.getX() << ", " << (*leg2)->loc_.getZ() << std::endl;
								tf::Stamped<tf::Point> loc2(tf::Point(global2[0], -cloud_->points[now_w2+640*i].y, global2[1]), (*leg2)->loc_.stamp_, (*leg2)->loc_.frame_id_);
								(*leg2)->loc_ = loc2;
								(*leg2)->pixel_ = cv::Rect((leg2_pixel+(now_w2 - leg2_pixel)/2)-((max_w2-min_w2)/4), (i+((340-inter)-i)/2)-((max_w2-min_w2)/4), (max_w2-min_w2)/2, (max_w2-min_w2)/2);
//								cv::rectangle(viz_img, cv::Point((leg2_pixel+(now_w2 - leg2_pixel)/2)-((max_w2-min_w2)/4), (i+((340-inter)-i)/2)-((max_w2-min_w2)/4)), cv::Point((leg2_pixel+(now_w2 - leg2_pixel)/2)+((max_w2-min_w2)/4), (i+((340-inter)-i)/2)+((max_w2-min_w2)/4)), cv::Scalar(255, 0, 255), 3);

								//std::cout << "global1" << global1[0] << ", " << global1[1] << ", " << odom_pose[2] << std::endl;
								//std::cout << "global2" << global2[0] << ", " << global2[1] << ", " << odom_pose[2] << std::endl;
								//std::cout << "odom" << odom_pose[0] << ", " << odom_pose[1] << ", " << odom_pose[2] << std::endl;
								//std::cout << "--------------------------------------------------------------------" << std::endl;	

								int tmp_height = (340+(inter*4)) < 479?(340+(inter*4)):479;
								cv::Rect tmp_rect(std::min(b_left, u_left)-10, i, std::max(b_right, u_right)-std::min(b_left, u_left)+20, tmp_height-i);
								legs_features.insert(legs_features.end(),new LegsFeature((*leg1), (*leg2), tmp_rect, dir, (int*)calloc(16, sizeof(int))));
							}
							break;
						}
					}
		    	}
		    }
		}
	}
	return legs_features;
}

std::string Extraction::Calc_Direction(cv::Mat& viz_img, tf::Point leg1, tf::Point leg2, geometry_msgs::Point* leg_joint)
{
	std::string dir;
	tf::Point leg1_t = leg1;
	tf::Point leg2_t = leg2;

	tf::Point leg1_b;
	tf::Point leg2_b;

	int top1 = (int((-leg1.getY()/leg1.getZ())*525) + 240) > 4?(int((-leg1.getY()/leg1.getZ())*525) + 240):4;
	int top2 = (int((-leg2.getY()/leg2.getZ())*525) + 240) > 4?(int((-leg2.getY()/leg2.getZ())*525) + 240):4;

	int bottom1 = (int((0.4/leg1.getZ())*525) + 240) < 480?(int((0.4/leg1.getZ())*525) + 240):460;
	int bottom2 = (int((0.4/leg2.getZ())*525) + 240) < 480?(int((0.4/leg2.getZ())*525) + 240):460;

	int inter = int((0.05/leg1.getZ())*525);

	int leg1_c = int((leg1.getX()/leg1.getZ())*525 + 320);
	int leg2_c = int((leg2.getX()/leg2.getZ())*525 + 320);

	int max_w1 = (leg1_c+inter) < 640?(leg1_c+inter):639;
	int min_w1 = (leg1_c-inter) > 14?(leg1_c-inter):14;

	int max_w2 = (leg2_c+inter) < 640?(leg2_c+inter):639;
	int min_w2 = (leg2_c-inter) > 14?(leg2_c-inter):14;

	cv::rectangle(viz_img, cv::Point(leg1_c-13, top1-3), cv::Point(leg1_c-7, top1+3), cv::Scalar(255, 0, 255), 3);
	cv::rectangle(viz_img, cv::Point(leg2_c-13, top2-3), cv::Point(leg2_c-7, top2+3), cv::Scalar(255, 0, 255), 3);

	double distance = 5;
	int tmp_w = 0;
	for(int w=min_w1; w<max_w1; w++)
	{
		double dx = cloud_->points[w+640*bottom1].x - leg1.getX();
		double dz = cloud_->points[w+640*bottom1].z - leg1.getZ();

		/////////////거리값이 10센티 미만이면 Cluster////////////////////
		if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
		{
			distance = std::min(distance, sqrt(dx*dx + dz*dz));
			tmp_w = w;
		}
	}
	leg1_b = tf::Point(cloud_->points[tmp_w+640*bottom1].x, -0.5, cloud_->points[tmp_w+640*bottom1].z);
	cv::rectangle(viz_img, cv::Point(tmp_w-13, bottom1-3), cv::Point(tmp_w-7, bottom1), cv::Scalar(50, 10, 255), 3);

	distance = 5;
	tmp_w = 0;
	for(int w=min_w2; w<max_w2; w++)
	{
		double dx = cloud_->points[w+640*bottom2].x - leg2.getX();
		double dz = cloud_->points[w+640*bottom2].z - leg2.getZ();

		/////////////거리값이 10센티 미만이면 Cluster////////////////////
		if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
		{
			distance = std::min(distance, sqrt(dx*dx + dz*dz));
			tmp_w = w;
		}
	}
	leg2_b = tf::Point(cloud_->points[tmp_w+640*bottom2].x, -0.5, cloud_->points[tmp_w+640*bottom2].z);

	cv::rectangle(viz_img, cv::Point(tmp_w-13, bottom2-3), cv::Point(tmp_w-7, bottom2), cv::Scalar(50, 10, 255), 3);

	double leg1_mz=0;
	double leg2_mz=0;
	int cnt=0;
	for(int h=int(top1-float(top1-bottom1)/3); h<int(bottom1+float(top1-bottom1)/3); h++, cnt++)
	{
		distance = 5;
		for(int w=min_w1; w<max_w1; w++)
		{
			double dx = cloud_->points[w+640*h].x - leg1.getX();
			double dz = cloud_->points[w+640*h].z - leg1.getZ();

			/////////////거리값이 10센티 미만이면 Cluster////////////////////
			if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
			{
				distance = std::min(distance, sqrt(dx*dx + dz*dz));
				tmp_w = w;
			}
		}
		leg1_mz += cloud_->points[tmp_w+640*h].z;
	}
	leg1_mz /= cnt;

	cnt=0;
	for(int h=int(top2-float(top2-bottom2)/3); h<int(bottom2+float(top2-bottom2)/3); h++, cnt++)
	{
		distance = 5;
		for(int w=min_w2; w<max_w2; w++)
		{
			double dx = cloud_->points[w+640*h].x - leg2.getX();
			double dz = cloud_->points[w+640*h].z - leg2.getZ();

			/////////////거리값이 50센티 미만이면 Cluster////////////////////
			if(distance!=std::min(distance, sqrt(dx*dx + dz*dz)) && sqrt(dx*dx + dz*dz)<0.1 && !isnan(sqrt(dx*dx + dz*dz)))
			{
				distance = std::min(distance, sqrt(dx*dx + dz*dz));
				tmp_w = w;
			}
		}
		leg2_mz += cloud_->points[tmp_w+640*h].z;

	}
	leg2_mz /= cnt;

	if(leg1_mz < ((leg1_b.getZ()+leg1_t.getZ())/2) || leg2_mz < ((leg2_b.getZ()+leg2_t.getZ())/2))
		dir = std::string("front");
	else
		dir = std::string("back");

	leg_joint[0].x = leg1_b.getZ();
	leg_joint[0].y = -leg1_b.getX();
	leg_joint[0].z = leg1_b.getY();

	leg_joint[1].x = leg1_mz;
	leg_joint[1].y = std::min(-leg1_b.getX(), -leg1_t.getX()) + std::abs(leg1_b.getX()-leg1_t.getX())/2;
	leg_joint[1].z = std::min(leg1_b.getY(), leg1_t.getY()) + std::abs(leg1_b.getY()-leg1_t.getY())/2;

	leg_joint[2].x = leg1_t.getZ();
	leg_joint[2].y = -leg1_t.getX();
	leg_joint[2].z = leg1_t.getY();

	leg_joint[3].x = leg2_b.getZ();
	leg_joint[3].y = -leg2_b.getX();
	leg_joint[3].z = leg2_b.getY();

	leg_joint[4].x = leg2_mz;
	leg_joint[4].y = std::min(-leg2_b.getX(), -leg2_t.getX()) + std::abs(leg2_b.getX()-leg2_t.getX())/2;
	leg_joint[4].z = std::min(leg2_b.getY(), leg2_t.getY()) + std::abs(leg2_b.getY()-leg2_t.getY())/2;

	leg_joint[5].x = leg2_t.getZ();
	leg_joint[5].y = -leg2_t.getX();
	leg_joint[5].z = leg2_t.getY();


	return dir;
}
