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
class Draw_GUI
{
public:
	cv::Mat bgr_img, distance_img, similarity_img, hist_img[4], dgraph_img, cgraph_img, wgraph_img;
	cv::Mat bgr_canvas, dist_canvas, similarity_canvas;
	cv::Mat dist_graph_img[2], color_graph_img[2], dist_graph_canvas[2], color_graph_canvas[2];
	TDI tracked_data_information[2];
	TDI pre_tracked_data_information[2];

	float dist[4], color[4];
	int   undet_time[2];
	float draw_dist[2][50], draw_color[2][50];
	Draw_GUI()
	{
		bgr_img = cv::Mat(cv::Size(1000, 970), CV_8UC3, cv::Scalar::all(220));
		bgr_canvas = cv::Mat(cv::Size(1000, 970), CV_8UC3, cv::Scalar::all(220));
		distance_img = cv::Mat(cv::Size(460, 460), CV_8UC3, cv::Scalar::all(255));
		dist_canvas = cv::Mat(cv::Size(460, 460), CV_8UC3, cv::Scalar::all(255));
		similarity_img = cv::Mat(cv::Size(710, 440), CV_8UC3, cv::Scalar::all(255));
		similarity_canvas = cv::Mat(cv::Size(710, 440), CV_8UC3, cv::Scalar::all(255));
		hist_img[0] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(0));
		hist_img[1] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(0));
		hist_img[2] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(0));
		hist_img[3] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(0));
		dist_graph_img[0] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		dist_graph_img[1] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		dist_graph_canvas[0] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		dist_graph_canvas[1] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		color_graph_img[0] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		color_graph_img[1] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		color_graph_canvas[0] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		color_graph_canvas[1] = cv::Mat(cv::Size(250, 100), CV_8UC3, cv::Scalar::all(255));
		dgraph_img = cv::Mat(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(255));
		cgraph_img = cv::Mat(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(255));
		wgraph_img = cv::Mat(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(255));

		int n=0;
		for(n=0; n<2; n++)
		{
			pre_tracked_data_information[n].flag = false;
			pre_tracked_data_information[n].pos = pre_tracked_data_information[n].leg1 = pre_tracked_data_information[n].leg2 = tf::Vector3(0,0,0);
			pre_tracked_data_information[n].hist_val = (int*)calloc(16, sizeof(int));

			tracked_data_information[n].flag = false;
			tracked_data_information[n].pos = tracked_data_information[n].leg1 = tracked_data_information[n].leg2 = tf::Vector3(0,0,0);
			tracked_data_information[n].hist_val = (int*)calloc(16, sizeof(int));
		}
		for(n=0; n<4; n++)
		{
			dist[n] = 0;
			color[n] = 8;
		}

	}

	~Draw_GUI()
	{
	}

	void update_new_data()
	{
		for(std::list<LegsFeature*>::iterator tf_iter = tracked_features_.begin(); tf_iter != tracked_features_.end(); tf_iter++)
		{
			tracked_data_information[atoi((*tf_iter)->object_id.data())].flag = true;
			tracked_data_information[atoi((*tf_iter)->object_id.data())].pos = (*tf_iter)->center_;
			tracked_data_information[atoi((*tf_iter)->object_id.data())].leg1 = (*tf_iter)->leg1_->loc_;
			tracked_data_information[atoi((*tf_iter)->object_id.data())].leg2 = (*tf_iter)->leg2_->loc_;
			tracked_data_information[atoi((*tf_iter)->object_id.data())].hist_val = (*tf_iter)->hist_;
			undet_time[atoi((*tf_iter)->object_id.data())] = (*tf_iter)->undetected_time_;
		}
	}

	void copy_data()
	{
		for(int n=0; n<2; n++)
		{
			pre_tracked_data_information[n].flag = tracked_data_information[n].flag;
			pre_tracked_data_information[n].pos = tracked_data_information[n].pos;
			pre_tracked_data_information[n].leg1 = tracked_data_information[n].leg1;
			pre_tracked_data_information[n].leg2 = tracked_data_information[n].leg2;
			pre_tracked_data_information[n].hist_val = tracked_data_information[n].hist_val;
		}
	}

	void release_data()
	{
		for(int n=0; n<2; n++)
		{
			tracked_data_information[n].flag = false;
			tracked_data_information[n].pos = tracked_data_information[n].leg1 = tracked_data_information[n].leg2 = tf::Vector3(0,0,0);
			tracked_data_information[n].hist_val = (int*)calloc(16, sizeof(int));
			undet_time[n] = 0;
		}
	}

	void draw()
	{
		draw_distance_differ();
		draw_information();
		draw_similarity();
	}

	void update_distance_differ()
	{
		bool flag[2]={false, false};
		cv::Mat img_roi;
		distance_img.copyTo(dist_canvas);

		for(int n=0; n<2; n++)
		{
			cv::Point leg1_pos, leg2_pos, cen_pos, id_pos;
			if(tracked_data_information[n].flag)
			{
				leg1_pos = cv::Point(225-tracked_data_information[n].leg1.getX()*100, 450-tracked_data_information[n].leg1.getZ()*100);
				leg2_pos = cv::Point(225-tracked_data_information[n].leg2.getX()*100, 450-tracked_data_information[n].leg2.getZ()*100);
				cen_pos =  cv::Point(225-tracked_data_information[n].pos.getX()*100, 450-tracked_data_information[n].pos.getZ()*100);
				id_pos =  cv::Point(225-tracked_data_information[n].pos.getX()*100-10, 450-tracked_data_information[n].pos.getZ()*100);
				cv::circle(dist_canvas, leg1_pos, 3, cv::Scalar(0,100,100), 2);
				cv::circle(dist_canvas, leg2_pos, 3, cv::Scalar(0,100,100), 2);

				cv::circle(dist_canvas, cen_pos, 5, cv::Scalar(0,0,255), 2);
				char id[10];
				sprintf(id, "id : %d", n);
				cv::putText(dist_canvas, id, id_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
			}

			cv::Point pre_cen_pos;
			if(pre_tracked_data_information[n].flag)
			{
				pre_cen_pos =  cv::Point(225-pre_tracked_data_information[n].pos.getX()*100, 450-pre_tracked_data_information[n].pos.getZ()*100);
				cv::rectangle(dist_canvas, cv::Point(pre_cen_pos.x-3, pre_cen_pos.y-3), cv::Point(pre_cen_pos.x+3, pre_cen_pos.y+3), cv::Scalar(255,0,255), 2);
			}

			if(tracked_data_information[n].flag && pre_tracked_data_information[n].flag)
				cv::line(dist_canvas, pre_cen_pos, cen_pos, cv::Scalar(255,0,0));
		}

		for(int n=0; n<4; n++)
		{
			if(tracked_data_information[n/2].flag && pre_tracked_data_information[n%2].flag)
			{
				float dx, dz;
				dx = tracked_data_information[n/2].pos.getX() - pre_tracked_data_information[n%2].pos.getX();
				dz = tracked_data_information[n/2].pos.getZ() - pre_tracked_data_information[n%2].pos.getZ();
				dist[n] = sqrt(dx*dx + dz*dz);
			}
		}
		/////copy to background image////
		img_roi = bgr_canvas(cv::Rect(10, 30, 460, 460));	dist_canvas.copyTo(img_roi);
	}

	void update_information_histogram()
	{
		cv::Mat img_roi;

		cv::Mat buf(1, 16, CV_8UC3);
		for( int i = 0; i < 16; i++ )
					buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./16), 255, 255);
		cvtColor(buf, buf, CV_HSV2BGR);

		int binW = hist_img[0].cols / 16;
		for(int n=0; n<2; n++)
		{
			for(int i=0; i<16; i++)
			{
				if(tracked_data_information[n].flag)
				{
					int sat_val = cv::saturate_cast<int>(tracked_data_information[n].hist_val[i]*hist_img[n].rows/255);
					cv::rectangle( hist_img[n], cv::Point(i*binW,hist_img[n].rows),
										cv::Point((i+1)*binW,hist_img[n].rows - sat_val),
										cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
				}
				if(pre_tracked_data_information[n].flag)
				{
					int sat_val = cv::saturate_cast<int>(pre_tracked_data_information[n].hist_val[i]*hist_img[2+n].rows/255);
					cv::rectangle( hist_img[2+n], cv::Point(i*binW,hist_img[2+n].rows),
										cv::Point((i+1)*binW,hist_img[2+n].rows - sat_val),
										cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
				}
			}
		}

		for(int n=0; n<4; n++)
		{
			color[n] = 8;
			if(tracked_data_information[n/2].flag && pre_tracked_data_information[n%2].flag)
			{
				for(int i=0; i<16; i++)
				{
					if(abs(tracked_data_information[n/2].hist_val[i] - pre_tracked_data_information[n%2].hist_val[i]) < 10)	;
					else
						color[n] += ((tracked_data_information[n/2].hist_val[i] - pre_tracked_data_information[n%2].hist_val[i]) * (tracked_data_information[n/2].hist_val[i] - pre_tracked_data_information[n%2].hist_val[i]));
				}
				color[n] = sqrt(color[n])/10;
				color[n] = std::max(8, (int)color[n]);
				color[n] = std::min(30, (int)color[n]);
			}
		}

		img_roi = bgr_canvas(cv::Rect(480, 30, 250, 100));		hist_img[0].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(740, 30, 250, 100));		hist_img[1].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(480, 160, 250, 100));		hist_img[2].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(740, 160, 250, 100));		hist_img[3].copyTo(img_roi);

		hist_img[0] = cv::Scalar::all(0);
		hist_img[1] = cv::Scalar::all(0);
		hist_img[2] = cv::Scalar::all(0);
		hist_img[3] = cv::Scalar::all(0);
	}

	void update_information_graph()
	{
		cv::Mat img_roi;

		for(int n=0; n<2; n++)
		{
			dist_graph_img[n].copyTo(dist_graph_canvas[n]);
			color_graph_img[n].copyTo(color_graph_canvas[n]);
			for(int i=49; i>0; i--)
			{
				draw_dist[n][i] = draw_dist[n][i-1];
				draw_color[n][i] = draw_color[n][i-1];
			}
			draw_dist[n][0] = dist[n*3];
			draw_color[n][0] = color[n*3];

			for(int i=1; i<50; i++)
			{
				cv::line(dist_graph_canvas[n], cv::Point(10+((i-1)*4.5), 85-(draw_dist[n][i-1]*70)), cv::Point(10+(i*4.5), 85-(draw_dist[n][i]*70)), cv::Scalar(0,10,10));
				cv::circle(dist_graph_canvas[n], cv::Point(10+(i*4.5), 85-(draw_dist[n][i]*70)), 1, cv::Scalar(0,100,100), 1);
			}

			for(int i=1; i<50; i++)
			{
				cv::line(color_graph_canvas[n], cv::Point(10+((i-1)*4.5), 85-(draw_color[n][i-1])), cv::Point(10+(i*4.5), 85-(draw_color[n][i])), cv::Scalar(0,10,10));
				cv::circle(color_graph_canvas[n], cv::Point(10+(i*4.5), 85-(draw_color[n][i])), 1, cv::Scalar(0,100,100), 1);
			}
		}
		img_roi = bgr_canvas(cv::Rect(480, 290, 250, 100));	dist_graph_canvas[0].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(740, 290, 250, 100));	dist_graph_canvas[1].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(480, 390, 250, 100));	color_graph_canvas[0].copyTo(img_roi);
		img_roi = bgr_canvas(cv::Rect(740, 390, 250, 100));	color_graph_canvas[1].copyTo(img_roi);
	}

	void update_similarity()
	{
		cv::Mat img_roi[3][4];

		similarity_img.copyTo(similarity_canvas);

		img_roi[0][0] = similarity_canvas(cv::Rect(20, 0, 210, 110));
		img_roi[0][1] = similarity_canvas(cv::Rect(20, 110, 210, 110));
		img_roi[0][2] = similarity_canvas(cv::Rect(20, 220, 210, 110));
		img_roi[0][3] = similarity_canvas(cv::Rect(20, 330, 210, 110));

		img_roi[1][0] = similarity_canvas(cv::Rect(250, 0, 210, 110));
		img_roi[1][1] = similarity_canvas(cv::Rect(250, 110, 210, 110));
		img_roi[1][2] = similarity_canvas(cv::Rect(250, 220, 210, 110));
		img_roi[1][3] = similarity_canvas(cv::Rect(250, 330, 210, 110));

		img_roi[2][0] = similarity_canvas(cv::Rect(480, 0, 210, 110));
		img_roi[2][1] = similarity_canvas(cv::Rect(480, 110, 210, 110));
		img_roi[2][2] = similarity_canvas(cv::Rect(480, 220, 210, 110));
		img_roi[2][3] = similarity_canvas(cv::Rect(480, 330, 210, 110));

		cv::Point sim_value_canvas[4] = {cv::Point(900,580), cv::Point(900,690), cv::Point(900,800), cv::Point(900,910)};

		for(int i=0; i<4; i++)
		{
			//distance similarity
			cv::Mat tmp_dgraph(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(0));
			dgraph_img.copyTo(tmp_dgraph);
			int dist_value = (1 - ((float)dist[i])*((float)dist[i]))*100;
			cv::circle(tmp_dgraph, cv::Point(dist[i]*200+6,5+100-dist_value), 3, cv::Scalar(0,0,255), 2);
			tmp_dgraph.copyTo(img_roi[0][i]);

			//color similarity
			cv::Mat tmp_cgraph(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(0));
			cgraph_img.copyTo(tmp_cgraph);
			int color_value = (-1.0/22.0*(float)color[i] + 15.0/11.0)*100;
			cv::circle(tmp_cgraph, cv::Point((color[i]-8)*200.0/22.0+6,5+100-color_value), 3, cv::Scalar(255,0,0), 2);
			tmp_cgraph.copyTo(img_roi[1][i]);

			cv::Mat tmp_wgraph(cv::Size(210, 110), CV_8UC3, cv::Scalar::all(0));
			wgraph_img.copyTo(tmp_wgraph);
			int weight_d_value = (0.3 + 0.4 * exp((-(float)(undet_time[i/2]*undet_time[i/2]))/(20*20))) * 100;
			cv::circle(tmp_wgraph, cv::Point(undet_time[i/2]*4+6,5+100-weight_d_value), 3, cv::Scalar(255,0,0), 2);
			tmp_wgraph.copyTo(img_roi[2][i]);

			char sim_value[30];
			sprintf(sim_value, "%.1f%%", (weight_d_value * (float(dist_value)/100) + (100-weight_d_value) * (float(color_value)/100)));
			cv::putText(bgr_canvas, sim_value, sim_value_canvas[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0),2,0.5);
		}
		cv::putText(bgr_canvas, "=", cv::Point(880,570), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_canvas, "=", cv::Point(880,680), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_canvas, "=", cv::Point(880,790), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_canvas, "=", cv::Point(880,900), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);

		similarity_canvas.copyTo(bgr_canvas(cv::Rect(130, 520, 710, 440)));
	}

	void draw_distance_differ()
	{
		cv::Mat img_roi;
		// [Draw Distance Differ]
		cv::putText(bgr_img, "Distance Differ", cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		/////grid////
		for(int i=25; i<490; i+=100)
			cv::line(distance_img, cv::Point(i,0), cv::Point(i,420), cv::Scalar(230,230,230));
		for(int i=420; i>0; i-=100)
			cv::line(distance_img, cv::Point(0,i), cv::Point(450,i), cv::Scalar(230,230,230));
		/////axis name////
		cv::circle(distance_img, cv::Point(225, 420), 10, cv::Scalar(0,0,0), 2);
	    cv::putText(distance_img, "R", cv::Point(220,425), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0),2,0.5);
	    cv::putText(distance_img, "-1", cv::Point(10,435), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
	    cv::putText(distance_img, "1", cv::Point(420,435), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
	    cv::putText(distance_img, "1", cv::Point(5,225), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
	    cv::putText(distance_img, "2", cv::Point(5,25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150,150,150),2,0.5);
		/////copy to background image////
		img_roi = bgr_img(cv::Rect(10, 30, 460, 460));	distance_img.copyTo(img_roi);
	}

	void draw_information()
	{
		cv::Mat img_roi;
		// [Information]
		cv::putText(bgr_img, "Tracked Humans Feature", cv::Point(480,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		/////copy to background image////
		img_roi = bgr_img(cv::Rect(480, 30, 250, 100));		hist_img[0].copyTo(img_roi);
		img_roi = bgr_img(cv::Rect(740, 30, 250, 100));		hist_img[1].copyTo(img_roi);
//		cv::putText(bgr_img, "(1)                                                   (2)", cv::Point(590,160), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);

		cv::putText(bgr_img, "Detected Humans Feature", cv::Point(480,150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		/////copy to background image////
		img_roi = bgr_img(cv::Rect(480, 160, 250, 100));	hist_img[2].copyTo(img_roi);
		img_roi = bgr_img(cv::Rect(740, 160, 250, 100));	hist_img[3].copyTo(img_roi);
//		cv::putText(bgr_img, "(1)                                                   (2)", cv::Point(590,330), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);

		cv::putText(bgr_img, "Feature Differ", cv::Point(480,280), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		/////grid////
		cv::line(dist_graph_img[0], cv::Point(10,85), cv::Point(240,85), cv::Scalar(0,0,0));
		cv::line(dist_graph_img[0], cv::Point(10,50), cv::Point(240,50), cv::Scalar(230,230,230));
		cv::line(dist_graph_img[0], cv::Point(10,15), cv::Point(240,15), cv::Scalar(230,230,230));
		for(int i=50; i<235; i+=45)
			cv::line(dist_graph_img[0], cv::Point(i,5), cv::Point(i,85), cv::Scalar(230,230,230));
		cv::line(dist_graph_img[0], cv::Point(10,5), cv::Point(10,85), cv::Scalar(0,0,0));
		/////axis name////
		cv::putText(dist_graph_img[0], "1", cv::Point(3,20), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "0", cv::Point(3,90), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "1", cv::Point(48,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "2", cv::Point(93,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "3", cv::Point(138,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "4", cv::Point(183,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(dist_graph_img[0], "5", cv::Point(228,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);

		dist_graph_img[0].copyTo(dist_graph_img[1]);
		/////copy to background image////
		img_roi = bgr_img(cv::Rect(480, 290, 250, 100));	dist_graph_img[0].copyTo(img_roi);
		img_roi = bgr_img(cv::Rect(740, 290, 250, 100));	dist_graph_img[1].copyTo(img_roi);

		/////grid////
		cv::line(color_graph_img[0], cv::Point(10,85), cv::Point(240,85), cv::Scalar(0,0,0));
		cv::line(color_graph_img[0], cv::Point(10,60), cv::Point(240,60), cv::Scalar(230,230,230));
		cv::line(color_graph_img[0], cv::Point(10,35), cv::Point(240,35), cv::Scalar(230,230,230));
		cv::line(color_graph_img[0], cv::Point(10,10), cv::Point(240,10), cv::Scalar(230,230,230));
		for(int i=50; i<235; i+=45)
			cv::line(color_graph_img[0], cv::Point(i,5), cv::Point(i,85), cv::Scalar(230,230,230));
		cv::line(color_graph_img[0], cv::Point(10,5), cv::Point(10,85), cv::Scalar(0,0,0));
		/////axis name////
		cv::putText(color_graph_img[0], "3", cv::Point(3,15), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "2", cv::Point(3,40), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "1", cv::Point(3,65), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "0", cv::Point(3,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);

		cv::putText(color_graph_img[0], "1", cv::Point(48,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "2", cv::Point(93,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "3", cv::Point(138,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "4", cv::Point(183,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);
		cv::putText(color_graph_img[0], "5", cv::Point(228,95), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0),1,0.5);

		color_graph_img[0].copyTo(color_graph_img[1]);
		/////copy to background image////
		img_roi = bgr_img(cv::Rect(480, 390, 250, 100));	color_graph_img[0].copyTo(img_roi);
		img_roi = bgr_img(cv::Rect(740, 390, 250, 100));	color_graph_img[1].copyTo(img_roi);
	}

	void draw_similarity()
	{
		cv::Mat img_roi;
		// Weight Function & Similarity
		cv::putText(bgr_img, "Similarity Calcurate", cv::Point(10,510), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);

		/////distance graph////
		cv::line(dgraph_img, cv::Point(5,5), cv::Point(5,105), cv::Scalar(200,200,200));
		cv::line(dgraph_img, cv::Point(5,5), cv::Point(205,5), cv::Scalar(200,200,200));
		cv::line(dgraph_img, cv::Point(205,5), cv::Point(205,105), cv::Scalar(200,200,200));
		cv::line(dgraph_img, cv::Point(5,105), cv::Point(205,105), cv::Scalar(200,200,200));

		/////draw graph////
		for(int i=5; i<210; i+=40)
			cv::line(dgraph_img, cv::Point(i,5), cv::Point(i,105), cv::Scalar(130,130,130));
		for(int i=5; i<110; i+=20)
			cv::line(dgraph_img, cv::Point(5,i), cv::Point(205,i), cv::Scalar(130,130,130));
		cv::rectangle(dgraph_img, cv::Point(10,10), cv::Point(200,100), cv::Scalar(255,255,255),CV_FILLED);
		int y_value_pre = 100;
		for(int i=0; i<100; i+=1)
		{
			int y_value = (1 - ((float)i/100)*((float)i/100))*100;
			//5 => y축 max값 105 => y축 min값
			cv::line(dgraph_img, cv::Point(i*2+5,5+100-y_value_pre), cv::Point(i*2+6,5+100-y_value), cv::Scalar(0,0,255));
			y_value_pre = y_value;
		}

		/////copy to function background image////
		img_roi = similarity_img(cv::Rect(20, 0, 210, 110));	dgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(20, 110, 210, 110));	dgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(20, 220, 210, 110));	dgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(20, 330, 210, 110));	dgraph_img.copyTo(img_roi);

		/////color graph////
		cv::line(cgraph_img, cv::Point(5,5), cv::Point(5,105), cv::Scalar(200,200,200));
		cv::line(cgraph_img, cv::Point(5,5), cv::Point(205,5), cv::Scalar(200,200,200));
		cv::line(cgraph_img, cv::Point(205,5), cv::Point(205,105), cv::Scalar(200,200,200));
		cv::line(cgraph_img, cv::Point(5,105), cv::Point(205,105), cv::Scalar(200,200,200));

		/////draw graph////
		for(int i=5; i<210; i+=40)
			cv::line(cgraph_img, cv::Point(i,5), cv::Point(i,105), cv::Scalar(130,130,130));
		for(int i=5; i<110; i+=20)
			cv::line(cgraph_img, cv::Point(5,i), cv::Point(205,i), cv::Scalar(130,130,130));
		cv::rectangle(cgraph_img, cv::Point(10,10), cv::Point(200,100), cv::Scalar(255,255,255),CV_FILLED);
		cv::line(cgraph_img, cv::Point(5,5), cv::Point(205,105), cv::Scalar(200,0,0));
		img_roi = similarity_img(cv::Rect(250, 0, 210, 110));	cgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(250, 110, 210, 110));	cgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(250, 220, 210, 110));	cgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(250, 330, 210, 110));	cgraph_img.copyTo(img_roi);

		/////weight graph////
		cv::line(wgraph_img, cv::Point(5,5), cv::Point(5,105), cv::Scalar(200,200,200));
		cv::line(wgraph_img, cv::Point(5,5), cv::Point(205,5), cv::Scalar(200,200,200));
		cv::line(wgraph_img, cv::Point(205,5), cv::Point(205,105), cv::Scalar(200,200,200));
		cv::line(wgraph_img, cv::Point(5,105), cv::Point(205,105), cv::Scalar(200,200,200));

		/////draw graph////
		for(int i=5; i<210; i+=40)
			cv::line(wgraph_img, cv::Point(i,5), cv::Point(i,105), cv::Scalar(130,130,130));
		for(int i=5; i<110; i+=20)
			cv::line(wgraph_img, cv::Point(5,i), cv::Point(205,i), cv::Scalar(130,130,130));
		cv::rectangle(wgraph_img, cv::Point(10,10), cv::Point(200,100), cv::Scalar(255,255,255),CV_FILLED);
		y_value_pre = 70;
		for(int i=0; i<50; i+=1)
		{
			int y_value = (0.3 + 0.4 * exp((-(float)(i*i))/(20*20))) * 100;
			cv::line(wgraph_img, cv::Point(i*4+5,5+100-y_value_pre), cv::Point(i*4+6,5+100-y_value), cv::Scalar(0,0,255));
			y_value_pre = y_value;
		}
		img_roi = similarity_img(cv::Rect(480, 0, 210, 110));	wgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(480, 110, 210, 110));	wgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(480, 220, 210, 110));	wgraph_img.copyTo(img_roi);
		img_roi = similarity_img(cv::Rect(480, 330, 210, 110));	wgraph_img.copyTo(img_roi);

		/////copy to background image////
		img_roi = bgr_img(cv::Rect(130, 520, 710, 440)); similarity_img.copyTo(img_roi);

		cv::putText(bgr_img, " t(1)-d(1)", cv::Point(10,570), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_img, " t(1)-d(2)", cv::Point(10,680), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_img, " t(2)-d(1)", cv::Point(10,790), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
		cv::putText(bgr_img, " t(2)-d(2)", cv::Point(10,900), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),2,0.5);
	}
};

void *graph_thr_func(void *data)
{
	Draw_GUI draw_gui;

	draw_gui.draw();

	int cnt=0;
	while(!thread_flag)
	{
//		if(cnt++ > 10)
	//	{
			cnt = 0;

			draw_gui.bgr_img.copyTo(draw_gui.bgr_canvas);
			draw_gui.update_new_data();
			draw_gui.update_distance_differ();
			draw_gui.update_information_histogram();
			draw_gui.update_information_graph();
			draw_gui.update_similarity();
			draw_gui.copy_data();
			draw_gui.release_data();
			cv::imshow("Processing GUI", draw_gui.bgr_canvas);
//			cv::imshow("Processing GUI", draw_gui.update_distance_differ);
			cvResizeWindow( "Processing GUI", 990, 970 );
			//위치, 색상
//		}


		usleep(100000);
	}
	pthread_exit((void *) 0);
}

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
				cv::Mat hog_vizimg = Get_Hogdescriptor_Visual(found_img, descriptors, cvSize(64, 128), cvSize(8,8), 3, 3);

				imshow("hog_viz", hog_vizimg);

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
                ROS_INFO("lost tracking");
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

	cv::Mat Get_Hogdescriptor_Visual(cv::Mat& origImg,
	                                   std::vector<float>& descriptorValues,
	                                   cv::Size winSize,
	                                   cv::Size cellSize,
	                                   int scaleFactor,
	                                   double viz_factor)
	{
	    cv::Mat visual_image;
	    cv::resize(origImg, visual_image, cv::Size(origImg.cols*scaleFactor, origImg.rows*scaleFactor));

	    int gradientBinSize = 9;
	    // dividing 180° into 9 bins, how large (in rad) is one bin?
	    float radRangeForOneBin = 3.14/(float)gradientBinSize;

	    // prepare data structure: 9 orientation / gradient strenghts for each cell
	    int cells_in_x_dir = winSize.width / cellSize.width;
	    int cells_in_y_dir = winSize.height / cellSize.height;
	    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
	    float*** gradientStrengths = new float**[cells_in_y_dir];
	    int** cellUpdateCounter   = new int*[cells_in_y_dir];
	    for (int y=0; y<cells_in_y_dir; y++)
	    {
	        gradientStrengths[y] = new float*[cells_in_x_dir];
	        cellUpdateCounter[y] = new int[cells_in_x_dir];
	        for (int x=0; x<cells_in_x_dir; x++)
	        {
	            gradientStrengths[y][x] = new float[gradientBinSize];
	            cellUpdateCounter[y][x] = 0;

	            for (int bin=0; bin<gradientBinSize; bin++)
	                gradientStrengths[y][x][bin] = 0.0;
	        }
	    }

	    // nr of blocks = nr of cells - 1
	    // since there is a new block on each cell (overlapping blocks!) but the last one
	    int blocks_in_x_dir = cells_in_x_dir - 1;
	    int blocks_in_y_dir = cells_in_y_dir - 1;

	    // compute gradient strengths per cell
	    int descriptorDataIdx = 0;
	    int cellx = 0;
	    int celly = 0;

	    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
	    {
	        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)
	        {
	            // 4 cells per block ...
	            for (int cellNr=0; cellNr<4; cellNr++)
	            {
	                // compute corresponding cell nr
	                int cellx = blockx;
	                int celly = blocky;
	                if (cellNr==1) celly++;
	                if (cellNr==2) cellx++;
	                if (cellNr==3)
	                {
	                    cellx++;
	                    celly++;
	                }

	                for (int bin=0; bin<gradientBinSize; bin++)
	                {
	                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
	                    descriptorDataIdx++;

	                    gradientStrengths[celly][cellx][bin] += gradientStrength;

	                } // for (all bins)


	                // note: overlapping blocks lead to multiple updates of this sum!
	                // we therefore keep track how often a cell was updated,
	                // to compute average gradient strengths
	                cellUpdateCounter[celly][cellx]++;

	            } // for (all cells)


	        } // for (all block x pos)
	    } // for (all block y pos)


	    // compute average gradient strengths
	    for (int celly=0; celly<cells_in_y_dir; celly++)
	    {
	        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
	        {

	            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

	            // compute average gradient strenghts for each gradient bin direction
	            for (int bin=0; bin<gradientBinSize; bin++)
	            {
	                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
	            }
	        }
	    }
	    // draw cells
	    for (int celly=0; celly<cells_in_y_dir; celly++)
	    {
	        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
	        {
	            int drawX = cellx * cellSize.width;
	            int drawY = celly * cellSize.height;

	            int mx = drawX + cellSize.width/2;
	            int my = drawY + cellSize.height/2;

	            cv::rectangle(visual_image,
							cv::Point(max(0,drawX*scaleFactor),max(0,drawY*scaleFactor)),
							cv::Point(min(640,(drawX+cellSize.width)*scaleFactor),min(480,
							(drawY+cellSize.height)*scaleFactor)),
							CV_RGB(200,100,100),
							1);

	            // draw in each cell all 9 gradient strengths
	            for (int bin=0; bin<gradientBinSize; bin++)
	            {
	                float currentGradStrength = gradientStrengths[celly][cellx][bin];

	                // no line to draw?
	                if (currentGradStrength==0)
	                    continue;

	                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;

	                float dirVecX = cos( currRad );
	                float dirVecY = sin( currRad );
	                float maxVecLen = cellSize.width/2;
	                float scale = viz_factor; // just a visual_imagealization scale,
	                                          // to see the lines better

	                // compute line coordinates
	                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
	                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
	                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
	                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

	                // draw gradient visual_imagealization
	                cv::line(visual_image,
	                		cv::Point(x1*scaleFactor,y1*scaleFactor),
	                		cv::Point(x2*scaleFactor,y2*scaleFactor),
	                		CV_RGB(255,255,255),
	                		1);

	            } // for (all bins)
	        } // for (cellx)
	    } // for (celly)

	    // don't forget to free memory allocated by helper data structures!
	    for (int y=0; y<cells_in_y_dir; y++)
	    {
	      for (int x=0; x<cells_in_x_dir; x++)
	      {
	           delete[] gradientStrengths[y][x];
	      }
	      delete[] gradientStrengths[y];
	      delete[] cellUpdateCounter[y];
	    }
	    delete[] gradientStrengths;
	    delete[] cellUpdateCounter;

	    return visual_image;
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
		thread_id = pthread_create(&p_thread, NULL, graph_thr_func, NULL);
		ros::spin();
	}
	return 0;
}

