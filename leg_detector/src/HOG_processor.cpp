#include "detector/HOG_processor.h"
#include <boost/thread.hpp>
#include <boost/random.hpp>
boost::mt19937 boost_seed(time(0));

using namespace hog_detector;


Trainer::Trainer(std::string featuresFile, std::string svmModelFile, std::string pos_dir, std::string neg_dir)
{
	cv::HOGDescriptor hog;
	size_t posCount = 0, negCount = 0;
	cv::Mat rgb_img;
	DIR *dir;
	struct dirent* entity;

	SVMLight::SVMTrainer svm(featuresFile);
	std::string dirToOpen = pos_dir;

	dir = opendir(dirToOpen.c_str());

	std::cout << "Training Positive" << std::endl;
	std::cout << "Process directory: " << dirToOpen.c_str() << std::endl;

	if(NULL == dir)
	{
		std::cout << "could not open directory: " << dirToOpen.c_str() << std::endl;
//		ros::shutdown();
	}
	cv::Mat add_img(cv::Size(64, 128), CV_32FC1, cv::Scalar::all(0));

	while(entity = readdir(dir))
	{
		if(entity->d_type == DT_REG)
		{
			char *tmp;
			tmp = strtok(entity->d_name, "&");

			int num=0;
			while(tmp!=NULL)
			{
				tmp = strtok(NULL, "&.");
			}
			char jpg_file[200];
			sprintf(jpg_file, "%s%s", pos_dir.c_str(), entity->d_name);
			rgb_img = cv::imread(jpg_file,CV_LOAD_IMAGE_GRAYSCALE);
	        if (!rgb_img.data)
	            break;

	        cv::resize(rgb_img, rgb_img, cvSize(64, 128));

	        cv::Mat dst_img;
			for(int rot=0; rot<10; rot+=3)
			{
				cv::Mat mat = cv::getRotationMatrix2D(cv::Point(32,64),rot,1.0);
				cv::warpAffine(rgb_img, dst_img, mat, rgb_img.size(), CV_INTER_CUBIC + CV_WARP_FILL_OUTLIERS,cv::BORDER_WRAP , cvScalarAll( 255));

//				cv::equalizeHist(dst_img, dst_img);
				// obtain feature vector:
				std::vector<float> featureVector;
				hog.compute(dst_img, featureVector, cv::Size(8, 8), cv::Size(0, 0));

				for(int v=0; v<128; v++)
				{
					for(int u=0; u<64; u++)
					{
						add_img.at<float>(v, u) += float(dst_img.at<unsigned char>(v, u))/2360;
					}
				}
				// write feature vector to file that will be used for training:
				svm.writeFeatureVectorToFile(featureVector, true);                  // true = positive sample
				posCount++;
				// clean up:
				featureVector.clear();
			}
			for(int rot=0; rot>-10; rot-=3)
			{
				cv::Mat mat = cv::getRotationMatrix2D(cv::Point(32,64),rot,1.0);
				cv::warpAffine(rgb_img, dst_img, mat, rgb_img.size(), CV_INTER_CUBIC + CV_WARP_FILL_OUTLIERS,cv::BORDER_WRAP , cvScalarAll( 255));

//				cv::equalizeHist(dst_img, dst_img);
				// obtain feature vector:
				std::vector<float> featureVector;
				hog.compute(dst_img, featureVector, cv::Size(8, 8), cv::Size(0, 0));

				for(int v=0; v<128; v++)
				{
					for(int u=0; u<64; u++)
					{
						add_img.at<float>(v, u) += float(dst_img.at<unsigned char>(v, u))/2360;
					}
				}
				// write feature vector to file that will be used for training:
				svm.writeFeatureVectorToFile(featureVector, true);                  // true = positive sample
				posCount++;
				// clean up:
				featureVector.clear();
			}
	        rgb_img.release();              // we don't need the original image anymore
		}
	}

	closedir(dir);

	cv::Mat tmp_img(cv::Size(64, 128), CV_8UC1, cv::Scalar::all(0));
	add_img.convertTo(tmp_img, CV_8UC1);

//	std::cout << posCount << std::endl;
//	cv::imwrite("/home/jigwan/simonpic/src/leg_detector/leg_mean.jpg", tmp_img);
//	std::vector<float> descriptors;
//	hog.compute(tmp_img, descriptors, cv::Size(8, 8), cv::Size(0, 0));
//	cv::Mat hog_vizimg = Get_Hogdescriptor_Visual(tmp_img, descriptors, cvSize(64, 128), cvSize(8,8), 2, 2);
//	cv::imwrite("/home/jigwan/simonpic/src/leg_detector/leg_mean_hog.jpg", hog_vizimg);

	std::cout << "Training Negative" << std::endl;
	dirToOpen = neg_dir;
	dir = opendir(dirToOpen.c_str());

	std::cout << "Process directory: " << dirToOpen.c_str() << std::endl;

	if(NULL == dir)
	{
		std::cout << "could not open directory: " << dirToOpen.c_str() << std::endl;
	}

	while(entity = readdir(dir))
	{
		if(entity->d_type == DT_REG)
		{
			char *tmp;
			tmp = strtok(entity->d_name, "  ");

			int num=0;
			while(tmp!=NULL)
			{
				tmp = strtok(NULL, " .");
			}

			char jpg_file[200];
			sprintf(jpg_file, "%s%s", neg_dir.c_str(), entity->d_name);
			rgb_img = cv::imread(jpg_file,CV_LOAD_IMAGE_GRAYSCALE);

	        if (!rgb_img.data)
	            break;
	        cv::resize(rgb_img, rgb_img, cvSize(rgb_img.size().width*2, rgb_img.size().height*2));
	        if(rgb_img.size().width<64 || rgb_img.size().height<128)
	        {
	        	continue;
	        }
	        else
	        {
	        	int iter=0;
	        	if(rgb_img.size().width/64 > rgb_img.size().height/128)
	        		iter = rgb_img.size().width/64;
	        	else
	        		iter = rgb_img.size().height/128;

		        cv::equalizeHist(rgb_img, rgb_img);
				for(int iter_count = 0; iter_count < iter*2; iter_count++)
				{
					boost::variate_generator< boost::mt19937&, boost::uniform_int<> > gen_x(boost_seed, boost::uniform_int<> (0, rgb_img.size().width-64));
					boost::variate_generator< boost::mt19937&, boost::uniform_int<> > gen_y(boost_seed, boost::uniform_int<> (0, rgb_img.size().height-128));
//
					int gen_x_ = gen_x();
					int gen_y_ = gen_y();
//
					cv::Mat imageROI = rgb_img(cv::Rect(gen_x_, gen_y_, 64, 128));

//					cv::resize(rgb_img, rgb_img, cvSize(64, 128));
					// obtain feature vector:
					std::vector<float> featureVector;
					hog.compute(imageROI, featureVector, cv::Size(8, 8), cv::Size(0, 0));

					// write feature vector to file that will be used for training:
					svm.writeFeatureVectorToFile(featureVector, false);                  // true = positive sample
					negCount++;

					// clean up:
					featureVector.clear();
				}
	        }
	        rgb_img.release();              // we don't need the original image anymore
		}
	}
	closedir(dir);
	std::cout   << "finished writing features: "
				<< posCount << " positive and "
				<< negCount << " negative samples used" << std::endl;

	std::string modelName(svmModelFile);
	svm.trainAndSaveModel(modelName);
	std::cout   << "SVM saved to " << modelName << std::endl;
}

Trainer::~Trainer()
{
}

Detector::Detector(std::string svmModelFile)
{
	svmModelFile_ = svmModelFile;
	hog_.cellSize;
}

Detector::~Detector()
{
}

void Detector::Set_SVM()
{
	SVMLight::SVMClassifier classify(svmModelFile_);
	std::vector<float> descriptorVector = classify.getDescriptorVector();
	hog_.setSVMDetector(descriptorVector);
}

bool Detector::Classify_HOG_SVM(cv::Mat& img_data, cv::Mat& img_viz, cv::Rect rect, int groupThreshold, cv::Size winStride, cv::Size trainingPadding, double scale, double hitThreshold)
{
	if((img_data.rows >= 128) && (img_data.cols >= 64))
	{
		cv::Mat gray_img;
		cv::cvtColor(img_data, gray_img, CV_BGR2GRAY);
		cv::equalizeHist(gray_img, gray_img);
		hog_.detectMultiScale(gray_img, found_, hitThreshold, winStride, trainingPadding, scale, groupThreshold);
		if(found_.size())
		{
//			Show_Detections(img_viz, rect);
			found_.clear();
			return 1;
		}
		else
		{
			found_.clear();
			return 0;
		}
	}
	else
		return 0;
}

void Detector::Show_Detections(cv::Mat& img, cv::Rect rect)
{
	std::vector<cv::Rect> found_filtered;
    size_t i, j;
    for (i = 0; i < found_.size(); ++i)
    {
        cv::Rect r = found_[i];
        for (j = 0; j < found_.size(); ++j)
            if (j != i && (r & found_[j]) == r)
                break;
        if (j == found_.size())
            found_filtered.push_back(r);
    }
    for (i = 0; i < found_filtered.size(); i++)
    {
        cv::Rect r = found_filtered[i];
        cv::Point tl(r.tl().x + rect.x, r.tl().y + rect.y);
        cv::Point br(r.br().x + rect.x, r.br().y + rect.y);

//		cv::Mat found_img = img(r);
//		cv::HOGDescriptor hog;
//		std::vector<float> descriptors;
//		hog.compute(found_img, descriptors, cv::Size(8, 8), cv::Size(0, 0));
//		cv::Mat hog_vizimg = Get_Hogdescriptor_Visual(found_img, descriptors, cvSize(64, 128), cvSize(8,8), 2, 2);
//
//		imshow("hog_viz", hog_vizimg);
//		cv::rectangle(img, tl, br, cv::Scalar(64, 10, 64), 3);
    }
}
