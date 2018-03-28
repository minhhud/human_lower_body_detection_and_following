#ifndef DETECT_HOG_HH
#define DETECT_HOG_HH

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <ios>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <dirent.h>
#include <omp.h>

namespace SVMLight
{
    class SVMTrainer
    {
    private:
        std::fstream featuresFile_;
        std::string featuresFileName_;
    public:
        SVMTrainer(const std::string& featuresFileName);
        void writeFeatureVectorToFile(const std::vector<float>& featureVector, bool isPositive);
        void trainAndSaveModel(const std::string& modelFileName);
    };

    class SVMClassifier
    {
    public:
        SVMClassifier(const std::string& modelFileName);
        std::vector<float> getDescriptorVector();
    };
};

namespace hog_detector
{
	class Trainer
	{
	public:
		Trainer(std::string featuresFile, std::string svmModelFile, std::string pos_dir, std::string neg_dir);
		~Trainer();
	};

	class Detector
	{
	public:
		Detector(std::string svmModelFile);
		~Detector();
		void Set_SVM();
		bool Classify_HOG_SVM(cv::Mat& img_data, cv::Mat& img_viz, cv::Rect rect, int groupThreshold=4, cv::Size winStride=cv::Size(8, 8), cv::Size trainingPadding=cv::Size(0, 0), double scale=1.05, double hitThreshold=0.0);
		void Show_Detections(cv::Mat& img, cv::Rect rect);

		cv::HOGDescriptor hog_;
		std::vector<cv::Rect> found_;
		std::string svmModelFile_;
		std::vector<float> descriptorVector_;
	};
};
#endif
