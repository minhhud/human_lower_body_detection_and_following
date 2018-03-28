#ifndef __SVMLIGHTLIB_H__
#define __SVMLIGHTLIB_H__

#include <vector>
#include <string>
#include <fstream>

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
}

#endif
