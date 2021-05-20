//
// Created by root on 5/15/21.
//

#ifndef OG_SLAM_IMAGE_H
#define OG_SLAM_IMAGE_H

#include <string>
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

#include "SharedQueue.h"


namespace OG_SLAM {

    struct Image {

        Image(const cv::Mat& image, std::string  imPath) : mImage(image.clone()), mPath(std::move(imPath)) {}
        virtual ~Image() = default;

        cv::Mat mImage;
        std::string mPath;
    };

    typedef std::shared_ptr<Image> ImagePtr;

    struct ImageTs : public Image {

        ImageTs(const cv::Mat& image, double ts, const std::string& imPath) :
                Image(image, imPath), mTimeStamp(ts) {}

        double mTimeStamp;
    };

    //typedef std::shared_ptr<ImageTs> ImageTsPtr;

    typedef SharedQueue<ImagePtr> ImageQueue;
    typedef std::shared_ptr<ImageQueue> ImageQueuePtr;

} // OG_SLAM


#endif //OG_SLAM_IMAGE_H
