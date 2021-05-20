//
// Created by root on 5/17/21.
//

#ifndef OG_SLAM_IMAGEFILTER_H
#define OG_SLAM_IMAGEFILTER_H

#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "Image.h"


namespace OG_SLAM {

    class ImageFilter {
    public:
        virtual void filter(ImagePtr& pImage) {}
    };

    typedef std::shared_ptr<ImageFilter> ImageFilterPtr;

    class ImageFilterGray : public ImageFilter {
    public:
        explicit ImageFilterGray(const bool colorOrder) : mbRGB(colorOrder) {}

        void filter(ImagePtr& pImage) override;

    private:
        // Color Order: true: RGB, false: BGR
        const bool mbRGB;
    };

    // class ImageFilterRectify

} // OG_SLAM


#endif //OG_SLAM_IMAGEFILTER_H
