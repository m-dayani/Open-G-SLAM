//
// Created by root on 5/17/21.
//

#include "ImageFilter.h"


namespace OG_SLAM {

    void ImageFilterGray::filter(ImagePtr &pImage) {

        cv::Mat mImGray = pImage->mImage;

        if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        }

        mImGray.copyTo(pImage->mImage);
    }

} // OG_SLAM