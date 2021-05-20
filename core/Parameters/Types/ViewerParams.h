//
// Created by root on 5/13/21.
//

#ifndef OG_SLAM_VIEWERPARAMS_H
#define OG_SLAM_VIEWERPARAMS_H

#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include "YamlParserCV.h"


namespace OG_SLAM {

    struct ViewerParams {

        ViewerParams();
        explicit ViewerParams(const cv::FileStorage& fsSettings);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& viewerNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        //bool missParams = false;
        bool mbUseViewer;

        // 1/fps in ms
        //double mT;
        //float mImageWidth, mImageHeight;
        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;
    };

    typedef std::shared_ptr<ViewerParams> ViewerParamsPtr;

} // OG_SLAM


#endif //OG_SLAM_VIEWERPARAMS_H
