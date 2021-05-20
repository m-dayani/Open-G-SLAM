//
// Created by root on 5/13/21.
//

#ifndef OG_SLAM_IMU_PARAMS_H
#define OG_SLAM_IMU_PARAMS_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "YamlParserCV.h"


namespace OG_SLAM {

    struct IMU_Params {

        IMU_Params();
        explicit IMU_Params(const cv::FileStorage& fSettings);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& imuNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        bool missParams;

        float freq;
        float Ng, Na, Ngw, Naw;

        float sf;
        //cv::Mat Tbs;
    };

    typedef std::shared_ptr<IMU_Params> IMU_ParamsPtr;

} // OG_SLAM


#endif //OG_SLAM_IMU_PARAMS_H
