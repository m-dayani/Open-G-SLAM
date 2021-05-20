//
// Created by root on 5/13/21.
//

#ifndef OG_SLAM_FEATURES2D_H
#define OG_SLAM_FEATURES2D_H

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

#include "YamlParserCV.h"


namespace OG_SLAM {

    struct ORB_Params {

        ORB_Params();
        explicit ORB_Params(const cv::FileNode& orbNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& orbNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        std::string mPathVocab;

        int mnFeatures;
        int mnLevels;
        float mfScaleFactor;
        int mnMinThFAST, mnIniThFAST;
    };

    typedef std::shared_ptr<ORB_Params> ORB_ParamsPtr;

    struct AKAZE_Params {

        AKAZE_Params();
        explicit AKAZE_Params(const cv::FileNode& akazeNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& akazeNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        int mnFeatures;
        int mnOctaves, mnOctaveLayers;
        float mfScaleFactor;
        float mfIniTh, mfMinTh;
    };

    typedef std::shared_ptr<AKAZE_Params> AKAZE_ParamsPtr;

    struct Ft2D_Params {

        enum FtDtModes {
            NONE,
            ORB,
            AKAZE,
            MIXED
        };

        Ft2D_Params();
        explicit Ft2D_Params(const cv::FileStorage& fSettings);

        static FtDtModes mapFtDtMode(const std::string& ftDtMode);
        static std::string mapFtDtMode(FtDtModes ftDtMode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& ftNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        FtDtModes mFtDtMode;

        ORB_ParamsPtr mpOrbParams;
        AKAZE_ParamsPtr mpAkazeParams;
    };

    typedef std::shared_ptr<Ft2D_Params> Ft2D_ParamsPtr;

} // OG_SLAM


#endif //OG_SLAM_FEATURES2D_H
