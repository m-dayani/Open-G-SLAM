//
// Created by root on 5/10/21.
//

#ifndef OG_SLAM_DS_PARAMS_H
#define OG_SLAM_DS_PARAMS_H

#include <memory>

#include "SensorConfig.h"
#include "YamlParserCV.h"


namespace OG_SLAM {

    struct DS_Params {

        enum DS_Formats {
            NOT_SUPPORTED,
            EUROC,
            EV_ETHZ,
            EV_MVSEC
        };

        DS_Params();
        explicit DS_Params(const cv::FileStorage& fsSettings);

        static DS_Formats mapDsFormats(const std::string& dsFormat);
        static std::string mapDsFormats(DS_Formats dsFormat);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);

        // This is absolute or relative to execution path
        std::string getDatasetRoot();
        // All of these are relative to each sequence (root+seq_name)
        std::string getImageFilePath();
        std::string getImageBasePath();
        std::string getIMU_Path();
        std::string getGroundTruthPosePath();
        std::string getEventsPath();
        // This is relative to execution path
        std::string getResultsBasePath();

        std::string printStr(const std::string& prefix = "");

        std::string mName;
        DS_Formats mFormat;
        SensorConfigPtr mpSensorConfig;

        std::map<std::string, std::string> mmPaths;

        int mnSeqTarget;
        std::vector<std::string> mvSeqNames;

        int mnMaxIter;
        double mfTsFactor;

        bool mbImuGyroFirst;
        bool mbGtQwFirst;
        bool mbGtPosFirst;
    };

    typedef std::shared_ptr<DS_Params> DS_ParamsPtr;

} // OG_SLAM

#endif //OG_SLAM_DS_PARAMS_H
