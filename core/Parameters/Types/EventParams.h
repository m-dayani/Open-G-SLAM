//
// Created by root on 5/14/21.
//

#ifndef OG_SLAM_EVENTPARAMS_H
#define OG_SLAM_EVENTPARAMS_H

#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>

#include "YamlParserCV.h"


namespace OG_SLAM {

    struct EvDataParams {

        EvDataParams();
        explicit EvDataParams(const cv::FileNode& dataNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& dataNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        bool mbRectified;

        bool mbL1FixedWin;
        int mnL1ChunkSize;
        int mnL1NumLoop;

        float mfMinEvGenRate, mfMaxEvGenRate;
        float mfMaxPxDisp;
    };

    typedef std::shared_ptr<EvDataParams> EvDataParamsPtr;

    struct EvAlgoParams {

        enum L2TrackingMode {
            ODOMETERY,
            TLM,
            TLM_CHR
        };

        EvAlgoParams();
        explicit EvAlgoParams(const cv::FileNode& dataNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& evAlgoNode);

        static L2TrackingMode mapTrackMode(const std::string& trackMode);
        static std::string mapTrackMode(L2TrackingMode trackMode);

        std::string printStr(const std::string& prefix = std::string()) const;

        L2TrackingMode mTrackMode;

        bool mbTrackTinyFrames;

        float mfL1ImSigma, mfL2ImSigma;
    };

    typedef std::shared_ptr<EvAlgoParams> EvAlgoParamsPtr;

    struct EvFtParams {

        EvFtParams();
        explicit EvFtParams(const cv::FileNode& dataNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& evFtNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        int mnFtDtMode;
        int mnFeatures;
        int mnThFAST;
        int mnL1NumLevels, mnL2NumLevels;
        float mfL1ScaleFactor, mfL2ScaleFactor;
    };

    typedef std::shared_ptr<EvFtParams> EvFtParamsPtr;

    struct EvKLT_Params {

        EvKLT_Params();
        explicit EvKLT_Params(const cv::FileNode& dataNode);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& ekltNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        int mnMaxLevel;
        int mnWinSize;
        float mfEps;
        int mnMaxIter;
        float mfThRefreshPts;
        float mfThRefreshPtsDist;
    };

    typedef std::shared_ptr<EvKLT_Params> EvKLT_ParamsPtr;

    struct EventParams {

        //EventParams();
        explicit EventParams(const cv::FileStorage& fsSettings);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& eventNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        EvDataParamsPtr mpDataParams;
        EvAlgoParamsPtr mpAlgoParams;
        EvFtParamsPtr mpEvFtParams;
        EvKLT_ParamsPtr mpEKLT_Params;
    };

    typedef std::shared_ptr<EventParams> EventParamsPtr;

} // OG_SLAM

#endif //OG_SLAM_EVENTPARAMS_H
