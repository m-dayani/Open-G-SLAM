//
// Created by root on 5/16/21.
//

#ifndef OG_SLAM_BASELOADER_H
#define OG_SLAM_BASELOADER_H

#include <vector>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

#include "DS_Params.h"
#include "ImageDS.h"
#include "ImageHook.h"
#include "IMU_DS.h"
#include "IMU_Hook.h"
#include "PoseDS.h"
#include "PoseHook.h"


namespace OG_SLAM {

    class BaseLoader {
    public:
        explicit BaseLoader(const DS_ParamsPtr& pDsParams);
        virtual ~BaseLoader() = default;

        virtual void addImageHook(ImageHookPtr& pImageHook);
        virtual void addIMU_Hook(IMU_HookPtr& pImuHook);
        virtual void addGT_PoseHook(PoseHookPtr& pPoseHook);

        // default play: Image-based!
        virtual void play();

        //Getters/Setters
        std::string getDatasetName() const { return mDsName; }
        unsigned int getNumSequences() const { return mSeqCount; }
        unsigned int getNumTargetSequences() const;
        unsigned int getMaxNumIter() const { return mnMaxIter; }
        std::string getSequenceName() const;
        std::string getOutputBasePath() const { return mPathOutBase; };

        //Utils
        virtual bool isGood() const;
        virtual void resetSequences();
        virtual void resetCurrSequence();

        void incSequence();
        void decSequence();

        virtual bool checkSequence(unsigned int seq) const;

        //Image
        unsigned int getNumImages();
        unsigned int getNumTotalImages();
        void getImage(size_t idx, cv::Mat &image, double &ts);
        void getImage(size_t idx, cv::Mat &image, double &ts, std::string& imPath);
        double getImageTime(size_t idx);
        std::string getImageFileName(size_t idx, bool fullName = true);

        //IMU
        // initTs is in (sec)
        //void initImu(double initTs, int idx = -1);
        unsigned int getNextImu(double ts, std::vector<IMU_DataPtr> &vpImuData);

        //GT
        unsigned int getNextPoseGT(double ts, std::vector<PosePtr>& vpPose);

    protected:
        bool checkDatasetPaths(const DS_ParamsPtr& pDsParams);
        virtual bool resolveSeqInfo(const DS_ParamsPtr& pDsParams);
        virtual void resolveOutputBasePath(const DS_ParamsPtr& pDsParams);

        std::string getSequencePath();

        virtual void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx);
        virtual bool checkLoadState();

        void loadData();
        bool updateLoadState();

    protected:
        // Data Stores
        std::vector<ImageDS_UPtr> mvpImDs;
        std::vector<IMU_DS_UPtr> mvpImuDs;
        std::vector<PoseDS_UPtr> mvpGtDs;

        // Hooks
        std::vector<ImageHookPtr> mvpImageHooks;
        std::vector<IMU_HookPtr> mvpIMU_Hooks;
        std::vector<PoseHookPtr> mvpGtPoseHooks;

        TabularTextDS::LoadState mLoadState;
        DS_Params::DS_Formats mDsFormat;

        std::string mDsName;

        //std::string mPathSettings;
        std::string mPathDsRoot;
        std::string mPathOutBase;
        std::string mPathImFile;
        std::string mPathImBase;
        std::string mPathImu;
        std::string mPathGT;

        // Sequence count counts the size of active dataStores (like mImDs)
        unsigned int mSeqCount;
        int mSeqTarget;
        std::vector<std::string> mSeqNames;
        // If mSeqCount == 1, mSeqIdx must always be zero
        unsigned int mSeqIdx;

        unsigned int mnMaxIter;

        double mTsFactor;

        //std::shared_ptr<MyCalibrator> mpCalib;

    private:

    };

} // OG_SLAM


#endif //OG_SLAM_BASELOADER_H
