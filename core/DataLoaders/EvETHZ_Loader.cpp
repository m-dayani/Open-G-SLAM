//
// Created by root on 5/21/21.
//

#include "EvETHZ_Loader.h"

using namespace std;

namespace OG_SLAM {


    EvETHZ_Loader::EvETHZ_Loader(const DS_ParamsPtr &pDsParams) : EurocLoader(pDsParams) {

        // Check dataset format
        if (mDsFormat != DS_Params::EV_ETHZ) {
            LOG(ERROR) << "** Initializing EvETHZ loader with " << DS_Params::mapDsFormats(mDsFormat) << " data!\n";
            mLoadState = TabularTextDS::BAD_DATA;
            return;
        }

        mPathEvents = pDsParams->getEventsPath();

        this->loadData();
        this->updateLoadState();
    }

    void EvETHZ_Loader::addEventHook(const EventHookPtr &pEventHook) {

        mvpEventHooks.push_back(pEventHook);
    }

    void EvETHZ_Loader::play() {

        if (mLoadState != TabularTextDS::GOOD) {
            LOG(ERROR) << "** Loader::play: Load State is not good, abort\n";
            return;
        }

        uint nImages = this->getNumImages();

        // Loop through images
        for (uint i = 0; i < nImages; i++) {

            // Get Data
            // Image
            double ts = 0.0;
            cv::Mat image;
            string imPath;
            this->getImage(i, image, ts, imPath);
            ImagePtr imData = make_shared<ImageTs>(image, ts, imPath);

            // IMU
            vector<IMU_DataPtr> vpImuData;
            this->getNextImu(ts, vpImuData);

            // GT
            vector<PosePtr> vpPose;
            this->getNextPoseGT(ts, vpPose);

            // Events
            vector<EvDataPtr> vpEvData;
            this->getNextEvents(ts, vpEvData);

            // Dispatch Data
            for (ImageHookPtr& imHook : mvpImageHooks) {
                imHook->dispatch(imData);
            }

            for (const IMU_DataPtr& pImuData : vpImuData) {
                for (IMU_HookPtr& pImuHook : mvpIMU_Hooks) {
                    pImuHook->dispatch(pImuData);
                }
            }

            for (const PosePtr& pPose : vpPose) {
                for (PoseHookPtr& pGtPoseHook : mvpGtPoseHooks) {
                    pGtPoseHook->dispatch(pPose);
                }
            }

            for (EventHookPtr& pEvHook : mvpEventHooks) {
                pEvHook->dispatch(vpEvData);
            }
        }
    }

    void EvETHZ_Loader::resetCurrSequence() {

        EurocLoader::resetCurrSequence();

        if (!checkSequence(mSeqIdx))
            return;
        this->mvpEventDS[mSeqIdx]->reset();
    }

    unsigned long
    EvETHZ_Loader::getNextEvents(unsigned long chunkSize, std::vector<EvDataPtr> &evs, bool undistPoints, bool checkInImage) {

        if (!checkSequence(mSeqIdx)) {
            LOG(ERROR) << "EvEthzLoader::getNextEvents: Wrong Sequence number\n";
            return 0;
        }

        if (undistPoints) {
//            if (mpCalib) {
//                return mvpEventDS[mSeqIdx]->getEventChunkRectified(chunkSize, evs, mpCalib, checkInImage);
//            }
//            else {
                LOG(ERROR) << "EvEthzLoader::getNextEvents: Cannot find calibrator!\n";
                return 0;
//            }
        }
        else {
            return mvpEventDS[mSeqIdx]->getEventChunk(chunkSize, evs);
        }
    }

    unsigned long
    EvETHZ_Loader::getNextEvents(double tsEnd, std::vector<EvDataPtr> &evs, bool undistPoints, bool checkInImage) {

        if (!checkSequence(mSeqIdx)) {
            LOG(ERROR) << "EvEthzLoader::getNextEvents: Wrong Sequence number\n";
            return 0;
        }

        if (undistPoints) {
//            if (mpCalib) {
//                return mvpEvDs[mSeqIdx]->getEventChunkRectified(tsEnd, evs, mpCalib, checkInImage);
//            }
//            else {
                LOG(ERROR) << "EvEthzLoader::getNextEvents: Cannot find calibrator!\n";
                return 0;
//            }
        }
        else {
            return mvpEventDS[mSeqIdx]->getEventChunk(tsEnd, evs);
        }
    }

    void EvETHZ_Loader::loadSequence(const std::string &dsRoot, const std::string &sqPath, size_t idx) {

        EurocLoader::loadSequence(dsRoot, sqPath, idx);

        if (mvpEventDS.empty()) {
            mvpEventDS.resize(mSeqCount);
        }

        string seqPath = dsRoot + '/' + sqPath + '/';
        mvpEventDS[idx] = std::make_unique<EventDS>(seqPath + mPathEvents, mTsFactor);
    }

    bool EvETHZ_Loader::checkLoadState() {

        bool res = EurocLoader::checkLoadState();
        if (res) {
            res &= mvpEventDS.size() == mvpImDs.size();
        }
        if (res) {
            mSeqCount = mvpImDs.size();
            mLoadState = TabularTextDS::GOOD;
        }
        else {
            mSeqCount = 0;
            mLoadState = TabularTextDS::BAD_DATA;
        }
        return res;
    }
} // OG_SLAM
