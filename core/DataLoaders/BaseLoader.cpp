//
// Created by root on 5/16/21.
//

#include "BaseLoader.h"

#include <memory>

using namespace std;
using namespace boost::filesystem;

namespace OG_SLAM {

    BaseLoader::BaseLoader(const DS_ParamsPtr &pDsParams) :
            mLoadState(TabularTextDS::LoadState::BAD_PATH), mDsFormat(pDsParams->mFormat), mDsName(pDsParams->mName),
            mSeqCount(0), mSeqTarget(0), mSeqIdx(0), mnMaxIter(pDsParams->mnMaxIter), mTsFactor(pDsParams->mfTsFactor) {

        // Check important paths
        if (checkDatasetPaths(pDsParams)) {

            // If everything is good, load Data Stores
            this->loadData();
        }
        // Maybe data paths are wrong so check loader integrity
        this->updateLoadState();
    }

    bool BaseLoader::checkDatasetPaths(const DS_ParamsPtr &pDsParams) {

        // Check DS root path
        mPathDsRoot = pDsParams->getDatasetRoot();
        if (!TabularTextDS::checkDirectory(mPathDsRoot)) {
            mLoadState = TabularTextDS::BAD_PATH;
            return false;
        }

        // Assign relative sequence data paths
        mPathImFile = pDsParams->getImageFilePath();
        mPathImBase = pDsParams->getImageBasePath();
        mPathImu = pDsParams->getIMU_Path();
        mPathGT = pDsParams->getGroundTruthPosePath();

        // Resolve and check sequence paths
        if (!this->resolveSeqInfo(pDsParams)) {
            mLoadState = TabularTextDS::BAD_PATH;
            return false;
        }

        // Resolve output trajectory path
        this->resolveOutputBasePath(pDsParams);

        mLoadState = TabularTextDS::READY;
        return true;
    }

    /**
     * This will set 3 internal class variables:
     *  mSeqTarget, mSeqCount, mSeqNames
     * @param pDsParams
     * @return
     */
    bool BaseLoader::resolveSeqInfo(const DS_ParamsPtr &pDsParams) {

        mSeqTarget = pDsParams->mnSeqTarget;

        vector<string> seqNames = pDsParams->mvSeqNames;
        unsigned int seqCount = seqNames.size();

        if (seqCount) {
            // Check target sequence path(s) exist.
            if (mSeqTarget >= 0) {
                if (mSeqTarget < seqCount) {

                    string seqPath = mPathDsRoot + '/' + seqNames[mSeqTarget];
                    if (!TabularTextDS::checkDirectory(seqPath)) {
                        LOG(ERROR) << "** Failed to find sequence: " << seqPath << endl;
                        return false;
                    }
                    else {
                        mSeqCount = 1;
                        mSeqNames.resize(mSeqCount);
                        mSeqNames[0] = seqNames[mSeqTarget];
                        mSeqTarget = 0;
                        mSeqIdx = 0;
                    }
                }
                else {
                    LOG(ERROR) << "** Target sequence number is outside range.\n";
                    return false;
                }
            }
            else {
                // If some sequences does not exist, we still want to
                // be able to work with existing sequences
                for (size_t seq = 0; seq < seqCount; seq++) {
                    string seqPath = mPathDsRoot + '/' + seqNames[seq];
                    if (!TabularTextDS::checkDirectory(seqPath)) {
                        LOG(ERROR) << "** Failed to find sequence: " << seqPath << endl;
                    }
                    else {
                        mSeqNames.push_back(seqNames[seq]);
                    }
                }
                mSeqCount = mSeqNames.size();
                if (!mSeqCount)
                    return false;
                mSeqIdx = 0;
            }
        }
        else {
            LOG(ERROR) << "** Empty sequence names.\n";
            return false;
        }
        return true;
    }

    void BaseLoader::resolveOutputBasePath(const DS_ParamsPtr &pDsParams) {

        mPathOutBase = DS_Params::mapDsFormats(mDsFormat) + '_' + pDsParams->mpSensorConfig->toDsStr();

        if (!mSeqNames.empty() && mSeqTarget >= 0 && mSeqTarget < mSeqNames.size()) {
            mPathOutBase += '_' + mSeqNames[mSeqTarget];
        }
    }

    bool BaseLoader::checkLoadState() {

        unsigned imDsSize = mvpImDs.size();
        unsigned imuDsSize = mvpImuDs.size();
        unsigned gtDsSize = mvpGtDs.size();

        bool cond = imDsSize == imuDsSize && imDsSize == gtDsSize && imDsSize > 0;
        if (cond) {
            mSeqCount = imDsSize;
            mLoadState = TabularTextDS::GOOD;
        }
        else {
            mSeqCount = 0;
            mLoadState = TabularTextDS::BAD_DATA;
        }
        return cond;
    }

    bool BaseLoader::updateLoadState() {

        return this->checkLoadState();
    }

    void BaseLoader::loadData() {

        if (mSeqTarget < 0) {
            mSeqCount = mSeqNames.size();
        }
        else {
            mSeqCount = 1;
        }

        if (mSeqCount) {

            mvpImDs.resize(mSeqCount);
            mvpImuDs.resize(mSeqCount);
            mvpGtDs.resize(mSeqCount);

            if (mSeqTarget < 0) {
                for (size_t seq = 0; seq < mSeqCount; seq++) {
                    loadSequence(mPathDsRoot, mSeqNames[seq], seq);
                }
            }
            else if (mSeqTarget < mSeqNames.size()) {
                loadSequence(mPathDsRoot, mSeqNames[mSeqTarget], 0);
            }
        }
    }

    // Deal with timestamp units from the beginning
    void BaseLoader::loadSequence(const string &dsRoot, const string &sqPath, const size_t idx) {

        string seqPath = dsRoot + '/' + sqPath + '/';
        this->mvpImDs[idx] = std::make_unique<ImageDS>(seqPath + mPathImFile, seqPath + mPathImBase, mTsFactor);
        this->mvpImuDs[idx] = std::make_unique<IMU_DS>(seqPath + mPathImu, true, mTsFactor);
        this->mvpGtDs[idx] = std::make_unique<PoseDS>(seqPath + mPathGT, true, true, mTsFactor);
    }

    string BaseLoader::getSequencePath() {

        if (mSeqNames.empty() || mSeqTarget < 0 || mSeqTarget >= mSeqNames.size())
            return string();
        return mPathDsRoot + '/' + mSeqNames[mSeqTarget];
    }

    void BaseLoader::resetSequences() {
        if (this->mSeqTarget < 0)
            this->mSeqIdx = 0;
    }

    void BaseLoader::incSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx < mSeqCount-1)
            this->mSeqIdx++;
    }

    void BaseLoader::decSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx > 1)
            this->mSeqIdx--;
    }

    bool BaseLoader::checkSequence(const unsigned int seq) const {

        if (mLoadState == TabularTextDS::GOOD) {
            if (mSeqTarget < 0) {
                return seq >= 0 && seq < mSeqCount;
            } else {
                return seq == 0;
            }
        }
        return false;
    }

    unsigned int BaseLoader::getNumTargetSequences() const {

        if (mSeqTarget < 0)
            return this->getNumSequences();
        else {
            if (checkSequence(mSeqIdx))
                return 1;
            else
                return 0;
        }
    }

    string BaseLoader::getSequenceName() const {

        if (!checkSequence(mSeqIdx))
            return string();
        return mSeqNames[mSeqIdx];
    }

    bool BaseLoader::isGood() const {

        return mLoadState == TabularTextDS::GOOD;
    }

    void BaseLoader::resetCurrSequence() {

        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->reset();
        this->mvpImuDs[mSeqIdx]->reset();
        this->mvpGtDs[mSeqIdx]->reset();
    }

    unsigned int BaseLoader::getNumImages() {
        if (!checkSequence(mSeqIdx))
            return 0;
        if (this->mvpImDs.empty() || !this->mvpImDs[mSeqIdx])
            return 0;
        return this->mvpImDs[mSeqIdx]->getNumFiles();
    }

    void BaseLoader::getImage(const size_t idx, cv::Mat &image, double &ts) {
        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->getImage(idx, image, ts);
    }

    void BaseLoader::getImage(const size_t idx, cv::Mat &image, double &ts, string& imPath) {
        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->getImage(idx, image, ts, imPath);
    }

    string BaseLoader::getImageFileName(const size_t idx, bool fullName) {
        if (!checkSequence(mSeqIdx))
            return string();
        return this->mvpImDs[mSeqIdx]->getFileName(idx, fullName);
    }

    double BaseLoader::getImageTime(size_t idx) {
        if (!checkSequence(mSeqIdx))
            return 0.0;
        return this->mvpImDs[mSeqIdx]->getTimeStamp(idx);
    }

    unsigned int BaseLoader::getNumTotalImages() {

        unsigned int numIm = 0;
        if (mSeqTarget < 0) {
            for (auto & mvpImD : mvpImDs) {
                numIm += mvpImD->getNumFiles();
            }
        }
        else {
            numIm = this->getNumImages();
        }
        return numIm;
    }

    unsigned int BaseLoader::getNextImu(const double ts, vector<IMU_DataPtr> &vpImuData) {

        if (!checkSequence(mSeqIdx))
            return 0;
        return this->mvpImuDs[mSeqIdx]->getNextChunk(ts, vpImuData);
    }

    unsigned int BaseLoader::getNextPoseGT(const double ts, vector<PosePtr> &vpPoseGT) {

        if (!checkSequence(mSeqIdx))
            return 0;
        return this->mvpGtDs[mSeqIdx]->getNextChunk(ts, vpPoseGT);
    }

    void BaseLoader::addImageHook(ImageHookPtr &pImageHook) {

        mvpImageHooks.push_back(pImageHook);
    }

    void BaseLoader::addIMU_Hook(IMU_HookPtr &pImuHook) {

        mvpIMU_Hooks.push_back(pImuHook);
    }

    void BaseLoader::addGT_PoseHook(PoseHookPtr &pPoseHook) {

        mvpGtPoseHooks.push_back(pPoseHook);
    }

    void BaseLoader::play() {

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
        }
    }

} // OG_SLAM