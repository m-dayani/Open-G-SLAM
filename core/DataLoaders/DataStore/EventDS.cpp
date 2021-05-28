//
// Created by root on 5/22/21.
//

#include "EventDS.h"

using namespace std;


namespace OG_SLAM {

    EventDS::EventDS(const string &evPath, const double tsFactor) : mLastEvTs(0.0)
    {
        boost::filesystem::path p(evPath);  // avoid repeated path construction below
        mTsFactor = tsFactor;

        try {
            if (exists(p))    // does path p actually exist?
            {
                this->mDataPath = evPath;
                if (is_regular_file(p))        // is path p a regular file?
                {
                    mPathState = PathState::FILE;
                    if (checkExtension(p, ".txt")) {
                        mLoadState = LoadState::READY;
                    }
                    else {
                        mLoadState = LoadState::BAD_PATH;
                    }
                }
                else if (is_directory(p))      // is path p a directory?
                {
                    mPathState = PathState::DIR;

                    for (auto&& x : boost::filesystem::directory_iterator(p))
                    {
                        const boost::filesystem::path& evFilePath = x.path();
                        if (checkExtension(evFilePath, ".txt"))
                        {
                            mvFileNames.push_back(evFilePath.string());
                            mnFiles++;
                        }
                    }
                    if (mnFiles > 0) {
                        mLoadState = LoadState::READY;
                        std::sort(mvFileNames.begin(), mvFileNames.end());
                    }
                    else {
                        mLoadState = LoadState::BAD_PATH;
                    }
                }
                else
                    cout << p << " exists, but is not a regular file or directory\n";

                this->loadTxtFile();
            }
            else {
                cerr << p << " does not exist\n";
            }
        }
        catch (const boost::filesystem::filesystem_error& ex)
        {
            cerr << ex.what() << '\n';
        }

    }

    EventDS::~EventDS()
    {
        if (mTxtDataFile.is_open())
        {
            mTxtDataFile.close();
        }
    }

    boost::any EventDS::parseLine(const string &evStr) {

        std::istringstream stream(evStr);

        double ts;
        float x, y;
        bool p;

        stream >> ts >> x >> y >> p;

        EvDataPtr evData = make_shared<EventData>(ts/mTsFactor, x, y, p);
        return boost::any(evData);
    }

    /*EventData EventDS::parseLine(const std::string &evStr, const cv::Mat& K,
                                        const cv::Mat& distCoefs, const cv::Mat& rectMat) {

        std::istringstream stream(evStr);

        double ts;
        float x, y;
        bool p;

        stream >> ts >> x >> y >> p;

        cv::Point2f evPt(x, y);
        MyCalibrator::undistPointPinhole(evPt, evPt, K, distCoefs, rectMat);

        return {ts/mTsFactor, evPt.x, evPt.y, p};
    }

    EventData EventDS::parseLine(const std::string &evStr, const MyCalibPtr& pCalib) {

        std::istringstream stream(evStr);

        double ts;
        float x, y;
        bool p;

        stream >> ts >> x >> y >> p;

        cv::Point2f evPt(x, y);
        pCalib->undistPointMaps(evPt, evPt);

        return {ts/mTsFactor, evPt.x, evPt.y, p};
    }*/

    unsigned long EventDS::getEventChunk(const unsigned long chunkSize, vector<EvDataPtr> &evBuffer) {

        return getTxtData(chunkSize, evBuffer);
    }

    unsigned long EventDS::getEventChunk(const double tsEnd, vector<EvDataPtr> &evBuffer) {

        if (!this->mTxtDataFile.is_open()) {

            LOG(ERROR) << "Text data file is not open\n";
            return 0;
        }

        string line;

        unsigned long dtCount = 0;

        //If data manager constantly supply the same outData,
        // all data will be stacked together.
        if (tsEnd <= mLastEvTs) {

            LOG(WARNING) << "Same event timestamp supplied: " << tsEnd << endl;
            return 0;
        }

        evBuffer.reserve(DEF_L1_CHUNK_SIZE * DEF_L1_NUM_LOOP);

        while (this->checkTxtStream())
        {
            getline(this->mTxtDataFile, line);
            this->mnCurrByteIdx += line.length();

            if (isComment(line))
                continue;

            EvDataPtr currEv = boost::any_cast<EvDataPtr>(this->parseLine(line));
            evBuffer.push_back(currEv);

            dtCount++;

            if (currEv->ts >= tsEnd) {
                mLastEvTs = currEv->ts;
                break;
            }
        }
        return dtCount;
    }

    /*unsigned long EventDS::getEventChunkRectified(unsigned long chunkSize, vector<EventData> &evBuffer,
                                                         const cv::Mat& K, const cv::Mat& distCoefs, const cv::Mat& rectMat,
                                                         const cv::Scalar& imageSize, const bool checkInImage) {

        if (!this->mTxtDataFile.is_open()) {

            LOG(ERROR) << "Text data file is not open\n";
            return 0;
        }

        string line;

        unsigned long dtCount = 0;

        //If data manager constantly supply the same outData,
        // all data will be stacked together.
        if (chunkSize > 0)
            evBuffer.reserve(chunkSize);

        while (this->checkTxtStream())
        {
            if (chunkSize > 0 && dtCount >= chunkSize)
                break;

            getline(this->mTxtDataFile, line);
            this->mnCurrByteIdx += line.length();

            if (isComment(line))
                continue;

            EventData currEv = this->parseLine(line, K, distCoefs, rectMat);

            if (checkInImage && !MyCalibrator::isInImage(currEv.x, currEv.y, imageSize))
                continue;

            evBuffer.push_back(currEv);

            dtCount++;
        }
        return dtCount;
    }

    unsigned long EventDS::getEventChunkRectified(double tsEnd, vector<EventData> &evBuffer,
                                                         const cv::Mat& K, const cv::Mat& distCoefs, const cv::Mat& rectMat,
                                                         const cv::Scalar& imageSize, const bool checkInImage) {

        if (!this->mTxtDataFile.is_open()) {

            LOG(ERROR) << "Text data file is not open\n";
            return 0;
        }

        string line;

        unsigned long dtCount = 0;

        //If data manager constantly supply the same outData,
        // all data will be stacked together.
        if (tsEnd <= mLastEvTs) {
            LOG(WARNING) << "Same event timestamp supplied: " << tsEnd << endl;
            return 0;
        }
        evBuffer.reserve(DEF_L1_CHUNK_SIZE * DEF_L1_NUM_LOOP);

        while (this->checkTxtStream())
        {
            getline(this->mTxtDataFile, line);
            this->mnCurrByteIdx += line.length();

            if (isComment(line))
                continue;

            EventData currEv = this->parseLine(line, K, distCoefs, rectMat);

            if (checkInImage && !MyCalibrator::isInImage(currEv.x, currEv.y, imageSize))
                continue;

            evBuffer.push_back(currEv);

            dtCount++;

            if (currEv.ts >= tsEnd) {
                mLastEvTs = currEv.ts;
                break;
            }
        }
        return dtCount;
    }

    unsigned long EventDS::getEventChunkRectified(unsigned long chunkSize, vector<EventData> &evBuffer,
                                                         const MyCalibPtr& pCalib, const bool checkInImage) {

        if (!this->mTxtDataFile.is_open()) {

            LOG(ERROR) << "Text data file is not open\n";
            return 0;
        }

        string line;

        unsigned long dtCount = 0;

        //If data manager constantly supply the same outData,
        // all data will be stacked together.
        if (chunkSize > 0)
            evBuffer.reserve(chunkSize);

        while (this->checkTxtStream())
        {
            if (chunkSize > 0 && dtCount >= chunkSize)
                break;

            getline(this->mTxtDataFile, line);
            this->mnCurrByteIdx += line.length();

            if (isComment(line))
                continue;

            EventData currEv = this->parseLine(line, pCalib);

            if (checkInImage && !pCalib->isInImage(currEv.x, currEv.y))
                continue;

            evBuffer.push_back(currEv);

            dtCount++;

            //DLOG_EVERY_N(INFO, 1000) << "Current ev. ts: " << currEv.ts << "\n";
        }
        return dtCount;
    }

    unsigned long EventDS::getEventChunkRectified(double tsEnd, vector<EventData> &evBuffer,
                                                         const MyCalibPtr& pCalib, const bool checkInImage) {

        if (!this->mTxtDataFile.is_open()) {

            LOG(ERROR) << "Text data file is not open\n";
            return 0;
        }

        string line;

        unsigned long dtCount = 0;

        //If data manager constantly supply the same outData,
        // all data will be stacked together.
        if (tsEnd <= mLastEvTs) {
            LOG(WARNING) << "Same event timestamp supplied: " << tsEnd << endl;
            return 0;
        }
        evBuffer.reserve(DEF_L1_CHUNK_SIZE * DEF_L1_NUM_LOOP);

        while (this->checkTxtStream())
        {
            getline(this->mTxtDataFile, line);
            this->mnCurrByteIdx += line.length();

            if (isComment(line))
                continue;

            EventData currEv = this->parseLine(line, pCalib);

            if (checkInImage && !pCalib->isInImage(currEv.x, currEv.y))
                continue;

            evBuffer.push_back(currEv);

            dtCount++;

            if (currEv.ts >= tsEnd) {
                mLastEvTs = currEv.ts;
                break;
            }
        }
        return dtCount;
    }*/

    void EventDS::reset() {

        if (mLoadState != GOOD && mLoadState != READY) {
            LOG(WARNING) << "EventDS::reset(): Load stat is not good.\n";
            return;
        }

        mLastEvTs = 0.0;
        mnCurrByteIdx = 0;
        mnCurrFileIdx = 0;

        if (mTxtDataFile.is_open()) {

            mTxtDataFile.close();
        }

        // loadTxtFile() only works in READY state!
        if (mLoadState == GOOD) {
            mLoadState = READY;
        }
        this->loadTxtFile();
    }

} // OG_SLAM