//
// Created by root on 5/14/21.
//

#include "EventParams.h"

using namespace std;

namespace OG_SLAM {


    EvDataParams::EvDataParams() :
            mbRectified(false), mbL1FixedWin(false), mnL1ChunkSize(2000), mnL1NumLoop(5),
            mfMinEvGenRate(1.f), mfMaxEvGenRate(100.f), mfMaxPxDisp(5.f)
    {}

    EvDataParams::EvDataParams(const cv::FileNode &dataNode) : EvDataParams() {

        this->read(dataNode);
    }

    void EvDataParams::write(cv::FileStorage &fs) const {

        YamlParserCV::writeBool(fs, "isRectified", mbRectified);
        YamlParserCV::writeBool(fs, "l1FixedWin", mbL1FixedWin);

        YamlParserCV::writeInt(fs, "l1ChunkSize", mnL1ChunkSize);
        YamlParserCV::writeInt(fs, "l1NumLoop", mnL1NumLoop);

        YamlParserCV::writeReal(fs, "minEvGenRate", mfMinEvGenRate);
        YamlParserCV::writeReal(fs, "maxEvGenRate", mfMaxEvGenRate);
        YamlParserCV::writeReal(fs, "maxPixelDisp", mfMaxPxDisp);
    }

    void EvDataParams::read(const cv::FileNode &dataNode) {

        mbRectified = YamlParserCV::readBool(dataNode, "isRectified", mbRectified);
        mbL1FixedWin = YamlParserCV::readBool(dataNode, "l1FixedWin", mbL1FixedWin);

        mnL1ChunkSize = YamlParserCV::readInt(dataNode, "l1ChunkSize", mnL1ChunkSize);
        mnL1NumLoop = YamlParserCV::readInt(dataNode, "l1NumLoop", mnL1NumLoop);

        mfMinEvGenRate = YamlParserCV::readReal<float>(dataNode, "minEvGenRate", mfMinEvGenRate);
        mfMaxEvGenRate = YamlParserCV::readReal<float>(dataNode, "maxEvGenRate", mfMaxEvGenRate);
        mfMaxPxDisp = YamlParserCV::readReal<float>(dataNode, "maxPixelDisp", mfMaxPxDisp);
    }

    std::string EvDataParams::printStr(const std::string &prefix) const {

        ostringstream oss;

        string evRectifiedStr = "Yes";
        if (!mbRectified)
            evRectifiedStr = "No";
        oss << prefix << "Events Rectified? " << evRectifiedStr << endl;

        string l1FixedWinStr = "Yes";
        if (!mbL1FixedWin)
            l1FixedWinStr = "No";
        oss << prefix << "Level 1 Window Size is fixed? " << l1FixedWinStr << endl;

        oss << prefix << "Init. L1 Chunk Size: " << mnL1ChunkSize << endl;
        oss << prefix << "L1 Num. Loop: " << mnL1NumLoop << endl;

        oss << prefix << "Min. Event Generation Rate: " << mfMinEvGenRate << endl;
        oss << prefix << "Max. Pixel Displacement (L1 Tracking): " << mfMaxPxDisp << endl;

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    EvAlgoParams::EvAlgoParams() :
            mTrackMode(ODOMETERY), mbTrackTinyFrames(false), mfL1ImSigma(1.f), mfL2ImSigma(1.f)
    {}

    EvAlgoParams::EvAlgoParams(const cv::FileNode &dataNode) : EvAlgoParams() {

        this->read(dataNode);
    }

    void EvAlgoParams::write(cv::FileStorage &fs) const {

        YamlParserCV::writeString(fs, "l2TrackMode", mapTrackMode(mTrackMode));

        YamlParserCV::writeBool(fs, "trackTinyFrames", mbTrackTinyFrames);

        YamlParserCV::writeReal(fs, "l1ImSigma", mfL1ImSigma);
        YamlParserCV::writeReal(fs, "l2ImSigma", mfL2ImSigma);
    }

    void EvAlgoParams::read(const cv::FileNode &evAlgoNode) {

        string trModeStr = YamlParserCV::readString(evAlgoNode, "l2TrackMode", mapTrackMode(mTrackMode));
        mTrackMode = mapTrackMode(trModeStr);

        mbTrackTinyFrames = YamlParserCV::readBool(evAlgoNode, "trackTinyFrames", mbTrackTinyFrames);

        mfL1ImSigma = YamlParserCV::readReal<float>(evAlgoNode, "l1ImSigma", mfL1ImSigma);
        mfL2ImSigma = YamlParserCV::readReal<float>(evAlgoNode, "l2ImSigma", mfL2ImSigma);
    }

    std::string EvAlgoParams::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Level 2 Tracking Mode: " << mapTrackMode(mTrackMode) << endl;

        string trTinyFrStr = "Yes";
        if (!mbTrackTinyFrames)
            trTinyFrStr = "No";
        oss << prefix << "Track Tiny Frames in Level 2? " << trTinyFrStr << endl;

        oss << prefix << "Level 1 Image Reconst. Sigma: " << mfL1ImSigma << endl;
        oss << prefix << "Level 2 Image Reconst. Sigma: " << mfL2ImSigma << endl;

        return oss.str();
    }

    EvAlgoParams::L2TrackingMode EvAlgoParams::mapTrackMode(const string &trackMode) {

        if (trackMode == "tlm-chr") {
            return TLM_CHR;
        }
        else if (trackMode == "tlm") {
            return TLM;
        }
        else {
            return ODOMETERY;
        }
    }

    std::string EvAlgoParams::mapTrackMode(L2TrackingMode trackMode) {

        switch (trackMode) {
            case TLM:
                return "tlm";
            case TLM_CHR:
                return "tlm-chr";
            case ODOMETERY:
            default:
                return "odom";
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    EvFtParams::EvFtParams() :
            mnFtDtMode(0), mnFeatures(0), mnThFAST(0), mnL1NumLevels(1), mnL2NumLevels(1),
            mfL1ScaleFactor(1.f), mfL2ScaleFactor(1.f)
    {}

    EvFtParams::EvFtParams(const cv::FileNode &dataNode) : EvFtParams() {

        this->read(dataNode);
    }

    void EvFtParams::write(cv::FileStorage &fs) const {

        YamlParserCV::writeInt(fs, "detMode", mnFtDtMode);

        YamlParserCV::writeInt(fs, "maxNumPts", mnFeatures);

        YamlParserCV::writeInt(fs, "fastTh", mnThFAST);

        YamlParserCV::writeInt(fs, "l1NLevels", mnL1NumLevels);
        YamlParserCV::writeInt(fs, "l2NLevels", mnL2NumLevels);

        YamlParserCV::writeReal(fs, "l1ScaleFactor", mfL1ScaleFactor);
        YamlParserCV::writeReal(fs, "l2ScaleFactor", mfL2ScaleFactor);
    }

    void EvFtParams::read(const cv::FileNode &evFtNode) {

        mnFtDtMode = YamlParserCV::readInt(evFtNode, "detMode", mnFtDtMode);

        mnFeatures = YamlParserCV::readInt(evFtNode, "maxNumPts", mnFeatures);

        mnThFAST = YamlParserCV::readInt(evFtNode, "fastTh", mnThFAST);

        mnL1NumLevels = YamlParserCV::readInt(evFtNode, "l1NLevels", mnL1NumLevels);
        mnL2NumLevels = YamlParserCV::readInt(evFtNode, "l2NLevels", mnL2NumLevels);

        mfL1ScaleFactor = YamlParserCV::readReal<float>(evFtNode, "l1ScaleFactor", mfL1ScaleFactor);
        mfL2ScaleFactor = YamlParserCV::readReal<float>(evFtNode, "l2ScaleFactor", mfL2ScaleFactor);
    }

    std::string EvFtParams::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Feature Detection Mode: " << mnFtDtMode << endl;
        oss << prefix << "Max Num. Feature Points: " << mnFeatures << endl;

        oss << prefix << "FAST Threshold: " << mnThFAST << endl;
        oss << prefix << "L1 Num. Scale Levels: " << mnL1NumLevels << endl;
        oss << prefix << "L2 Num. Scale Levels: " << mnL2NumLevels << endl;
        oss << prefix << "L1 Scale Factor: " << mfL1ScaleFactor << endl;
        oss << prefix << "L2 Scale Factor: " << mfL2ScaleFactor << endl;

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    EvKLT_Params::EvKLT_Params() :
            mnMaxLevel(0), mnMaxIter(10), mnWinSize(23), mfEps(0.03f), mfThRefreshPts(0.9f),
            mfThRefreshPtsDist(1.f)
    {}

    EvKLT_Params::EvKLT_Params(const cv::FileNode &dataNode) : EvKLT_Params() {

        this->read(dataNode);
    }

    void EvKLT_Params::write(cv::FileStorage &fs) const {

        YamlParserCV::writeInt(fs, "maxLevel", mnMaxLevel);
        YamlParserCV::writeInt(fs, "winSize", mnWinSize);

        YamlParserCV::writeReal(fs, "eps", mfEps);
        YamlParserCV::writeInt(fs, "maxIter", mnMaxIter);

        YamlParserCV::writeReal(fs, "maxThRefreshPoints", mfThRefreshPts);
        YamlParserCV::writeReal(fs, "distRefreshPoints", mfThRefreshPtsDist);
    }

    void EvKLT_Params::read(const cv::FileNode &ekltNode) {

        mnMaxLevel = YamlParserCV::readInt(ekltNode, "maxLevel", mnMaxLevel);
        mnWinSize = YamlParserCV::readInt(ekltNode, "winSize", mnWinSize);

        mfEps = YamlParserCV::readReal<float>(ekltNode, "eps", mfEps);
        mnMaxIter = YamlParserCV::readInt(ekltNode, "maxIter", mnMaxIter);

        mfThRefreshPts = YamlParserCV::readReal<float>(ekltNode, "maxThRefreshPoints", mfThRefreshPts);
        mfThRefreshPtsDist = YamlParserCV::readReal<float>(ekltNode, "distRefreshPoints", mfThRefreshPtsDist);
    }

    std::string EvKLT_Params::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Max Num. Levels (zero-based): " << mnMaxLevel << endl;
        oss << prefix << "Window Size: " << mnWinSize << endl;

        oss << prefix << "Epsilon: " << mfEps << endl;
        oss << prefix << "Max. Iterations: " << mnMaxIter << endl;

        oss << prefix << "Refresh Points Threshold: " << mfThRefreshPts << endl;
        oss << prefix << "Refresh Points Distance Threshold: " << mfThRefreshPtsDist << endl;

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    EventParams::EventParams(const cv::FileStorage &fsSettings) {

        this->read(fsSettings["Event"]);
    }

    void EventParams::write(cv::FileStorage &fs) const {

        fs << "Event" << "{";

        fs << "Data" << "{";
        mpDataParams->write(fs);
        fs << "}";
        fs << "Algorithm" << "{";
        mpAlgoParams->write(fs);
        fs << "}";
        fs << "Features" << "{";
        mpEvFtParams->write(fs);
        fs << "}";
        fs << "KLT" << "{";
        mpEKLT_Params->write(fs);
        fs << "}";
        fs << "}";
    }

    void EventParams::read(const cv::FileNode &eventNode) {

        mpDataParams = make_shared<EvDataParams>(eventNode["Data"]);
        mpAlgoParams = make_shared<EvAlgoParams>(eventNode["Algorithm"]);
        mpEvFtParams = make_shared<EvFtParams>(eventNode["Features"]);
        mpEKLT_Params = make_shared<EvKLT_Params>(eventNode["KLT"]);
    }

    std::string EventParams::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Event Data Parameters:\n";
        oss << mpDataParams->printStr(prefix+"\t");
        oss << prefix << "Event Algorithm Parameters:\n";
        oss << mpAlgoParams->printStr(prefix+"\t");
        oss << prefix << "Event Features Parameters:\n";
        oss << mpEvFtParams->printStr(prefix+"\t");
        oss << prefix << "Event KLT Parameters:\n";
        oss << mpEKLT_Params->printStr(prefix+"\t");

        return oss.str();
    }
} // OG_SLAM
