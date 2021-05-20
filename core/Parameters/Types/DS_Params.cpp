//
// Created by root on 5/10/21.
//

#include "DS_Params.h"

using namespace std;

namespace OG_SLAM {

    DS_Params::DS_Params() :
            mFormat(NOT_SUPPORTED), mpSensorConfig(), mnSeqTarget(0), mnMaxIter(1), mfTsFactor(1.0),
            mbImuGyroFirst(true), mbGtPosFirst(true), mbGtQwFirst(true)
    {}

    DS_Params::DS_Params(const cv::FileStorage &fsSettings) : DS_Params() {

        cv::FileNode dsNode = fsSettings["DS"];
        if (dsNode.empty()) {
            return;
        }

        this->read(dsNode);
    }

    DS_Params::DS_Formats DS_Params::mapDsFormats(const string& strFormat) {

        if (strFormat == "euroc")
            return DS_Params::EUROC;
        else if (strFormat == "ev_ethz")
            return DS_Params::EV_ETHZ;
        else if (strFormat == "ev_mvsec")
            return DS_Params::EV_MVSEC;
        else
            return DS_Params::NOT_SUPPORTED;
    }

    string DS_Params::mapDsFormats(const DS_Params::DS_Formats dsFormat) {

        switch (dsFormat) {
            case DS_Params::EUROC:
                return "euroc";
            case DS_Params::EV_ETHZ:
                return "ev_ethz";
            case DS_Params::EV_MVSEC:
                return "ev_mvsec";
            case DS_Params::NOT_SUPPORTED:
            default:
                return "not_supported";
        }
    }

    void DS_Params::write(cv::FileStorage &fs) const {

        fs << "DS" << "{";

        YamlParserCV::writeString(fs, "name", mName);
        YamlParserCV::writeString(fs, "format", mapDsFormats(mFormat));
        //fs << *(mpSensorConfig);
        mpSensorConfig->write(fs);

        YamlParserCV::writeMap<string>(fs, "paths", mmPaths);

        fs << "sequence" << "{";
        YamlParserCV::writeInt(fs, "target", mnSeqTarget);
        YamlParserCV::writeSequence(fs, "names", mvSeqNames);
        fs << "}";

        YamlParserCV::writeInt(fs, "numMaxIter", mnMaxIter);
        YamlParserCV::writeReal(fs, "fsFactor", mfTsFactor);

        YamlParserCV::writeBool(fs, "gtQwFirst", mbGtQwFirst);
        YamlParserCV::writeBool(fs, "gtPosFirst", mbGtPosFirst);
        YamlParserCV::writeBool(fs, "imuGyroFirst", mbImuGyroFirst);

        fs << "}";
    }

    void DS_Params::read(const cv::FileNode &dsNode) {

        // Resolve DS name
        mName = YamlParserCV::readString(dsNode, "name", "");

        // Resolve DS format
        string dsFormat = YamlParserCV::readString(dsNode, "format", mapDsFormats(NOT_SUPPORTED));
        mFormat = mapDsFormats(dsFormat);

        // DS sensor configuration
        mpSensorConfig = std::make_shared<SensorConfig>(dsNode);
        //dsNode >> *(mpSensorConfig);

        // Important DS Paths
        YamlParserCV::readMap<string>(dsNode, "paths", mmPaths);

        // Sequence Info.
        cv::FileNode seqNode = dsNode["sequence"];
        if (!seqNode.empty()) {
            mnSeqTarget = YamlParserCV::readInt(seqNode, "target", 0);
            YamlParserCV::readSequence<string>(seqNode, "names", mvSeqNames);
        }

        // Other DS info.
        mnMaxIter = YamlParserCV::readInt(dsNode, "numMaxIter", 1);
        mfTsFactor = YamlParserCV::readReal<double>(dsNode, "tsFactor", 1.0);

        mbGtQwFirst = YamlParserCV::readBool(dsNode, "gtQwFirst", true);
        mbGtPosFirst = YamlParserCV::readBool(dsNode, "gtPosFirst", true);
        mbImuGyroFirst = YamlParserCV::readBool(dsNode, "imuGyroFirst", true);
    }

    std::string DS_Params::printStr(const std::string& prefix) {

        ostringstream oss;

        oss << prefix << "Name: " << mName << endl;
        oss << prefix << "Format: " << mapDsFormats(mFormat) << endl;
        oss << prefix << "Sensor Configuration: " << mpSensorConfig->toStr() << endl;

        oss << prefix << "Paths:\n";
        for (const auto& pathItem : mmPaths) {
            oss << prefix << "\t" << pathItem.first << ": " << pathItem.second << endl;
        }

        oss << prefix << "Sequence Info:\n";
        oss << prefix << "\tTarget: " << mnSeqTarget << endl;
        oss << prefix << "\tNames:\n" << prefix << "\t\t";
        for(const string& seqName : mvSeqNames) {
            oss << seqName << ", ";
        }
        oss << endl;

        oss << prefix << "Num Max. Iterations: " << mnMaxIter << endl;
        oss << prefix << "Timestamp Factor: " << mfTsFactor << endl;

        oss << prefix << "GT Qw First? " << mbGtQwFirst << endl;
        oss << prefix << "GT Pos First? " << mbGtPosFirst << endl;
        oss << prefix << "IMU Gyro First? " << mbImuGyroFirst << endl;

        return oss.str();
    }

    std::string DS_Params::getDatasetRoot() {

        if (mmPaths.count("root")) {
            return mmPaths["root"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getImageFilePath() {

        if (mmPaths.count("imageFile")) {
            return mmPaths["imageFile"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getImageBasePath() {

        if (mmPaths.count("imageBase")) {
            return mmPaths["imageBase"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getIMU_Path() {

        if (mmPaths.count("imu")) {
            return mmPaths["imu"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getGroundTruthPosePath() {

        if (mmPaths.count("gt")) {
            return mmPaths["gt"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getEventsPath() {

        if (mmPaths.count("events")) {
            return mmPaths["events"];
        }
        return std::__cxx11::string();
    }

    std::string DS_Params::getResultsBasePath() {

        if (mmPaths.count("resultsBase")) {
            return mmPaths["resultsBase"];
        }
        return std::__cxx11::string();
    }

} // OG_SLAM
