//
// Created by root on 5/13/21.
//

#include "Features2D.h"

using namespace std;


namespace OG_SLAM {


    ORB_Params::ORB_Params() :
            mnFeatures(0), mnLevels(1), mfScaleFactor(1.f), mnIniThFAST(10), mnMinThFAST(7)
    {}

    ORB_Params::ORB_Params(const cv::FileNode &orbNode) : ORB_Params() {

        this->read(orbNode);
    }

    void ORB_Params::write(cv::FileStorage &fs) const {

        YamlParserCV::writeString(fs, "pathVocab", mPathVocab);

        fs << "extractor" << "{";

        YamlParserCV::writeInt(fs, "nFeatures", mnFeatures);
        YamlParserCV::writeInt(fs, "nLevels", mnLevels);

        YamlParserCV::writeReal(fs, "scaleFactor", mfScaleFactor);
        YamlParserCV::writeInt(fs, "iniThFAST", mnIniThFAST);
        YamlParserCV::writeInt(fs, "minThFAST", mnMinThFAST);

        fs << "}";
    }

    void ORB_Params::read(const cv::FileNode &orbNode) {

        mPathVocab = YamlParserCV::readString(orbNode, "pathVocab", string());

        cv::FileNode orbExtNode = orbNode["extractor"];

        mnFeatures = YamlParserCV::readInt(orbExtNode, "nFeatures", mnFeatures);

        mnLevels = YamlParserCV::readInt(orbExtNode, "nLevels", mnLevels);
        mfScaleFactor = YamlParserCV::readReal<float>(orbExtNode, "scaleFactor", mfScaleFactor);

        mnIniThFAST = YamlParserCV::readInt(orbExtNode, "iniThFAST", mnIniThFAST);
        mnMinThFAST = YamlParserCV::readInt(orbExtNode, "minThFAST", mnMinThFAST);
    }

    std::string ORB_Params::printStr(const std::string &prefix) const {

        ostringstream oss;

        oss << prefix << "ORB Vocabulary Path: " << mPathVocab << endl;

        oss << prefix << "ORB Extractor Parameters:\n";
        oss << prefix << "\tNum. Features: " << mnFeatures << endl;
        oss << prefix << "\tNum. Scale Levels: " << mnLevels << endl;
        oss << prefix << "\tScale Factor: " << mfScaleFactor << endl;
        oss << prefix << "\tIni. FAST Threshold: " << mnIniThFAST << endl;
        oss << prefix << "\tMin FAST Threshold: " << mnMinThFAST << endl;

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    AKAZE_Params::AKAZE_Params() :
            mnFeatures(0), mnOctaves(1), mnOctaveLayers(1), mfScaleFactor(1.f), mfIniTh(1e-3), mfMinTh(1e-3)
    {}

    AKAZE_Params::AKAZE_Params(const cv::FileNode &akazeNode) : AKAZE_Params() {

        this->read(akazeNode);
    }

    void AKAZE_Params::write(cv::FileStorage &fs) const {

        fs << "extractor" << "{";

        YamlParserCV::writeInt(fs, "nFeatures", mnFeatures);
        YamlParserCV::writeInt(fs, "nOctaves", mnOctaves);
        YamlParserCV::writeInt(fs, "nOctaveLayers", mnOctaveLayers);

        //YamlParserCV::writeReal(fs, "scaleFactor", mfScaleFactor);
        YamlParserCV::writeReal(fs, "iniTh", mfIniTh);
        YamlParserCV::writeReal(fs, "minTh", mfMinTh);

        fs << "}";
    }

    void AKAZE_Params::read(const cv::FileNode &akazeNode) {

        cv::FileNode akazeExtNode = akazeNode["extractor"];

        mnFeatures = YamlParserCV::readInt(akazeExtNode, "nFeatures", mnFeatures);
        mnOctaves = YamlParserCV::readInt(akazeExtNode, "nOctaves", mnOctaves);
        mnOctaveLayers = YamlParserCV::readInt(akazeExtNode, "nOctaveLayers", mnOctaveLayers);

        // AKAZE Scale Factor is calculated based on ORB sf
        //mfScaleFactor = YamlParserCV::readReal<float>(akazeExtNode, "scaleFactor", mfScaleFactor);

        mfIniTh = YamlParserCV::readReal<float>(akazeExtNode, "iniTh", mfIniTh);
        mfMinTh = YamlParserCV::readReal<float>(akazeExtNode, "minTh", mfMinTh);
    }

    std::string AKAZE_Params::printStr(const std::string &prefix) const {

        ostringstream oss;

        oss << prefix << "AKAZE Extractor Parameters:\n";
        oss << prefix << "\tNum. Features: " << mnFeatures << endl;
        oss << prefix << "\tNum. Octaves: " << mnOctaves << endl;
        oss << prefix << "\tNum. Octave Layers: " << mnOctaveLayers << endl;
        //oss << prefix << "\tScale Factor: " << mfScaleFactor << endl;
        oss << prefix << "\tIni. Threshold: " << mfIniTh << endl;
        oss << prefix << "\tMin Threshold: " << mfMinTh << endl;

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    Ft2D_Params::Ft2D_Params() : mFtDtMode(NONE) {}

    Ft2D_Params::Ft2D_Params(const cv::FileStorage &fSettings) : Ft2D_Params() {

        this->read(fSettings["Features"]);
    }

    void Ft2D_Params::write(cv::FileStorage &fs) const {

        fs << "Features" << "{";

        YamlParserCV::writeString(fs, "mode", mapFtDtMode(mFtDtMode));

        fs << "ORB" << "{";
        mpOrbParams->write(fs);
        fs << "}";

        fs << "AKAZE" << "{";
        mpAkazeParams->write(fs);
        fs << "}";

        fs << "}";
    }

    void Ft2D_Params::read(const cv::FileNode &ftNode) {

        string ftDtModeStr = YamlParserCV::readString(ftNode, "mode", mapFtDtMode(NONE));
        mFtDtMode = mapFtDtMode(ftDtModeStr);

        mpOrbParams = make_shared<ORB_Params>(ftNode["ORB"]);

        mpAkazeParams = make_shared<AKAZE_Params>(ftNode["AKAZE"]);
    }

    std::string Ft2D_Params::printStr(const string &prefix) const {

        ostringstream oss;

        string ftModeStr = mapFtDtMode(mFtDtMode);
        std::transform(ftModeStr.begin(), ftModeStr.end(), ftModeStr.begin(), ::toupper);
        oss << prefix << "Feature Detection Mode: " << ftModeStr << endl;

        oss << prefix << "ORB Parameters:\n";
        oss << mpOrbParams->printStr(prefix+"\t");

        oss << prefix << "AKAZE Parameters:\n";
        oss << mpAkazeParams->printStr(prefix+"\t");

        return oss.str();
    }

    Ft2D_Params::FtDtModes Ft2D_Params::mapFtDtMode(const string &ftDtMode) {

        if (ftDtMode == "orb") {
            return ORB;
        }
        else if (ftDtMode == "akaze") {
            return AKAZE;
        }
        else if (ftDtMode == "mixed") {
            return MIXED;
        }
        else {
            return Ft2D_Params::NONE;
        }
    }

    std::string Ft2D_Params::mapFtDtMode(Ft2D_Params::FtDtModes ftDtMode) {

        switch (ftDtMode) {
            case ORB:
                return "orb";
            case AKAZE:
                return "akaze";
            case MIXED:
                return "mixed";
            case NONE:
            default:
                return "none";
        }
    }

} // OG_SLAM
