//
// Created by root on 5/10/21.
//

#include "ParameterServer.h"

using namespace std;


namespace OG_SLAM {

ParameterServer::ParameterServer(std::string &settingsFile) {

    cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {

        cerr << "** ERROR: Failed to open settings file: " << settingsFile << endl;
        return;
    }

    mpDS_Params = make_shared<DS_Params>(fsSettings);

    mpCamParams = make_shared<CamParams>(fsSettings);

    mpImuParams = make_shared<IMU_Params>(fsSettings);

    mpFt2dParams = make_shared<Ft2D_Params>(fsSettings);

    mpViewerParams = make_shared<ViewerParams>(fsSettings);

    mpEventParams = make_shared<EventParams>(fsSettings);

    fsSettings.release();
}

std::string ParameterServer::getFullStat() {

    ostringstream oss;

    oss << "# Dataset Info:\n";
    oss << mpDS_Params->printStr("\t");

    oss << "# Camera Parameters:\n";
    oss << CamParams::printStr(mpCamParams.get(), "\t");
    if (mpCamParams->mpLinkedCam) {
        oss << "# Right Camera Parameters:\n";
        oss << CamParams::printStr(mpCamParams->mpLinkedCam, "\t");
    }

    oss << "# IMU Parameters:\n";
    oss << mpImuParams->printStr("\t");

    oss << "# 2D Image Features:\n";
    oss << mpFt2dParams->printStr("\t");

    oss << "# Viewer Parameters:\n";
    oss << mpViewerParams->printStr("\t");

    oss << "# Event Parameters:\n";
    oss << mpEventParams->printStr("\t");

    return oss.str();
}

    void ParameterServer::save(const string &pathParams) {

        cv::FileStorage paramsFile(pathParams, cv::FileStorage::WRITE);
        if(!paramsFile.isOpened()) {

            cerr << "** ERROR: Failed to open settings file: " << pathParams << endl;
            return;
        }

        mpDS_Params->write(paramsFile);
        mpCamParams->write(paramsFile);
        mpImuParams->write(paramsFile);
        mpFt2dParams->write(paramsFile);
        mpViewerParams->write(paramsFile);
        mpEventParams->write(paramsFile);

        paramsFile.release();
    }

    const DS_ParamsPtr &ParameterServer::getDS_Params() const {
        return mpDS_Params;
    }

    const CamParamsPtr &ParameterServer::getCamParams() const {
        return mpCamParams;
    }

    const IMU_ParamsPtr &ParameterServer::getIMU_Params() const {
        return mpImuParams;
    }

    const Ft2D_ParamsPtr &ParameterServer::getFt2D_Params() const {
        return mpFt2dParams;
    }

    const ViewerParamsPtr &ParameterServer::getViewerParams() const {
        return mpViewerParams;
    }

    const EventParamsPtr &ParameterServer::getEventParams() const {
        return mpEventParams;
    }

} // OG_SLAM
