//
// Created by root on 5/10/21.
//

#ifndef OG_SLAM_PARAMETERSERVER_H
#define OG_SLAM_PARAMETERSERVER_H

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include "DS_Params.h"
#include "CamParams.h"
#include "IMU_Params.h"
#include "Features2D.h"
#include "ViewerParams.h"
#include "EventParams.h"


namespace OG_SLAM {

    class ParameterServer {
    public:
        explicit ParameterServer(std::string& settingsFile);

        void save(const std::string& pathParams);

        std::string getFullStat();

        const DS_ParamsPtr &getDS_Params() const;

        const CamParamsPtr &getCamParams() const;

        const IMU_ParamsPtr &getIMU_Params() const;

        const Ft2D_ParamsPtr &getFt2D_Params() const;

        const ViewerParamsPtr &getViewerParams() const;

        const EventParamsPtr &getEventParams() const;

    private:
        DS_ParamsPtr mpDS_Params;
        CamParamsPtr mpCamParams;
        IMU_ParamsPtr mpImuParams;
        Ft2D_ParamsPtr mpFt2dParams;
        ViewerParamsPtr mpViewerParams;
        EventParamsPtr mpEventParams;
    };

} // OG_SLAM

#endif //OG_SLAM_PARAMETERSERVER_H
