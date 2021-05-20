//
// Created by root on 5/18/21.
//

#ifndef OG_SLAM_IMU_HOOK_H
#define OG_SLAM_IMU_HOOK_H

#include <memory>
#include <utility>

#include "IMU.h"
#include "SharedQueue.h"


namespace OG_SLAM {

    class IMU_Hook {
    public:
        explicit IMU_Hook(IMU_QueuePtr  pImuQueue) : mpqIMU_Data(std::move(pImuQueue)) {}

        void dispatch(const IMU_DataPtr& pImuData) { mpqIMU_Data->push(pImuData); }

    protected:
        IMU_QueuePtr mpqIMU_Data;
    };

    typedef std::shared_ptr<IMU_Hook> IMU_HookPtr;

} // OG_SLAM


#endif //OG_SLAM_IMU_HOOK_H
