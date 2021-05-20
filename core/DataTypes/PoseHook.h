//
// Created by root on 5/18/21.
//

#ifndef OG_SLAM_POSEHOOK_H
#define OG_SLAM_POSEHOOK_H

#include <memory>
#include <utility>

#include "Pose.h"

namespace OG_SLAM {

    class PoseHook {
    public:
        explicit PoseHook(PoseQueuePtr  pPoseQueue) : mpqPose(std::move(pPoseQueue)) {}

        void dispatch(const PosePtr& pPose) { mpqPose->push(pPose); }

    protected:
        PoseQueuePtr mpqPose;
    };

    typedef std::shared_ptr<PoseHook> PoseHookPtr;

} // OG_SLAM

#endif //OG_SLAM_POSEHOOK_H
