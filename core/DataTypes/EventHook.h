//
// Created by root on 5/22/21.
//

#ifndef OG_SLAM_EVENTHOOK_H
#define OG_SLAM_EVENTHOOK_H

#include <utility>

#include "Event.h"
#include "SharedQueue.h"

namespace OG_SLAM {

    class EventHook {
    public:
        explicit EventHook(EventQueuePtr pEvQueue) : mpEvQueue(std::move(pEvQueue)) {}

        void dispatch(const EvDataPtr& pEvData) { mpEvQueue->push(pEvData); }
        void dispatch(const std::vector<EvDataPtr>& vpEvData) { mpEvQueue->fillBuffer(vpEvData); }

    protected:
        EventQueuePtr mpEvQueue;
    };

    typedef std::shared_ptr<EventHook> EventHookPtr;

} // OG_SLAM


#endif //OG_SLAM_EVENTHOOK_H
