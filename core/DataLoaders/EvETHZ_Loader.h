//
// Created by root on 5/21/21.
//

#ifndef OG_SLAM_EVETHZ_LOADER_H
#define OG_SLAM_EVETHZ_LOADER_H

#include "EurocLoader.h"
#include "EventDS.h"
#include "EventHook.h"

namespace OG_SLAM {

    class EvETHZ_Loader : public EurocLoader {
    public:
        explicit EvETHZ_Loader(const DS_ParamsPtr& pDsParams);
        ~EvETHZ_Loader() override = default;

        void addEventHook(const EventHookPtr& pEventHook);

        void play() override;

        void resetCurrSequence() override;

        unsigned long getNextEvents(unsigned long chunkSize, std::vector<EvDataPtr>& evs,
                                    bool undistPoints = false, bool checkInImage = true);
        unsigned long getNextEvents(double tsEnd, std::vector<EvDataPtr> &evs,
                                    bool undistPoints = false, bool checkInImage = true);

    protected:
        void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx) override;

        bool checkLoadState() override;

    private:
        std::string mPathEvents;

        std::vector<EventDS_UPtr> mvpEventDS;

        std::vector<EventHookPtr> mvpEventHooks;

    };

} // OG_SLAM


#endif //OG_SLAM_EVETHZ_LOADER_H
