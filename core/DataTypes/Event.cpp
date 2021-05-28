//
// Created by root on 5/22/21.
//

#include "Event.h"

using namespace std;

namespace OG_SLAM {

    double calcEventGenRate(const std::vector<EventData>& l1Evs, const int imWidth, const int imHeight) {

        double evTspan = l1Evs.back().ts-l1Evs[0].ts;
        double evGenRate = ((double) l1Evs.size()) / (evTspan * imWidth * imHeight);
        return evGenRate;
    }

    /* ------------------------------------------------------------------------------------------------------------- */

    unsigned long EventQueue::consumeBegin(unsigned long chunkSize, vector<EvDataPtr> &evs, vector<EvDataPtr> &accEvs) {

        std::unique_lock<std::mutex> lock1(mMutexBuffer);
        if (mQueue.empty())
            return 0;

        unsigned nEvs = min(chunkSize, mQueue.size());
        evs.resize(nEvs);

        for (unsigned i = 0; i < nEvs; i++) {
            EvDataPtr ev = mQueue.front();
            evs[i] = ev;
            accEvs.push_back(ev);
            mQueue.pop();
        }
        return nEvs;
    }

} // OG_SLAM
