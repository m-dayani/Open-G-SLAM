//
// Created by root on 5/22/21.
//

#ifndef OG_SLAM_EVENT_H
#define OG_SLAM_EVENT_H

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <opencv2/core.hpp>

#include "SharedQueue.h"


namespace OG_SLAM {

#define DEF_L1_CHUNK_SIZE 2000
#define DEF_L1_NUM_LOOP 5
#define DEF_IMG_SIGMA 1.0f
#define DEF_EV_WIN_MIN_SZ 2000
#define DEF_TH_MIN_KPTS 100
#define DEF_TH_MAX_KPTS 200
#define DEF_TH_MIN_MATCHES 50

#define DEF_LOW_FTS_THRESH 0.9f
#define DEF_TH_EV_FAST_DET 1
#define DEF_LK_ITR_MAX_CNT 10
#define DEF_LK_TERM_EPS 0.03f
#define DEF_LK_WIN_SIZE 15
#define DEF_LK_MAX_LEVEL 1
#define DEF_DIST_REFRESH_PTS 1.f

    struct EventData {

        EventData() = default;

        EventData(double ts, float x, float y, bool p) :
                ts(ts), x(x), y(y), p(p) {}

        void print() const {
            std::cout << "[ts, (x,y), pol]: [" << ts << ", (" << x << ", " << y << "), " << p << "]\n";
        }

        /*static bool isInImage(const EventData& ev, const float imWidth, const float imHeight) {
            return ev.x >= 0 && ev.x < imWidth && ev.y >= 0 && ev.y < imHeight;
        }
        bool isInImage(const float imWidth, const float imHeight) const {
            return x >= 0 && x < imWidth && y >= 0 && y < imHeight;
        }*/

        double ts = 0.0;
        float x = 0.f;
        float y = 0.f;
        bool p = false;
    };

    typedef std::shared_ptr<EventData> EvDataPtr;

    double calcEventGenRate(const std::vector<EventData>& l1Chunk, int imWidth, int imHeight);

    struct EventStat {

        unsigned int mnCurrFileIdx;
        unsigned long mnNextByteIdx;

        double evRate;
        double avgSpDistr;
        double avgVoxDistr;
        std::vector<int> vSpDistr;
        std::vector<int> vVoxDistr;
        cv::Point3f mGrid;
    };

    class EventQueue : public SharedQueue<EvDataPtr> {
    public:
        using SharedQueue<EvDataPtr>::fillBuffer;
        using SharedQueue<EvDataPtr>::consumeBegin;//(unsigned long chunkSize, std::vector<EventData>& evs);
        unsigned long consumeBegin(unsigned long chunkSize, std::vector<EvDataPtr>& evs, std::vector<EvDataPtr>& accEvs);

    private:
        //std::queue<EventData> mEvBuffer;
        //std::mutex mMutexBuffer;
    };

    typedef std::shared_ptr<EventQueue> EventQueuePtr;

} // OG_SLAM


#endif //OG_SLAM_EVENT_H
