//
// Created by root on 5/22/21.
//

#ifndef OG_SLAM_EVENTDS_H
#define OG_SLAM_EVENTDS_H

#include <opencv2/core.hpp>

#include "Event.h"
#include "TabularTextDS.h"


namespace OG_SLAM {

    class EventDS : public TabularTextDS {
    public:
        EventDS() : mLastEvTs(0.0) {}
        explicit EventDS(const std::string &filePath, double tsFactor);
        ~EventDS() override;

        unsigned long getEventChunk(unsigned long chunkSize, std::vector<EvDataPtr> &evBuffer);
        unsigned long getEventChunk(double tsEnd, std::vector<EvDataPtr> &evBuffer);
        // WARNING!! These methods only support Pinhole cameras
//        unsigned long getEventChunkRectified(unsigned long chunkSize, std::vector<EventData> &evBuffer,
//                                             const cv::Mat& K, const cv::Mat& distCoefs, const cv::Mat& rectMat,
//                                             const cv::Scalar& imageSize, bool checkInImage = true);
//        unsigned long getEventChunkRectified(double tsEnd, std::vector<EventData> &evBuffer,
//                                             const cv::Mat& K, const cv::Mat& distCoefs, const cv::Mat& rectMat,
//                                             const cv::Scalar& imageSize, bool checkInImage = true);
        // Use these methods whenever possible since they use camModel-free rectification
//        unsigned long getEventChunkRectified(unsigned long chunkSize, std::vector<EventData> &evBuffer,
//                                             const MyCalibPtr& pCalib, bool checkInImage = true);
//        unsigned long getEventChunkRectified(double tsEnd, std::vector<EventData> &evBuffer,
//                                             const MyCalibPtr& pCalib, bool checkInImage = true);
        //TODO: Develop methods to process event statistics too.
        //unsigned long getEventChunk(unsigned long chunkSize, std::vector<EventData> &evBuffer, EventStat& evStat);

        void reset() override;

    protected:
        boost::any parseLine(const std::string &evStr) override;
//        EventData parseLine(const std::string &evStr, const cv::Mat& K, const cv::Mat& distCoefs, const cv::Mat& rectMat);
//        EventData parseLine(const std::string &evStr, const MyCalibPtr& pCalib);

        //void parseEventData(float tsEnd, std::vector<EventData> &evBuffer);

    private:

        double mLastEvTs;
    };

    typedef std::shared_ptr<EventDS> EventDS_Ptr;
    typedef std::unique_ptr<EventDS> EventDS_UPtr;

} // OG_SLAM


#endif //OG_SLAM_EVENTDS_H
