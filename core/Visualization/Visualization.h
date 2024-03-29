//
// Created by root on 5/21/21.
//

#ifndef OG_SLAM_VISUALIZATION_H
#define OG_SLAM_VISUALIZATION_H

#include <memory>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "Image.h"


namespace OG_SLAM {

    class SimpleImageDisplay {
    public:
        explicit SimpleImageDisplay(float fps, std::string  winName = "Window") :
                mfps(fps), mbStop(false), mWindowName(std::move(winName)) {

            mqpImages = std::make_shared<ImageQueue>();
        }

        void run();
        void requestStop() { mbStop = true; }
        bool isStopped() const { return mbStop; }

        ImageQueuePtr mqpImages;
        float mfps;
        bool mbStop;
        std::string mWindowName;
    };

    class Visualization {

    };

} // OG_SLAM


#endif //OG_SLAM_VISUALIZATION_H
