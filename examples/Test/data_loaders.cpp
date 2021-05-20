
#include <iostream>
#include <memory>
#include <thread>

#include <opencv2/highgui.hpp>

#include "ParameterServer.h"
#include "BaseLoader.h"

using namespace std;
using namespace OG_SLAM;

class ImageDisplay {
public:

    explicit ImageDisplay(float fps) : mfps(fps), mStop(false) {

        mqpImages = make_shared<ImageQueue>();
    }

    void run() {

        cv::namedWindow("Sample Image", cv::WINDOW_AUTOSIZE);

        while (!mStop) {

            if (!mqpImages->empty()) {

                ImagePtr pImage = mqpImages->front();

                if (pImage->mImage.empty()) {
                    mqpImages->pop();
                    break;
                }

                double imTs = 0.0;
                if (dynamic_cast<ImageTs*>(pImage.get())) {
                    auto* pImageTs = dynamic_cast<ImageTs*>(pImage.get());
                    imTs = pImageTs->mTimeStamp;
                }
                cv::Mat imageToShow = pImage->mImage.clone();

                cv::cvtColor(imageToShow, imageToShow, cv::COLOR_GRAY2RGB);

                cv::putText(imageToShow, to_string(imTs), cv::Point2f(10,10),
                            cv::FONT_HERSHEY_COMPLEX_SMALL,((float)imageToShow.cols*1.5f)/720.f,
                            cv::Scalar(0, 180, 0), 1);

                cv::imshow("Sample Image", imageToShow);
                cv::waitKey(static_cast<int>(1000.f / mfps));
                mqpImages->pop();
            }
        }
    }

    ImageQueuePtr mqpImages;
    float mfps;
    bool mStop;
};


int main(int argc, char** argv) {

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " settings.yaml\n";
        return 1;
    }

    string settingsFile = argv[1];
    cout << "Settings File: " << settingsFile << endl;

    // Load Settings
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(settingsFile);
    cout << pParamServer->getFullStat();

    // Save Settings
    string paramsFile = "../../config/DUMMY.yaml";
    cout << "Saving parameters to: " << paramsFile << endl;
    pParamServer->save(paramsFile);

    // Init. Loader
    BaseLoader baseLoader(pParamServer->getDS_Params());
    cout << "Output Base Name: " << baseLoader.getOutputBasePath() << endl;

    // Get required camera parameters
    CamParamsPtr pCamParams = pParamServer->getCamParams();
    ImageDisplay imageDisplay(pCamParams->fps);

    // Prepare & add image hook to get images
    ImageFilterPtr imFilterGray = make_shared<ImageFilterGray>(pCamParams->mbRGB);
    ImageHookPtr imageHook = make_shared<ImageHookFiltered>(imageDisplay.mqpImages, imFilterGray);

    baseLoader.addImageHook(imageHook);

    // Run image display thread
    new std::thread(&ImageDisplay::run, &imageDisplay);

    // Play data loader
    baseLoader.play();

    // Wait until all images are displayed
    while (!imageDisplay.mqpImages->empty())
        std::this_thread::sleep_for(chrono::milliseconds(20));

    imageDisplay.mStop = true;
    //mptImageDisplay->join();

    return 0;
}