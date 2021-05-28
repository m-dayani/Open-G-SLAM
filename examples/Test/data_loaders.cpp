
#include <iostream>
#include <memory>
#include <thread>

#include "ParameterServer.h"
#include "EurocLoader.h"
#include "EvETHZ_Loader.h"
#include "Visualization.h"
#include "TabularTextWriter.h"

using namespace std;
using namespace OG_SLAM;


int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " settings.yaml\n";
        return 1;
    }

    string settingsFile = argv[1];
    cout << "Settings File: " << settingsFile << endl;

    // Testing parameter server
    // Load Settings
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(settingsFile);
    DLOG(INFO) << pParamServer->getFullStat();

    // Save Settings
    string paramsFile = "../../config/DUMMY.yaml";
    cout << "Saving parameters to: " << paramsFile << endl;
    pParamServer->save(paramsFile);

    // Get required camera parameters
    CamParamsPtr pCamParams = pParamServer->getCamParams();
    SimpleImageDisplay imageDisplay(pCamParams->fps);

    // Testing different loaders
    // Init. Loader
    EvETHZ_Loader myLoader(pParamServer->getDS_Params());
    DLOG(INFO) << myLoader.printLoaderStateStr();

    // Prepare & add image hook to get images
    ImageFilterPtr imFilterGray = make_shared<ImageFilterGray>(pCamParams->mbRGB);
    ImageHookPtr imageHook = make_shared<ImageHookFiltered>(imageDisplay.mqpImages, imFilterGray);
    myLoader.addImageHook(imageHook);

    string imuFile = "../../../data/dummy-imu.txt";
    IMU_Writer imuWriter(imuFile);
    IMU_HookPtr pImuHook = make_shared<IMU_Hook>(imuWriter.mpDataQueue);
    myLoader.addIMU_Hook(pImuHook);

    string gtPoseFile = "../../../data/dummy-gt-pose.txt";
    GtPoseWriter gtPoseWriter(gtPoseFile);
    PoseHookPtr pPoseHook = make_shared<PoseHook>(gtPoseWriter.mpDataQueue);
    myLoader.addGT_PoseHook(pPoseHook);

    // Run image display thread (Test Image Feed)
    auto* mptImDisplay = new std::thread(&SimpleImageDisplay::run, &imageDisplay);
    auto* mptImuWriter = new std::thread(&IMU_Writer::run, &imuWriter);
    auto* mptGtPoseWriter = new std::thread(&GtPoseWriter::run, &gtPoseWriter);

    // Play data loader
    myLoader.play();

    // TODO: Enhance this
    // Wait until all images are displayed
    while (!imageDisplay.mqpImages->empty())
        std::this_thread::sleep_for(chrono::milliseconds(20));

    imageDisplay.mbStop = true;
    imuWriter.mbStop = true;
    gtPoseWriter.mbStop = true;

    mptImDisplay->join();
    delete mptImDisplay;
    mptImuWriter->join();
    delete mptImuWriter;
    mptGtPoseWriter->join();
    delete mptGtPoseWriter;

    return 0;
}