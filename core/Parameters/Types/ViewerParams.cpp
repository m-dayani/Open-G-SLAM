//
// Created by root on 5/13/21.
//

#include "ViewerParams.h"

using namespace std;

namespace OG_SLAM {


    ViewerParams::ViewerParams() :
            mbUseViewer(true), mViewpointX(0.f), mViewpointY(0.f), mViewpointZ(0.f), mViewpointF(0.f),
            mCameraLineWidth(1.f), mCameraSize(1.f), mGraphLineWidth(1.f), mKeyFrameLineWidth(1.f),
            mKeyFrameSize(1.f), mPointSize(1.f)
    {}

    ViewerParams::ViewerParams(const cv::FileStorage &fsSettings) : ViewerParams() {

        this->read(fsSettings["Viewer"]);
    }

    void ViewerParams::write(cv::FileStorage &fs) const {

        fs << "Viewer" << "{";

        YamlParserCV::writeBool(fs, "UseViewer", mbUseViewer);

        YamlParserCV::writeReal(fs, "KeyFrameSize", mKeyFrameSize);
        YamlParserCV::writeReal(fs, "KeyFrameLineWidth", mKeyFrameLineWidth);
        YamlParserCV::writeReal(fs, "GraphLineWidth", mGraphLineWidth);
        YamlParserCV::writeReal(fs, "PointSize", mPointSize);
        YamlParserCV::writeReal(fs, "CameraSize", mCameraSize);
        YamlParserCV::writeReal(fs, "CameraLineWidth", mCameraLineWidth);

        YamlParserCV::writeReal(fs, "ViewpointX", mViewpointX);
        YamlParserCV::writeReal(fs, "ViewpointY", mViewpointY);
        YamlParserCV::writeReal(fs, "ViewpointZ", mViewpointZ);
        YamlParserCV::writeReal(fs, "ViewpointF", mViewpointF);

        fs << "}";
    }

    void ViewerParams::read(const cv::FileNode &viewerNode) {

        mbUseViewer = YamlParserCV::readBool(viewerNode, "UseViewer", mbUseViewer);

        mKeyFrameSize = YamlParserCV::readReal<float>(viewerNode, "KeyFrameSize", mKeyFrameSize);
        mKeyFrameLineWidth = YamlParserCV::readReal<float>(viewerNode, "KeyFrameLineWidth", mKeyFrameLineWidth);
        mGraphLineWidth = YamlParserCV::readReal<float>(viewerNode, "GraphLineWidth", mGraphLineWidth);
        mPointSize = YamlParserCV::readReal<float>(viewerNode, "PointSize", mPointSize);
        mCameraSize = YamlParserCV::readReal<float>(viewerNode, "CameraSize", mCameraSize);
        mCameraLineWidth = YamlParserCV::readReal<float>(viewerNode, "CameraLineWidth", mCameraLineWidth);

        mViewpointX = YamlParserCV::readReal<float>(viewerNode, "ViewpointX", mViewpointX);
        mViewpointY = YamlParserCV::readReal<float>(viewerNode, "ViewpointY", mViewpointY);
        mViewpointZ = YamlParserCV::readReal<float>(viewerNode, "ViewpointZ", mViewpointZ);
        mViewpointF = YamlParserCV::readReal<float>(viewerNode, "ViewpointF", mViewpointF);
    }

    std::string ViewerParams::printStr(const std::string &prefix) const {

        ostringstream oss;

        string useViewerStr = "Yes";
        if (!mbUseViewer)
            useViewerStr = "No";
        oss << prefix << "Use Viewer? " << useViewerStr << endl;

        oss << prefix << "Viewer:\n";
        //oss << prefix << "Frame time: " << mT << endl;
        oss << prefix << "View point X: " << mViewpointX << endl;
        oss << prefix << "View point Y: " << mViewpointY << endl;
        oss << prefix << "View point Z: " << mViewpointZ << endl;
        oss << prefix << "View point F: " << mViewpointF << endl;

        //oss << prefix << "Viewer image width: " << mImageWidth << endl;
        //oss << prefix << "Viewer image height: " << mImageHeight << endl;

        oss << prefix << "Map-Drawer: \n";
        oss << prefix << "Key frame size: " << mKeyFrameSize << endl;
        oss << prefix << "Key frame line width: " << mKeyFrameLineWidth << endl;
        oss << prefix << "Graph line width: " << mGraphLineWidth << endl;
        oss << prefix << "Point size: " << mPointSize << endl;
        oss << prefix << "Camera size: " << mCameraSize << endl;
        oss << prefix << "Camera line width: " << mCameraLineWidth << endl;

        return oss.str();
    }
} // OG_SLAM
