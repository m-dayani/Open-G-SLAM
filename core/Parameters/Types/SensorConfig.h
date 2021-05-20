//
// Created by root on 5/10/21.
//

#ifndef OG_SLAM_SENSORCONFIG_H
#define OG_SLAM_SENSORCONFIG_H

#include <memory>
#include <string>

#include "YamlParserCV.h"


namespace OG_SLAM {

    class SensorConfig {
    public:
        enum SensorConfigEnum {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2,
            IMU_MONOCULAR = 3,
            IMU_STEREO = 4,
            EVENT_ONLY = 5,
            EVENT_MONO = 6,
            EVENT_IMU = 7,
            EVENT_IMU_MONO = 8,
            IDLE = 9
        };

        SensorConfig() : mSensor(IDLE) {}
        explicit SensorConfig(SensorConfigEnum sConf) : mSensor(sConf) {}
        explicit SensorConfig(const cv::FileStorage& fs);
        explicit SensorConfig(const cv::FileNode& dsNode);

        static SensorConfigEnum mapConfig(const std::string &dsConfig);
        static std::string mapConfig(SensorConfigEnum dsConfig);

        // Convert Sensor config. types
        std::string toStr() const;
        std::string toDsStr() const;

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);
        void read(const cv::FileStorage& fs);

        // Sensor configurations:
        // 3 camera config.
        bool isMonocular() const;
        bool isStereo() const;
        bool isRGBD() const;
        // 3 types of sensors
        bool isImage() const;
        bool isEvent() const;
        bool isInertial() const;
        bool isEventOnly() const;
        bool isImageOnly() const;

        bool operator==(SensorConfigEnum sConf);
        bool operator!=(SensorConfigEnum sConf);

        SensorConfigEnum getConfig() const { return this->mSensor; }
        void setConfig(SensorConfigEnum sConf) { this->mSensor = sConf; }

    private:
        SensorConfigEnum mSensor;

    };

    typedef std::shared_ptr<SensorConfig> SensorConfigPtr;

    //These write and read functions are required by OpenCV
    static void write(cv::FileStorage& fs, const std::string&, const SensorConfig& sConf) {

        sConf.write(fs);
    }
    static void read(const cv::FileNode& node, SensorConfig& sConf, const SensorConfig& default_value = SensorConfig()) {

        if(node.empty())
            sConf.setConfig(default_value.getConfig());
        else
            sConf.read(node);
    }

} // OG_SLAM


#endif //OG_SLAM_SENSORCONFIG_H
