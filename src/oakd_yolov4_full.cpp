#include "oakd_yolov4/device_initializer_full.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "oakd_yolov4_full"
    );

    ros::NodeHandle node_handle("~");
    std::vector<oakd_ros::DataOutputQueuePtr> image_data_streams;
    std::vector<oakd_ros::DataOutputQueuePtr> nnet_data_streams;
    std::vector<oakd_ros::DataOutputQueuePtr> imu_data_streams;

    oakd_ros::OakdYoloDeviceInitializer initializer(
        node_handle,
        "/home/marsh/mlros_ws/src/oakd_yolov4/oakd_yolov4_ros/resource/tiny-yolo-v4_openvino_2021.2_6shave.blob"
    );
    if(initializer.isReady())
    {
        initializer.initDevice(
            image_data_streams,
            nnet_data_streams,
            imu_data_streams,
            true,
            false,
            30.0f,
            200u
        );
    }
    else
    {
        ROS_ERROR("initializer is not ready. stopping node.");
        return 1;
    }

    dai::rosBridge::ImageConverter rgb_converter(
        initializer.getRGBFrameName(),
        false
    );
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgb_publisher(
        image_data_streams[OAKD_IMAGE_STREAMS_PREVIEW],
        node_handle,
        "color/image",
        std::bind(
            &dai::rosBridge::ImageConverter::toRosMsg,
            &rgb_converter,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        1,
        initializer.getColorURI(),
        "color"
    );

    dai::rosBridge::ImageConverter depth_converter(
        initializer.getDepthFrameName(),
        true
    );
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depth_publisher(
        image_data_streams[OAKD_IMAGE_STREAMS_DEPTH],
        node_handle,
        "stereo/depth",
        std::bind(
            &dai::rosBridge::ImageConverter::toRosMsg,
            &depth_converter,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        1,
        initializer.getStereoURI(),
        "stereo"
    );

    dai::rosBridge::SpatialDetectionConverter detection_converter(
        initializer.getRGBFrameName(),
        416,
        416,
        false
    );
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detection_publisher(
        nnet_data_streams[OAKD_NNET_STREAMS_DETECTIONS],
        node_handle,
        "color/yolov4_spatial_detections",
        std::bind(
            static_cast<void(dai::rosBridge::SpatialDetectionConverter::*)
                (std::shared_ptr<dai::SpatialImgDetections>,
                 depthai_ros_msgs::SpatialDetectionArray&)
            >(&dai::rosBridge::SpatialDetectionConverter::toRosMsg),
            &detection_converter,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        1
    );

    dai::rosBridge::ImuConverter imu_converter(
        initializer.getIMUFrameName()
    );
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imu_publisher(
        imu_data_streams[OAKD_IMU_STREAMS_IMU],
        node_handle,
        "imu",
        std::bind(
            &dai::rosBridge::ImuConverter::toRosMsg,
            &imu_converter,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        1
    );

    depth_publisher.addPubisherCallback();
    //depth_publisher.startPublisherThread();
    //detection_publisher.addPubisherCallback();
    detection_publisher.startPublisherThread();
    //rgb_publisher.addPubisherCallback();
    rgb_publisher.startPublisherThread();
    //imu_publisher.addPubisherCallback();
    imu_publisher.startPublisherThread();

    ros::spin();

    return 0;
}

