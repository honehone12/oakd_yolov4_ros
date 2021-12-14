#ifndef OAKD_YOLO_DEVICE_INITIALIZER_H_
#define OAKD_YOLO_DEVICE_INITIALIZER_H_

#include "ros/ros.h"
#include "depthai/depthai.hpp"

#include <vector>

    namespace oakd_ros
    {
        typedef std::shared_ptr<dai::DataOutputQueue> DataOutputQueuePtr;
        typedef std::shared_ptr<dai::node::ColorCamera> ColorCameraNodePtr;
        typedef std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> YoloSDNNodePtr;
        typedef std::shared_ptr<dai::node::MonoCamera> MonoCameraNodePtr;
        typedef std::shared_ptr<dai::node::StereoDepth> StereoDepthNodePtr;
        typedef std::shared_ptr<dai::node::IMU> IMUNodePtr;
        typedef std::shared_ptr<dai::node::XLinkOut> XLinkOutNodePtr;

        class OakdYoloDeviceInitializer
        {
        private:
            std::string device_name;
            std::string camera_param_uri;
            std::string nn_path;
            std::unique_ptr<dai::Device> device_ptr;
            dai::Pipeline pipeline;

        public:
            OakdYoloDeviceInitializer(ros::NodeHandle& node_handle, const std::string& model_path) :
                nn_path(model_path)
            { 
                if(
                    !node_handle.getParam(
                        "camera_name",
                        device_name
                    )
                    ||
                    !node_handle.getParam(
                        "camera_param_uri",
                        camera_param_uri
                    )
                )
                {
                    ROS_ERROR("couldn't find parameter. check param names.");
                }
            }
            ~OakdYoloDeviceInitializer()
            { }

            bool isReady()
            {
                return device_name != "" && camera_param_uri != "" && nn_path != "";
            }

            std::string getColorURI()
            {
                return camera_param_uri + "/" + "color.yaml";
            }

            std::string getStereoURI()
            {
                return camera_param_uri + "/" + "right.yaml";
            }

            std::string getRGBFrameName()
            {
                return device_name + "_rgb_camera_optical_frame";
            }

            // as examples
            std::string getDepthFrameName()
            {
                return device_name + "_right_camera_optical_frame";
            }

            std::string getIMUFrameName()
            {
                return device_name + "_imu_frame";
            }

            void initDevice(
                std::vector<DataOutputQueuePtr>& image_streams_buffer,
                std::vector<DataOutputQueuePtr>& nnet_streams_buffer,
                std::vector<DataOutputQueuePtr>& imu_streams_buffer,
                bool sync_nn,
                bool use_subpixel,
                bool use_lr_check,
                float camera_fps,
                uint32_t imu_fps
            );
        };

        void OakdYoloDeviceInitializer::initDevice(
                std::vector<DataOutputQueuePtr>& image_streams_buffer,
                std::vector<DataOutputQueuePtr>& nnet_streams_buffer,
                std::vector<DataOutputQueuePtr>& imu_streams_buffer,
                bool sync_nn,
                bool use_subpixel,
                bool use_rl_check,
                float camera_fps,
                uint32_t imu_fps
        )
        {
            ColorCameraNodePtr color_cam_node_ptr(pipeline.create<dai::node::ColorCamera>());
            YoloSDNNodePtr yolo_sdn_node_ptr(pipeline.create<dai::node::YoloSpatialDetectionNetwork>());
            MonoCameraNodePtr mono_left_node_ptr(pipeline.create<dai::node::MonoCamera>());
            MonoCameraNodePtr mono_right_node_ptr(pipeline.create<dai::node::MonoCamera>());
            StereoDepthNodePtr stereo_depth_node_ptr(pipeline.create<dai::node::StereoDepth>());
            IMUNodePtr imu_node_ptr(pipeline.create<dai::node::IMU>());
            XLinkOutNodePtr xlink_out_rgb_ptr(pipeline.create<dai::node::XLinkOut>());
            XLinkOutNodePtr xlink_out_depth_ptr(pipeline.create<dai::node::XLinkOut>());
            XLinkOutNodePtr xlink_out_nn_ptr(pipeline.create<dai::node::XLinkOut>());
            XLinkOutNodePtr xlink_out_imu_ptr(pipeline.create<dai::node::XLinkOut>());

            xlink_out_rgb_ptr->setStreamName("preview");
            xlink_out_nn_ptr->setStreamName("detections");
            xlink_out_depth_ptr->setStreamName("depth");
            xlink_out_imu_ptr->setStreamName("imu");
            
            color_cam_node_ptr->setPreviewSize(416, 416);
            color_cam_node_ptr->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            color_cam_node_ptr->setInterleaved(false);
            color_cam_node_ptr->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
            color_cam_node_ptr->setFps(camera_fps);
            
            mono_left_node_ptr->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
            mono_left_node_ptr->setBoardSocket(dai::CameraBoardSocket::LEFT);
            mono_left_node_ptr->setFps(camera_fps);
            mono_right_node_ptr->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
            mono_right_node_ptr->setBoardSocket(dai::CameraBoardSocket::RIGHT);
            mono_right_node_ptr->setFps(camera_fps);
            
            imu_node_ptr->enableIMUSensor(
                {
                    dai::IMUSensor::ACCELEROMETER_RAW, 
                    dai::IMUSensor::GYROSCOPE_RAW,
                    dai::IMUSensor::GAME_ROTATION_VECTOR
                },
                imu_fps
            );
            imu_node_ptr->setBatchReportThreshold(1);
            imu_node_ptr->setMaxBatchReports(20);
            
            stereo_depth_node_ptr->initialConfig.setConfidenceThreshold(255);
            stereo_depth_node_ptr->setSubpixel(use_subpixel);
            stereo_depth_node_ptr->setExtendedDisparity(!use_subpixel);
            stereo_depth_node_ptr->setLeftRightCheck(use_rl_check);

            yolo_sdn_node_ptr->setBlobPath(nn_path);
            yolo_sdn_node_ptr->setConfidenceThreshold(0.5f);
            yolo_sdn_node_ptr->input.setBlocking(false);
            yolo_sdn_node_ptr->setBoundingBoxScaleFactor(0.5f);
            yolo_sdn_node_ptr->setDepthLowerThreshold(100u);
            yolo_sdn_node_ptr->setDepthUpperThreshold(5000u);

            yolo_sdn_node_ptr->setNumClasses(80);
            yolo_sdn_node_ptr->setCoordinateSize(4);
            yolo_sdn_node_ptr->setAnchors(
                {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319}
            );
            yolo_sdn_node_ptr->setAnchorMasks(
                {{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}}
            );
            yolo_sdn_node_ptr->setIouThreshold(0.5f);

            mono_left_node_ptr->out.link(stereo_depth_node_ptr->left);
            mono_right_node_ptr->out.link(stereo_depth_node_ptr->right);
            stereo_depth_node_ptr->depth.link(yolo_sdn_node_ptr->inputDepth);
            color_cam_node_ptr->preview.link(yolo_sdn_node_ptr->input);
            if(sync_nn)
            {
                yolo_sdn_node_ptr->passthrough.link(xlink_out_rgb_ptr->input);
                yolo_sdn_node_ptr->passthroughDepth.link(xlink_out_depth_ptr->input);
            }
            else
            {
                color_cam_node_ptr->preview.link(xlink_out_rgb_ptr->input);
                stereo_depth_node_ptr->depth.link(xlink_out_depth_ptr->input);
            }
            yolo_sdn_node_ptr->out.link(xlink_out_nn_ptr->input);
            imu_node_ptr->out.link(xlink_out_imu_ptr->input);

            device_ptr = std::make_unique<dai::Device>(pipeline);

            image_streams_buffer.push_back(device_ptr->getOutputQueue("preview", 1, false));
            image_streams_buffer.push_back(device_ptr->getOutputQueue("depth", 1, false));
            nnet_streams_buffer.push_back(device_ptr->getOutputQueue("detections", 1, false));
            imu_streams_buffer.push_back(device_ptr->getOutputQueue("imu", 30, false));
        }
    }

#define OAKD_IMAGE_STREAMS_PREVIEW 0
#define OAKD_IMAGE_STREAMS_DEPTH 1
#define OAKD_NNET_STREAMS_DETECTIONS 0
#define OAKD_IMU_STREAMS_IMU 0

#endif

