#include "ros/ros.h"
#include "depthai/depthai.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/subscriber_filter.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv4/opencv2/opencv.hpp"

#include <deque>

#define OAKD_YOLO_MONITOR_FONTSIZE 0.5
#define OAKD_YOLO_MONITOR_FONTPOS_X 0.25
#define OAKD_YOLO_MONITOR_FONTPOS_Y 10.0
#define OAKD_YOLO_MONITOR_HUMAN_ID 0
#define OAKD_YOLO_MONITOR_QUEUE_SIZE 15
#define OAKD_YOLO_MONITOR_DENOMINATOR 0.066666667

namespace oakd_ros
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, depthai_ros_msgs::SpatialDetectionArray> DepthAiSyncPolicy;

    class OakdYoloMonitor
    {
    private:
        const std::vector<std::string> labels
        {
            "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
            "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
            "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
            "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
            "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
            "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
            "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
            "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
            "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
            "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
            "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
            "teddy bear",     "hair drier", "toothbrush"
        };

        image_transport::ImageTransport image_transport; //ImageTransport has to be a member. Unlike from NodeHandle.
        image_transport::SubscriberFilter img_subscriber;
        message_filters::Subscriber<depthai_ros_msgs::SpatialDetectionArray> detect_subscriber;
        ros::Publisher all_publisher;
        ros::Publisher human_publisher;
        ros::Publisher single_human_publisher;
        image_transport::Publisher img_publisher;
        message_filters::Synchronizer<DepthAiSyncPolicy> synchronizer;
        geometry_msgs::Point sum;
        std::deque<geometry_msgs::Point> human_point_queue;
    
    public:
        OakdYoloMonitor(ros::NodeHandle& node_handle) :
            image_transport(node_handle),
            img_subscriber(
                image_transport,
                "/oakd_yolov4/color/image",
                1
            ),
            detect_subscriber(
                node_handle,
                "/oakd_yolov4/color/yolov4_spatial_detections",
                1
            ),
            all_publisher(
                node_handle.advertise<geometry_msgs::PoseArray>(
                    "detected_all_pose_array",
                    1,
                    false
                )
            ),
            human_publisher(
                node_handle.advertise<geometry_msgs::PoseArray>(
                    "human_pose_array",
                    1,
                    false
                )
            ),
            single_human_publisher(
                node_handle.advertise<geometry_msgs::PoseStamped>(
                    "calibrated_single_pose",
                    1,
                    false
                )
            ),
            img_publisher(
                image_transport.advertise(
                    "image_labeled",
                    1
                )
            ),
            synchronizer(
                DepthAiSyncPolicy(10),
                img_subscriber,
                detect_subscriber
            )
        { 
            synchronizer.registerCallback(
                boost::bind(
                    &OakdYoloMonitor::synchronizedCallback,
                    this,   
                    _1,
                    _2
                )
            );

            human_point_queue.resize(OAKD_YOLO_MONITOR_QUEUE_SIZE);
        }
        ~OakdYoloMonitor()
        { }

        void synchronizedCallback(const sensor_msgs::ImageConstPtr& img_msg,
            const depthai_ros_msgs::SpatialDetectionArrayConstPtr& detected_msg);
    };

    void OakdYoloMonitor::synchronizedCallback(const sensor_msgs::ImageConstPtr& img_msg,
        const depthai_ros_msgs::SpatialDetectionArrayConstPtr& detected_msg)
    {
        ros::Time now(ros::Time::now());
        geometry_msgs::PoseArray all_pose_msg;
        all_pose_msg.header.stamp = now;
        all_pose_msg.header.frame_id = detected_msg->header.frame_id;
        geometry_msgs::PoseArray human_pose_msg;
        human_pose_msg.header.stamp = now;
        human_pose_msg.header.frame_id = detected_msg->header.frame_id;
        geometry_msgs::PoseStamped single_human_msg;
        single_human_msg.header.stamp = now;
        single_human_msg.header.frame_id = detected_msg->header.frame_id;
        geometry_msgs::Quaternion identity;
        identity.x = 0.0;
        identity.y = 0.0;
        identity.z = 0.0;
        identity.w = 1.0;
        cv_bridge::CvImagePtr cv_img_ptr(
            cv_bridge::toCvCopy(
                img_msg
            )
        );

        unsigned int count_detected(detected_msg->detections.size());
        double shortest_dist(DBL_MAX);
        int nearest_human_idx(-1);
        for (int i = 0; i < count_detected; i++)
        {
            geometry_msgs::Pose pose;
            pose.orientation = identity;
            pose.position.x = detected_msg->detections[i].position.x;
            ////////////////////////////////////////////////////////
            // todo:
            // i think putting minus wrongly looked better 
            // just because bounding box got wider in vertical actually.
            // if so, this coordinate system become RUF.
            // so, in this case, is fixing boundig box to 1:1
            // make this better or not ??
            pose.position.y = -detected_msg->detections[i].position.y;
            pose.position.z = detected_msg->detections[i].position.z;

            all_pose_msg.poses.push_back(pose);
            unsigned int id(detected_msg->detections[i].results[0].id);

            if(id == OAKD_YOLO_MONITOR_HUMAN_ID)
            {
                human_pose_msg.poses.push_back(pose);

                double x(detected_msg->detections[i].position.x);
                double y(detected_msg->detections[i].position.y);
                double dist_sqr(x * x + y * y);
                if(dist_sqr < shortest_dist)
                {
                    shortest_dist = dist_sqr;
                    nearest_human_idx = i;
                }
            }

            double width_x(detected_msg->detections[i].bbox.size_x * 0.5);
            double width_y(detected_msg->detections[i].bbox.size_y * 0.5);
            cv::Scalar color(255.0);

            cv::rectangle(
                cv_img_ptr->image,
                cv::Rect(
                    cv::Point(
                        detected_msg->detections[i].bbox.center.x - width_x,
                        detected_msg->detections[i].bbox.center.y - width_y
                    ),
                    cv::Point(
                        detected_msg->detections[i].bbox.center.x + width_x,
                        detected_msg->detections[i].bbox.center.y + width_y
                    )
                ),
                color
            );
            cv::putText(
                cv_img_ptr->image,
                labels[id],
                cv::Point(
                    detected_msg->detections[i].bbox.center.x - width_x + OAKD_YOLO_MONITOR_FONTPOS_X,
                    detected_msg->detections[i].bbox.center.y - width_y + OAKD_YOLO_MONITOR_FONTPOS_Y
                ),
                cv::FONT_HERSHEY_DUPLEX,
                OAKD_YOLO_MONITOR_FONTSIZE,
                color
            );

            ROS_INFO(
                "got detection [%d] => {%d: %s} <score: %lf>",
                i,
                id,
                labels[id].c_str(),
                detected_msg->detections[i].results[0].score
            );  
        }

        geometry_msgs::Point temp_point(human_point_queue.front());
        sum.x -= temp_point.x;
        sum.y -= temp_point.y;
        sum.z -= temp_point.z;

        human_point_queue.pop_front();
        if(nearest_human_idx >= 0)
        {
            temp_point = detected_msg->detections[nearest_human_idx].position;
            human_point_queue.emplace_back(temp_point);
            sum.x += temp_point.x;
            sum.y += temp_point.y;
            sum.z += temp_point.z;
        }
        else
        {
            human_point_queue.emplace_back();
        }

        single_human_msg.pose.position.x = sum.x * OAKD_YOLO_MONITOR_DENOMINATOR;
        single_human_msg.pose.position.y = sum.y * OAKD_YOLO_MONITOR_DENOMINATOR;
        single_human_msg.pose.position.z = sum.z * OAKD_YOLO_MONITOR_DENOMINATOR;
        single_human_publisher.publish(single_human_msg);

        img_publisher.publish(cv_img_ptr->toImageMsg());
        all_publisher.publish(all_pose_msg);
        human_publisher.publish(human_pose_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "oakd_yolov4_monitor"
    );

    ros::NodeHandle node_handle("oakd_yolov4_monitor");
    oakd_ros::OakdYoloMonitor monitor(node_handle);

    ros::spin();

    return 0;
}

