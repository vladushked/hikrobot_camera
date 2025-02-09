#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
#define FIT_min_x 420
#define FIT_min_y 70
#define FIT_max_x 2450
#define FIT_max_y 2000
#endif

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    ros::Time img_time;
    unsigned long img_index;

    // string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera("~");
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("rgb", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // 就是rgb格式

    std::string node_name = ros::this_node::getName();
    if (!node_name.empty() && node_name[0] == '/')
        node_name.erase(0, 1);

    //********** 10 Hz        **********/
    ros::Rate loop_rate(1000);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src, img_time, img_index);

        if (src.empty())
        {
            // std::cout << "Изображение пустое" << std::endl;
            continue;
        }
        
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = img_time; // Используем время захвата камеры
        image_msg.header.frame_id = node_name;
        image_msg.height = src.rows;
        image_msg.width = src.cols;
        image_msg.encoding = "bgr8";

        camera_info_msg.header.seq = img_index;
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        camera_info_msg.height = image_msg.height;
        camera_info_msg.width = image_msg.width;
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    return 0;
}
