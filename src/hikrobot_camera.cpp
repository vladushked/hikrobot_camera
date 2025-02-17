#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// Если требуется обрезка изображения (чтобы удалить лишние пиксели), задаём макрос
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
    // Инициализация ROS-нод
    ros::init(argc, argv, "hikrobot_dual_camera");
    ros::NodeHandle nh("~"); // приватный NodeHandle для получения параметров

    // Получение серийных номеров для левой и правой камер из параметров ROS
    std::string left_serial, right_serial;
    nh.param("left_serial", left_serial, std::string("left_serial"));
    nh.param("right_serial", right_serial, std::string("right_serial"));

    // Создание отдельных пространств имён для левой и правой камер и установка параметра серийного номера

    // Инициализация камер (конструктор камеры должен читать параметр "serial" из своего NodeHandle)
    camera::Camera left_camera(nh, left_serial);
    camera::Camera right_camera(nh, right_serial);

    // Инициализация image_transport для публикации изображений
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher left_pub = it.advertiseCamera("/camera/left/image_raw", 1);
    image_transport::CameraPublisher right_pub = it.advertiseCamera("/camera/right/image_raw", 1);

    sensor_msgs::Image left_image_msg;
    sensor_msgs::Image right_image_msg;
    sensor_msgs::CameraInfo left_camera_info_msg;
    sensor_msgs::CameraInfo right_camera_info_msg;

    cv_bridge::CvImagePtr left_cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_bridge::CvImagePtr right_cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    left_cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    right_cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

    ros::Rate loop_rate(2);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        // Получаем изображение с левой камеры
        cv::Mat left_src;
        ros::Time left_img_time;
        unsigned long left_img_index;
        // Получаем изображение с правой камеры
        cv::Mat right_src;
        ros::Time right_img_time;
        unsigned long right_img_index;

        if (!left_camera.FrameEmpty() && !right_camera.FrameEmpty())
        {
            left_camera.ReadImg(left_src, left_img_time, left_img_index);
            right_camera.ReadImg(right_src, right_img_time, right_img_index);

            left_cv_ptr->image = left_src;
            right_cv_ptr->image = right_src;

            left_image_msg = *(left_cv_ptr->toImageMsg());
            left_image_msg.header.stamp = left_img_time;
            left_image_msg.header.frame_id = "left_camera_frame";
            left_image_msg.height = left_src.rows;
            left_image_msg.width = left_src.cols;
            left_image_msg.encoding = "bgr8";
            left_camera_info_msg.header.seq = left_img_index;
            left_camera_info_msg.header.frame_id = left_image_msg.header.frame_id;
            left_camera_info_msg.header.stamp = left_image_msg.header.stamp;
            left_camera_info_msg.height = left_image_msg.height;
            left_camera_info_msg.width = left_image_msg.width;

            right_image_msg = *(right_cv_ptr->toImageMsg());
            right_image_msg.header.stamp = right_img_time;
            right_image_msg.header.frame_id = "right_camera_frame";
            right_image_msg.height = right_src.rows;
            right_image_msg.width = right_src.cols;
            right_image_msg.encoding = "bgr8";
            right_camera_info_msg.header.seq = right_img_index;
            right_camera_info_msg.header.frame_id = right_image_msg.header.frame_id;
            right_camera_info_msg.header.stamp = right_image_msg.header.stamp;
            right_camera_info_msg.height = right_image_msg.height;
            right_camera_info_msg.width = right_image_msg.width;

            left_pub.publish(left_image_msg, left_camera_info_msg);
            right_pub.publish(right_image_msg, right_camera_info_msg);
        }
    }
    return 0;
}
