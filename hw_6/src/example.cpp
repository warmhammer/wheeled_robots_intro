#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// #include "example.hpp"

static const std::string OPENCV_WINDOW = "Forward camera";

class Example {

  private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_sub;
    ros::Subscriber range_front_sub;
    ros::Rate rate = ros::Rate(30);

    cv_bridge::CvImagePtr cv_ptr;

    std::vector<double> sonar_data{0, 0, 0, 0};

  public:

    Example()
    {
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        image_sub = nh.subscribe("/head/camera1/image_raw", 1, &Example::camera_cb, this);
        range_front_sub = nh.subscribe("/range/front", 1, &Example::range_front, this);

        cv::namedWindow(OPENCV_WINDOW);
    
        ros::Duration(1).sleep();
    }

    ~Example()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        show_image(cv_ptr);
    }

    void show_image(const cv_bridge::CvImagePtr cv_ptr) {
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
    }

    void range_front(const sensor_msgs::Range::ConstPtr &msg) {
        sonar_data[0] = msg->range;
    }

    void spin() {

        double t0 = ros::Time::now().toSec();
        while (nh.ok()) {
            double t = ros::Time::now().toSec() - t0;

            geometry_msgs::Twist command;

            // check if there is no obstacles in forward robot direction
            if (sonar_data[0] > 0.7) {
                command.linear.x = 0.5;
                command.linear.y = 0.0;
                command.angular.z = 0.0;
            } else {
                command.linear.x = -0.1;
                command.linear.y = 0.0;
                command.angular.z = 0.4;
                cmd_vel_pub.publish(command);
                ros::Duration(1.0).sleep();
            }

            cmd_vel_pub.publish(command);

            ros::spinOnce();
            rate.sleep();
        }
    }

};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "example_node");

    Example ex;
    ex.spin();

    return 0;
}
