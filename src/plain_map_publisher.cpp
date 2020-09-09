#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



#include "quadtree.h"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void publish_plain_image(cv::Mat img);

//ros::Publisher map_pub;
ros::Publisher pub_plain_image;

int main(int argc, char **argv){

    ros::init(argc, argv, "plain_image_publisher");
    ros::NodeHandle n;

    //map_pub = n.advertise<nav_msgs::OccupancyGrid>("plain_map",10);
    pub_plain_image = n.advertise<sensor_msgs::Image>("/plain_map_publisher/plain_image", 1000);
    ROS_INFO("Publishing: %s", pub_plain_image.getTopic().c_str());
    ros::Subscriber map_sub = n.subscribe("map",1000,mapCallback);

    ros::spin();
    

    return 0;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::cout << "check" << std::endl;


    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    ROS_INFO("Got map %d %d", info.width, info.height);
    
    Map map(info.width, info.height);
    cv::Mat plain_mat = cv::Mat::zeros(info.height,info.width,CV_32F);
    for (unsigned int x = 0; x < info.width; x++)
        for (unsigned int y = 0; y < info.height; y++) {
            map.Insert(Cell(x,y,info.width,msg->data[x+ info.width * y]));
        
            if((int)(msg->data[x+ info.width * y]) == 100) {
                plain_mat.at<float>((int)y,(int)x) = 1.0;
            }
            else if((int)(msg->data[x+ info.width * y]) == 0) {
                plain_mat.at<float>((int)y,(int)x) = 0.5;
            }
            //std::cout << "data[" << x+ info.width * y << "] : " << (int)(msg->data[x+ info.width * y]) << std::endl;
        
        }

    // nav_msgs::OccupancyGrid* newGrid = map.Grid();
    // newGrid->header = header;
    // newGrid->info = info;
    ros::Rate loop_rate(1000);
    while(ros::ok()) {
    //     map_pub.publish(*newGrid);
    //     cv::imshow("img",plain_mat);
    //     cv::waitKey(10);
        publish_plain_image(plain_mat);
        loop_rate.sleep();
    }
}

void publish_plain_image(cv::Mat img) {

    // Create our message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // sensor_msgs::ImagePtr exl_msg = cv_bridge::CvImage(header, "mono16", img).toImageMsg();

    cv_bridge::CvImage out_msg;
    out_msg.header   = header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image    = img;

    // Publish
    pub_plain_image.publish(out_msg.toImageMsg());
    img.release();
}
