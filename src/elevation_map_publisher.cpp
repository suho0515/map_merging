#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <Eigen/StdVector>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

std::vector<Eigen::Vector3d> get_pointcloud();
cv::Mat pointcloud_to_mat(std::vector<Eigen::Vector3d> &pointcloud);
void publish_elevation_image(cv::Mat img);

ros::Publisher pub_elevation_image;

int main(int argc, char **argv) {

    // convert pointcloud to image
    // ==============================
    std::vector<Eigen::Vector3d> pointcloud = get_pointcloud();
    // ==============================

    // convert pointcloud to image
    // ==============================
    cv::Mat img = pointcloud_to_mat(pointcloud);
    //cv::imshow("img",img);
    // ==============================

    // publish image
    // ==============================
    ros::init(argc, argv, "elevation_image_publisher");
    ros::NodeHandle nh;
    pub_elevation_image = nh.advertise<sensor_msgs::Image>("/elevation_map_publisher/elevation_image", 1000);
    ROS_INFO("Publishing: %s", pub_elevation_image.getTopic().c_str());
    
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        publish_elevation_image(img);
        loop_rate.sleep();
    }
    // ==============================

    return 0;
}



std::vector<Eigen::Vector3d> get_pointcloud()
{
    // open file
    // ==============================
    std::ifstream inFile;

    inFile.open("../result.txt");

    if(!inFile) {
        std::cerr << "Unable to open file result.txt";
        exit(1);   // call system to stop
    }
    // ==============================

    // read file and convert it to std::vector<Eigen::Vector3d>
    // ==============================
    std::vector<Eigen::Vector3d> pointcloud;
    Eigen::Vector3d p;
    std::string str;
    int cnt = 0;
    while(inFile>>str) {
        if(cnt==0) {
            p[0] = atof(str.c_str());
            cnt++;
        }
        else if(cnt==1) {
            p[1] = atof(str.c_str());
            cnt++;
        }
        else if(cnt==2) {
            p[2] = atof(str.c_str());
            cnt = 0;
            pointcloud.push_back(p);
        }
    }
    std::cout << "pointcloud.size()" << pointcloud.size() << std::endl;
    // ==============================

    return pointcloud;
}

cv::Mat pointcloud_to_mat(std::vector<Eigen::Vector3d> &pointcloud)
{
      
    // for result_pc
    float max_x = pointcloud.at(0)[0];
    float max_y = pointcloud.at(0)[1];
    float max_z = pointcloud.at(0)[2];
    float min_x = pointcloud.at(0)[0];
    float min_y = pointcloud.at(0)[1];
    float min_z = pointcloud.at(0)[2];
    
    for(int i=0; i<pointcloud.size(); i++) {
        if(max_x < pointcloud[i][0]) max_x = pointcloud[i][0];
        else if(min_x > pointcloud[i][0]) min_x = pointcloud[i][0];

        if(max_y < pointcloud[i][1]) max_y = pointcloud[i][1];
        else if(min_y > pointcloud[i][1]) min_y = pointcloud[i][1];

        if(max_z < pointcloud[i][2]) max_z = pointcloud[i][2];
        else if(min_z > pointcloud[i][2]) min_z = pointcloud[i][2];
    }

    min_z = min_z - 0.1; // to prevent min value would be 0.0
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z; 

    int width = (int)((dx+0.1)*100.0);
    width = width - (width/10)*9;
    int height = (int)((dy+0.1)*100.0);
    height = height - (height/10)*9;
    
    float normalized_min_x = ((min_x+0.05)*10.0);
    float normalized_min_y = ((min_y+0.05)*10.0);
    float normalized_min_z = ((min_z+0.05)*10.0);

    // to prevent there are double
    normalized_min_x = round(normalized_min_x);
    normalized_min_y = round(normalized_min_y);
    normalized_min_z = round(normalized_min_z); 

    cv::Mat elevation_mat = cv::Mat::zeros((int)height,(int)width,CV_32F);

    for(int i=0; i<pointcloud.size(); i++) {

        if(min_x == 0.0) min_x = 0.05;
        if(min_y == 0.0) min_y = 0.05;
        if(min_z == 0.0) min_z = 0.05;

        float x = ((pointcloud[i][0]+0.05)*10.0)-normalized_min_x;
        float y = ((pointcloud[i][1]+0.05)*10.0)-normalized_min_y;
        float z = ((pointcloud[i][2]+0.05)*10.0)-normalized_min_z;

        float f_z = ((pointcloud[i][2]-min_z)/dz)*1.0;

        // to prevent there are double
        x = round(x);
        y = round(y);
        z = round(z); 
        //std::cout << "x, y, f_z : " << x <<" "<< y <<" "<< f_z << std::endl; 
        if(elevation_mat.at<float>((int)y,(int)x) < f_z) {
            elevation_mat.at<float>((int)y,(int)x) = f_z;
        }
    }

    return elevation_mat;
}

void publish_elevation_image(cv::Mat img) {

    // Create our message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // sensor_msgs::ImagePtr exl_msg = cv_bridge::CvImage(header, "mono16", img).toImageMsg();

    cv_bridge::CvImage out_msg;
    out_msg.header   = header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image    = img;

    // Publish
    pub_elevation_image.publish(out_msg.toImageMsg());
    img.release();
}
