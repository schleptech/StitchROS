#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/foreach.hpp>

/**Definition for Point Type*/
typedef pcl::PointXYZ PointT;

/**Definition for Point Cloud Type*/
typedef pcl::PointCloud<PointT> PointCloudT;
int imgCnt = 0;
int pcCnt = 0;
void PCCallback(const PointCloudT::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    //std::cout << "Received a Point Cloud" << std::endl;
    //std::cout << "Width: " << msg->width << std::endl;
    //std::cout << "Size: " << msg->size() << std::endl;
    //std::cout << "Points: " << msg->points.size() << std::endl;
    //pcl::io::savePCDFile(std::string("Cloud-") + std::to_string((pcCnt++)) + std::string(".pcd"), *msg,true);
    std::cout << "Received a Point Cloud" << std::endl;
   
}


void ImgCallback(const sensor_msgs::ImageConstPtr& msg) {
    std::cout << "Received an Image" << std::endl;
    cv::imwrite("SubscribedImg-" + std::to_string(imgCnt++) + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imgSub = it.subscribe("/zed2i/zed_node/rgb_raw/image_raw_color", 1, ImgCallback);
    
    

    ros::spin();

    return 0;
}