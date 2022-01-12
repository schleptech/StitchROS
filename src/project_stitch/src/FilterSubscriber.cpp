#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/time_sequencer.h>
#include <sensor_msgs/Image.h>


void callback(const ImageConstPtr& image, const )

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "listener");

    /*ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatters", 1000, chatterCallback);

    ros::spin();*/
}