
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <iostream>
int main(int argc, char* argv[]) {
    
    while(1) {
        //std::cout << "rosbag record --duration=1h -O bag_" +  std::to_string(time(NULL)) + " stitchData/images stitchData/points " << std::endl;
        system(("rosbag record --duration=350 -O bag_" +  std::to_string(time(NULL)) + " /zed2i/zed_node/rgb_raw/image_raw_color /zed2i/zed_node/point_cloud/cloud_registered /os_cloud_node/points /reach_ros_node").c_str());
        sleep(300);
    }
}
