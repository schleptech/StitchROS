#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ouster/build.h>
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>

using namespace ouster;

/**Definition for Point Type*/
typedef pcl::PointXYZ PointT;

/**Definition for Point Cloud Type*/
typedef pcl::PointCloud<PointT> PointCloudT;

const int N_SCANS = 1;
const size_t UDP_BUF_SIZE = 65536;

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}


class PointCloudMessenger {
    
    // the topic to publish at, will be overwritten to give the remapped name
    std::string cloud_topic;
    // source file name, will be overwritten to produce actually configured file
    std::string file_name;
    // republish interval in seconds
    double interval;
    // tf2 frame_id
    std::string frame_id;
    // latched topic enabled/disabled
    bool latch;
    // pointcloud message and publisher
    sensor_msgs::PointCloud2 cloud;
    ros::Publisher pub;
    // timer to handle republishing
    ros::Timer timer;

    

    void timer_callback(ros::TimerEvent const&) {
        // just re-publish
        publish();
    }

public:
    ros::NodeHandle nh;
    PointCloudMessenger(std::string topic, std::string fn, double intvl, std::string f_id, bool lth) {
        cloud_topic = topic;
        file_name = fn;
        interval = intvl;
        frame_id = f_id;
        latch = lth;
    }

    void publish() {
        ROS_DEBUG_STREAM_ONCE("Publishing pointcloud");
        ROS_DEBUG_STREAM_ONCE(" * number of points: " << cloud.width * cloud.height);
        ROS_DEBUG_STREAM_ONCE(" * frame_id: " << cloud.header.frame_id);
        ROS_DEBUG_STREAM_ONCE(" * topic_name: " << cloud_topic);
        int num_subscribers = pub.getNumSubscribers();
        if (num_subscribers > 0) {
            ROS_DEBUG("Publishing data to %d subscribers.", num_subscribers);
        }
        // update timestamp and publish
        cloud.header.stamp = ros::Time::now();
        pub.publish(cloud);
    }
  
    bool try_load_pointcloud() {
        if (file_name.empty()) {
            ROS_ERROR_STREAM("Can't load pointcloud: no file name provided");
            return false;
        }
        else if (pcl::io::loadPCDFile(file_name, cloud) < 0) {
            ROS_ERROR_STREAM("Failed to parse pointcloud from file ('" << file_name << "')");
            return false;
        }
        // success: set frame_id appropriately
        cloud.header.frame_id = frame_id;
        return true;
    }

    void init_run() {
        // init publisher
        pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, latch);
        //std::cout << "Advertised"  << std::endl;
        // treat publishing once as a special case to interval publishing
        //bool oneshot = interval <= 0;
        //timer = nh.createTimer(ros::Duration(interval),  &PointCloudMessenger::timer_callback, this, oneshot);
        //std::cout << "Timered"  << std::endl;
    }

    void print_config_info() {
        ROS_INFO_STREAM("Recognized the following parameters");
        ROS_INFO_STREAM(" * file_name: " << file_name);
        ROS_INFO_STREAM(" * interval: " << interval);
        ROS_INFO_STREAM(" * frame_id: " << frame_id);
        ROS_INFO_STREAM(" * topic_name: " << cloud_topic);
        ROS_INFO_STREAM(" * latch: " << std::boolalpha << latch);
    }

    void print_data_info() {
        ROS_INFO_STREAM("Loaded pointcloud with the following stats");
        ROS_INFO_STREAM(" * number of points: " << cloud.width * cloud.height);
        ROS_INFO_STREAM(" * total size [bytes]: " << cloud.data.size());
        ROS_INFO_STREAM(" * channel names: " << pcl::getFieldsList(cloud));
    }
};
std::ofstream logFile;
void log(std::string msg) {
    logFile.open("Point Cloud Log.txt", std::ofstream::app);
    logFile << msg << std::endl;
    logFile.close();
}

int main(int argc, char* argv[]) {
   logFile.open("Point Cloud Log.txt", std::ofstream::out);
   logFile.close();

    log("Running Point Cloud Publisher\n");

    /*if (argc != 3) {
        std::cerr << "Version: " << ouster::CLIENT_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: ouster_client_example <sensor_hostname> "
                     "<data_destination_ip>"
                  << std::endl;

        return EXIT_FAILURE;
    }*/
    std::cerr << "Ouster client example " << ouster::CLIENT_VERSION
              << std::endl;
    /*
     * The sensor client consists of the network client and a library for
     * reading and working with data.
     *
     * The network client supports reading and writing a limited number of
     * configuration parameters and receiving data without wo/oprking directly with
     * the socket APIs. See the `client.h` for more details. The minimum
     * required parameters are the sensor hostname/ip and the data destination
     * hostname/ip.
     */
    log("Getting Hostname");
    const std::string sensor_hostname = "192.168.1.201";
    

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";
    
    log("Connecting to Sensor\n");
    auto handle = sensor::init_client(sensor_hostname, "");
    if (!handle) {
        
        log("Failed to Connect");
    
        FATAL("Failed to connect");
    }
    std::cerr << "ok" << std::endl;
    
    log("Connected to Sensor\n");
    
    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    sensor::sensor_info info = sensor::parse_metadata(metadata);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    // azimuth_window config param reduce the amount of valid columns per scan
    // that we will receive
    int column_window_length =
        (column_window.second - column_window.first + w) % w + 1;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;

    ros::init(argc, argv, "pub_pcl");
    PointCloudMessenger pcm = PointCloudMessenger("stitchData/points", "cloudMessage.pcd", 0.1, "zed2i_left_camera_frame", false);
    pcm.init_run();
    int stamp =  0;
    bool yUp = true;
    double yTran = 0;
    while(pcm.nh.ok()) {
        
        PointCloudT::Ptr outputScan(new PointCloudT);
        // A LidarScan holds lidar data for an entire rotation of the device
        std::vector<LidarScan> scans{N_SCANS, LidarScan{w, h}};

        // A ScanBatcher can be used to batch packets into scans
        sensor::packet_format pf = sensor::get_format(info);
        ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

        /*
        * The network client provides some convenience wrappers around socket APIs
        * to facilitate reading lidar and IMU data from the network. It is also
        * possible to configure the sensor offline and read data directly from a
        * UDP socket.
        */
        std::cerr << "Capturing points... ";

        // buffer to store raw packet data
        std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

        for (int i = 0; i < N_SCANS;) {
            // wait until sensor data is available
            sensor::client_state st = sensor::poll_client(*handle);

            // check for error status
            if (st & sensor::CLIENT_ERROR)
                FATAL("Sensor client returned error state!");

            // check for lidar data, read a packet and add it to the current batch
            if (st & sensor::LIDAR_DATA) {
                if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                    FATAL("Failed to read a packet of the expected size!");

                // batcher will return "true" when the current scan is complete
                if (batch_to_scan(packet_buf.get(), scans[i])) {
                    // LidarScan provides access to azimuth block data and headers
                    auto n_invalid = std::count_if(
                        scans[i].headers.begin(), scans[i].headers.end(),
                        [](const LidarScan::BlockHeader& h) {
                            return h.status != 0xffffffff;
                        });
                    // retry until we receive a full set of valid measurements
                    // (accounting for azimuth_window settings if any)
                    if (n_invalid <= (int)w - column_window_length) i++;
                }
            }

            // check if IMU data is available (but don't do anything with it)
            if (st & sensor::IMU_DATA) {
                sensor::read_imu_packet(*handle, packet_buf.get(), pf);
            }
        }
        std::cerr << "ok" << std::endl;

        /*
        * The example code includes functions for efficiently and accurately
        * computing point clouds from range measurements. LidarScan data can also
        * be accessed directly using the Eigen[0] linear algebra library.
        *
         * [0] http://eigen.tuxfamily.org
        */
        std::cerr << "Computing point clouds... " << std::endl;

        // pre-compute a table for efficiently calculating point clouds from range
        XYZLut lut = ouster::make_xyz_lut(info);
        std::vector<LidarScan::Points> clouds;

        for (const LidarScan& scan : scans) {
            // compute a point cloud using the lookup table
            clouds.push_back(ouster::cartesian(scan, lut));

            // channel fields can be queried as well
            auto n_returns = (scan.field(LidarScan::Field::RANGE) != 0).count();

            std::cerr << "  Frame no. " << scan.frame_id << " with " << n_returns << " returns" << std::endl;
        }

        /*
        * Write output to CSV files. The output can be viewed in a point cloud
        * viewer like CloudCompare:
        *
        * [0] https://github.com/cloudcompare/cloudcompare
        */
        std::cerr << "Writing files... " << std::endl;

        int file_ind = 0;
        std::string file_base{"cloud_"};
        for (const LidarScan::Points& cloud : clouds) {
            // write each point, filtering out points without returns
            for (int i = 0; i < cloud.rows(); i++) {
                auto xyz = cloud.row(i);       
                if (!xyz.isApproxToConstant(0.0)) outputScan->points.push_back(PointT(xyz(0), xyz(1)-0.08, xyz(2)));
            }
        }
        
        pcl::io::savePCDFile("cloudMessage.pcd", *outputScan,true);
        if (!pcm.try_load_pointcloud()) return -1;
        
        pcm.init_run();
        pcm.publish();
        //ros::spin();
        //sleep(3);
    }
    return EXIT_SUCCESS;
}
