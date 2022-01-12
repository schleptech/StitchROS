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

//using namespace ouster;

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

int main(int argc, char* argv[]) {
    PointCloudT::Ptr outputScan(new PointCloudT);
    //TODO: If there's an argument, use that for the hostname instead of the environment
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
    const std::string sensor_hostname = getenv("OUSTER");


    const std::string data_destination = "";

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

    auto handle = ouster::sensor::init_client(sensor_hostname, data_destination);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = ouster::sensor::get_metadata(*handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    ouster::sensor::sensor_info info = ouster::sensor::parse_metadata(metadata);

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

    // A LidarScan holds lidar data for an entire rotation of the device
    //std::vector<ouster::LidarScan> scans{N_SCANS, ouster::LidarScan{w, h}};
    ouster::LidarScan scan{w, h};

    // A ScanBatcher can be used to batch packets into scans
    ouster::sensor::packet_format pf = ouster::sensor::get_format(info);
    ouster::ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);


    for(int i = 0; i < 8; i++) {
    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cerr << "Capturing points... ";

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

    /**Get Point Data*/
    //for (int i = 0; i < N_SCANS;) {
        // wait until sensor data is available
        ouster::sensor::client_state st = ouster::sensor::poll_client(*handle);

        // check for error status
        if (st & ouster::sensor::CLIENT_ERROR)
            FATAL("Sensor client returned error state!");

        // check for lidar data, read a packet and add it to the current batch
        if (st & ouster::sensor::LIDAR_DATA) {
            if (!ouster::sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                FATAL("Failed to read a packet of the expected size!");

            // batcher will return "true" when the current scan is complete
            if (batch_to_scan(packet_buf.get(), scan)) {
                // LidarScan provides access to azimuth block data and headers
                auto n_invalid = std::count_if(
                    scan.headers.begin(), scan.headers.end(),
                    [](const ouster::LidarScan::BlockHeader& h) {
                        return h.status != 0xffffffff;
                    });
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (n_invalid <= (int)w - column_window_length);// i++;
            }
        }

        // check if IMU data is available (but don't do anything with it)
        if (st & ouster::sensor::IMU_DATA) {
            ouster::sensor::read_imu_packet(*handle, packet_buf.get(), pf);
        }
    //}
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
    ouster::XYZLut lut = ouster::make_xyz_lut(info);
    //std::vector<ouster::LidarScan::Points> clouds;
    ouster::LidarScan::Points cloud;
    //for (const ouster::LidarScan& scan : scans) {
        // compute a point cloud using the lookup table
        //clouds.push_back(ouster::cartesian(scan, lut));
        cloud = ouster::cartesian(scan, lut);

        // channel fields can be queried as well
        auto n_returns = (scan.field(ouster::LidarScan::Field::RANGE) != 0).count();

        std::cerr << "  Frame no. " << scan.frame_id << " with " << n_returns
                  << " returns" << std::endl;
    //}

    /*
     * Write output to CSV files. The output can be viewed in a point cloud
     * viewer like CloudCompare:
     *
     * [0] https://github.com/cloudcompare/cloudcompare
     */
    std::cerr << "Writing files... " << std::endl;

    int file_ind = 0;
    std::string file_base{"cloud_"};
    //for (const ouster::LidarScan::Points& cloud : clouds) {
        //std::string filename = file_base + std::to_string(file_ind++) + ".csv";
        //std::ofstream out;
        //out.open(filename);
        //out << std::fixed << std::setprecision(4);
        //PointCloudT::Ptr outputScan(new PointCloudT);

        //Send Messages


        // write each point, filtering out points without returns
        outputScan->header.frame_id = "Test_Scan";
        outputScan->width = cloud.rows();
        
        for (int i = 0; i < cloud.rows(); i++) {
            auto xyz = cloud.row(i);
            outputScan->points.push_back(PointT(xyz(0), xyz(1), xyz(2)));
            if (!xyz.isApproxToConstant(0.0)) {
                //std::cout << "X: " << xyz(0) << ", ";
                //std::cout << "Y: " << xyz(1) << ", ";
                //std::cout << "Z: " << xyz(2) << " " << std::endl;
                //out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;  
            }
                
        }
        
        pcl::io::savePCDFile(std::string("Cloud-") + std::to_string((file_ind++)) + std::string(".pcd"), *outputScan,true);
        
    //}
    }

    std::cout << "Publishing Cloud" << std::endl;

    return 0;

    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloudT>("points2", 1);
    ros::Rate loopRate(4);

    while(nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), outputScan->header.stamp);
        pub.publish(outputScan);
        ros::spinOnce();
        loopRate.sleep();
    }
    //

    return EXIT_SUCCESS;
}
