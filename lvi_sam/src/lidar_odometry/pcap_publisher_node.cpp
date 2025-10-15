#include <rclcpp/rclcpp.hpp>
#include <pcap.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cmath>
#include "utility.h"
#include "lvi_sam_msgs/msg/cloud_info.hpp"
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

const int VLP16_PACKET_SIZE = 1206;
const int VLP16_SCANS_PER_PACKET = 384;
const float VLP16_VERTICAL_ANGLES[16] = {
    -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15
};

class PcapPublisher : public ParamServer {
public:
    PcapPublisher(){
        std::cout<<"Pcap publisher triggered"<<std::endl;

        points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (pointCloudTopic, 5);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu> (imuTopic, 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gpsTopic, 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(imageTopic, 5);
        
        cloud_msg_.header.frame_id = "velodyne";
        cloud_msg_.height = 1;
        cloud_msg_.is_dense = true;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
        modifier.setPointCloud2Fields(5,
                        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "ring", 1, sensor_msgs::msg::PointField::UINT16);
    }

    bool initializePcap() {
        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(ParamServer::pcap_file_.c_str(), errbuf);
        // RCLCPP_INFO(this->get_logger(), "pcap_file param in pcap_publisher_node: %s", ParamServer::pcap_file_.c_str());
        if (pcap_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Could not open pcap file: %s", errbuf);
            return false;
        }
        return true;
    }

    void processPackets() {
        struct pcap_pkthdr* header;
        const u_char* packet;
        int packet_count = 0;
        rclcpp::Time start_time = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Waiting for subscriber to connect to %s...", points_pub_->get_topic_name());
        rclcpp::Rate wait_rate(10);
        while (points_pub_->get_subscription_count() == 0 && rclcpp::ok()) {
            wait_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Subscriber detected. Starting packet publishing...");

        std::cout<<"process packets started"<<std::endl;

        while (int res = pcap_next_ex(pcap_, &header, &packet) >= 0) {
            if (res == 0) continue;
            
            if (header->len > 42 && packet[23] == 0x11) {
                uint16_t dest_port = (packet[36] << 8) | packet[37];
                if (dest_port == 41000) {
                    processLidarPacket(packet + 42);
                }
            }
            
            rclcpp::Duration delay = rclcpp::Duration::from_seconds((packet_count++) * 0.001);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            if(packet_count%100 == 0) {
                std::cout<< "Processed " << packet_count << " packets" << std::endl;
            }
        }
        std::cout<< "Total packets processed: " << packet_count << std::endl;
        pcap_close(pcap_);
    }

    void processMcap()
    {
        rosbag2_cpp::readers::SequentialReader reader;

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = ParamServer::mcap_file_;
        // RCLCPP_INFO(this->get_logger(), "mcap_file param in pcap_publisher_node: %s", (storage_options.uri).c_str());
        storage_options.storage_id = "mcap";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        // RCLCPP_INFO(this->get_logger(), "IMU data cdr done");

        reader.open(storage_options, converter_options);
        // RCLCPP_INFO(this->get_logger(), "IMU data open done");

        rclcpp::Serialization<sensor_msgs::msg::Imu> serializer_imu;
        rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer_gps;
        // RCLCPP_INFO(this->get_logger(), "IMU data serializer defined");

        while (reader.has_next()) {
            auto bag_message = reader.read_next();

            if (bag_message->topic_name == "/xsens/imu/data") {
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                sensor_msgs::msg::Imu imu_msg;
                serializer_imu.deserialize_message(&serialized_msg, &imu_msg);
                imu_msg.header.stamp = this->get_clock()->now();

                // RCLCPP_INFO(this->get_logger(), "Publishing IMU data");
                // imu_pub_->publish(imu_msg);

                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }

            if (bag_message->topic_name == "/xsens/gnss"){
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                sensor_msgs::msg::NavSatFix gps_msg;
                serializer_gps.deserialize_message(&serialized_msg, &gps_msg);

                gps_msg.header.stamp = this->get_clock()->now();
                gps_msg.header.frame_id = "gps";

                // RCLCPP_INFO(this->get_logger(), "Publishing GPS data");
                gps_pub_->publish(gps_msg);

                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            
        }
        RCLCPP_INFO(this->get_logger(), "MCAP data reading done");
    }

    void processImages() {
        // RCLCPP_INFO(this->get_logger(), ".h265 file being opened");
        cv::VideoCapture cap(ParamServer::image_file1);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Camera_01.h265");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), ".txt file being opened");
        std::ifstream timestamp_file(ParamServer::image_file2);
        if (!timestamp_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Camera_01.txt");
            return;
        }

        std::string line;
        int frame_id = 0;

        // RCLCPP_INFO(this->get_logger(), "entering while loop");

        while (std::getline(timestamp_file, line)) {
            cv::Mat frame;
            if (!cap.read(frame)) {
                RCLCPP_WARN(this->get_logger(), "End of video or failed to read frame");
                break;
            }

            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() < 4) continue;

            uint64_t stamp_ns = std::stoull(tokens[2]);
            rclcpp::Time ros_stamp(stamp_ns);

            std_msgs::msg::Header header;
            header.stamp = ros_stamp;
            header.frame_id = "camera";

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            // RCLCPP_INFO(this->get_logger(), "Publishing image data");
            // image_pub_->publish(*msg);

            rclcpp::sleep_for(std::chrono::milliseconds(33));
            frame_id++;
        }

        cap.release();
        timestamp_file.close();
    }

    // void processData(){
    //     processPackets();
    //     processMcap();
    //     // processImuMcap();
    //     processImages();
    // }

private:
    void processLidarPacket(const u_char* packet) {
        cloud_msg_.width = VLP16_SCANS_PER_PACKET;
        cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
        cloud_msg_.data.resize(cloud_msg_.row_step * cloud_msg_.height);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg_, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud_msg_, "ring");
        
        // Process Velodyne packet
        for (int block = 0; block < 12; ++block) {
            const u_char* block_ptr = packet + block * 100;
            uint16_t azimuth = (block_ptr[2] << 8) | block_ptr[3];
            
            for (int channel = 0; channel < 32; ++channel) {
                const u_char* channel_ptr = block_ptr + 4 + (channel * 3);
                
                uint16_t distance = (channel_ptr[0] << 8) | channel_ptr[1];
                uint8_t intensity = channel_ptr[2];
                
                // Convert to Cartesian coordinates
                float dist_m = distance * 0.002;
                float angle_rad = (azimuth / 100.0) * (M_PI / 180.0);
                float vert_angle = VLP16_VERTICAL_ANGLES[channel % 16] * (M_PI / 180.0);
                
                *iter_x = dist_m * std::cos(vert_angle) * std::cos(angle_rad);
                *iter_y = dist_m * std::cos(vert_angle) * std::sin(angle_rad);
                *iter_z = dist_m * std::sin(vert_angle);
                *iter_i = static_cast<float>(intensity);

                *iter_ring = static_cast<uint16_t>(channel);  // or channel % 16
                ++iter_ring;
                
                ++iter_x; ++iter_y; ++iter_z; ++iter_i;
            }
        }
        
        cloud_msg_.header.stamp = this->get_clock()->now();
        // RCLCPP_INFO(this->get_logger(), "Publishing point cloud with %u points", cloud_msg_.width);
        // points_pub_->publish(cloud_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    pcap_t* pcap_;
    sensor_msgs::msg::PointCloud2 cloud_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PcapPublisher>();

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> PCAP Publisher Started.\033[0m");

    if (!node->initializePcap()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize PCAP");
        return 1;
    }

    // std::thread thread([node]() {
    //     node->processData();
    // });

    std::thread lidar_thread([node]() {
        node->processPackets();
    });

    std::thread thread([node]() {
        node->processMcap();
    });

    // std::thread gps_thread([node]() {
    //     node->processGpsMcap();
    // });

    std::thread image_thread([node]() {
        node->processImages();
    });

    rclcpp::spin(node);

    lidar_thread.join();
    thread.join();
    // imu_thread.join();
    // gps_thread.join();
    image_thread.join();
        
    rclcpp::shutdown();
    return 0;
}
