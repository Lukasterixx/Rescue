#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <thread>    
#include <mutex>
#include <regex>
#include <filesystem>
#include <curl/curl.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

// --- SIYI ZT6 / ZT30 Protocol Constants ---
const std::string CAMERA_IP = "192.168.144.25"; 
const int CAMERA_PORT = 37260;                  
const int WEB_PORT = 82;
// Base paths for scraping
const std::string VIDEO_PATH = "/photo/100SIYI_VID/";
const std::string PHOTO_PATH = "/photo/101SIYI_IMG/"; // Standard path for photos

// We construct full URLs dynamically
const std::string BASE_URL_VIDEO = "http://" + CAMERA_IP + ":" + std::to_string(WEB_PORT) + VIDEO_PATH;
const std::string BASE_URL_PHOTO = "http://" + CAMERA_IP + ":" + std::to_string(WEB_PORT) + PHOTO_PATH;

const uint8_t HEADER_L = 0x55;                  
const uint8_t HEADER_H = 0x66;                  
const uint8_t CTRL_NEED_ACK = 0x01;             
const uint8_t CTRL_NO_ACK   = 0x00;

// Commands
const uint8_t CMD_GIMBAL_ROT   = 0x0E;          
const uint8_t CMD_PHOTO_VIDEO  = 0x0C;          
const uint8_t CMD_SYS_STATUS   = 0x0A;          
const uint8_t CMD_SPLIT_SCREEN = 0x11;
const uint8_t CMD_THERMAL      = 0x04;          


// --- HARDWARE CONSTRAINTS ---
const double LIMIT_PITCH_MIN = -90.0; 
const double LIMIT_PITCH_MAX = 25.0;  
const double LIMIT_YAW_MIN = -270.0;  
const double LIMIT_YAW_MAX = 270.0;   

// --- USER OFFSETS ---
const double YAW_OFFSET_DEG = -90.0;

// --- Libcurl Callbacks ---
size_t WriteStringCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

size_t WriteFileCallback(void* ptr, size_t size, size_t nmemb, FILE* stream) {
    return fwrite(ptr, size, nmemb, stream);
}

class CameraDriver : public rclcpp::Node
{
public:
  CameraDriver() : Node("camera_driver")
  {
    if (!setupSocket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket. Exiting.");
      exit(1);
    }
    
    curl_global_init(CURL_GLOBAL_ALL);

    initializeCamera();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/camera_pose", 10,
      std::bind(&CameraDriver::poseCallback, this, std::placeholders::_1));

    photo_req_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/photo_request_str", 10,
      std::bind(&CameraDriver::photoCallback, this, std::placeholders::_1));

    video_req_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/record_video_str", 10,
      std::bind(&CameraDriver::videoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Camera Driver Initialized (Instant Download).");
  }

  ~CameraDriver()
  {
    if (download_thread_.joinable()) download_thread_.join();
    if (sockfd_ > 0) close(sockfd_);
    curl_global_cleanup();
  }

private:
  int sockfd_ = -1;
  struct sockaddr_in servaddr_;
  uint16_t seq_ = 0;
  bool is_recording_ = false;

  std::thread download_thread_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr photo_req_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr video_req_sub_;

  const uint16_t crc16_tab[256] = {
    0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
  };

  // --- Callbacks ---

  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double world_pitch_deg = pitch * 180.0 / M_PI;
    double world_yaw_deg   = yaw * 180.0 / M_PI;

    world_yaw_deg += YAW_OFFSET_DEG;
    double camera_pitch_deg = world_pitch_deg;
    double camera_yaw_deg   = -world_yaw_deg;

    double safe_pitch = std::clamp(camera_pitch_deg, LIMIT_PITCH_MIN, LIMIT_PITCH_MAX);
    double safe_yaw   = std::clamp(camera_yaw_deg, LIMIT_YAW_MIN, LIMIT_YAW_MAX);

    if (std::abs(safe_pitch - camera_pitch_deg) > 0.1 || 
        std::abs(safe_yaw - camera_yaw_deg) > 0.1) 
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Pose Clamped: Safe P=%.1f Y=%.1f", safe_pitch, safe_yaw);
    }

    int16_t yaw_int   = static_cast<int16_t>(safe_yaw * 10.0);
    int16_t pitch_int = static_cast<int16_t>(safe_pitch * 10.0);

    std::vector<uint8_t> payload(4);
    payload[0] = yaw_int & 0xFF;
    payload[1] = (yaw_int >> 8) & 0xFF;
    payload[2] = pitch_int & 0xFF;
    payload[3] = (pitch_int >> 8) & 0xFF;

    sendCmd(CMD_GIMBAL_ROT, payload, false);
  }

  void photoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string folder_name = msg->data; 
    RCLCPP_INFO(this->get_logger(), "Requesting Photo for folder: %s", folder_name.c_str());
    
    // 1. Take Photo
    std::vector<uint8_t> payload = {0x00};
    sendCmd(CMD_PHOTO_VIDEO, payload, true);

    // 2. Trigger Async Download (Type 0 = Photo)
    if (download_thread_.joinable()) download_thread_.detach(); 
    download_thread_ = std::thread(&CameraDriver::downloadTask, this, folder_name, 0);
  }

  void videoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string folder_name = msg->data; 

    // Safety Sync
    bool current_state = isRecording();
    if (current_state != is_recording_) {
        RCLCPP_WARN(this->get_logger(), "State Sync: Camera %s -> Node %s", 
            current_state ? "REC" : "IDLE", current_state ? "REC" : "IDLE");
        is_recording_ = current_state;
    }

    if (is_recording_) {
      // STOP
      RCLCPP_INFO(this->get_logger(), "Stopping Recording for folder: %s", folder_name.c_str());
      std::vector<uint8_t> payload = {0x02}; 
      sendCmd(CMD_PHOTO_VIDEO, payload, true);
      is_recording_ = false;

      // 2. Trigger Async Download (Type 1 = Video)
      if (download_thread_.joinable()) download_thread_.detach(); 
      download_thread_ = std::thread(&CameraDriver::downloadTask, this, folder_name, 1);

    } else {
      // START
      RCLCPP_INFO(this->get_logger(), "Starting Recording for folder: %s", folder_name.c_str());
      std::vector<uint8_t> payload = {0x02}; 
      sendCmd(CMD_PHOTO_VIDEO, payload, true);
      is_recording_ = true;
    }
  }

  // --- Unified Download Logic ---
  // type: 0 = Photo (JPG), 1 = Video (MP4)
  void downloadTask(std::string folder_name, int type)
  {
      // Minimal wait to let the file register on the FS
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // 1. Setup Paths
      std::string share_dir;
      try {
          share_dir = ament_index_cpp::get_package_share_directory("go2_control_cpp");
      } catch (...) { return; }

      std::string target_dir = share_dir + "/photos/" + folder_name;
      if (!fs::exists(target_dir)) fs::create_directories(target_dir);

      CURL* curl = curl_easy_init();
      if (!curl) return;

      std::string base_url = (type == 0) ? BASE_URL_PHOTO : BASE_URL_VIDEO;
      std::string regex_str = (type == 0) ? "href=\"([^\"]+\\.jpg)\"" : "href=\"([^\"]+\\.mp4)\"";

      // 2. Fetch File List
      std::string html_content;
      curl_easy_setopt(curl, CURLOPT_URL, base_url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteStringCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &html_content);
      curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
      
      if (curl_easy_perform(curl) != CURLE_OK) {
          RCLCPP_ERROR(this->get_logger(), "[Async] Web scrape failed.");
          curl_easy_cleanup(curl);
          return;
      }

      // 3. Parse files
      std::regex file_regex(regex_str);
      std::sregex_iterator next(html_content.begin(), html_content.end(), file_regex);
      std::sregex_iterator end;

      std::string latest_file = "";
      // Grab the VERY LAST match (assuming it is the newest)
      while (next != end) {
          latest_file = (*next)[1].str();
          next++;
      }

      if (latest_file.empty()) {
          RCLCPP_WARN(this->get_logger(), "[Async] No files found.");
          curl_easy_cleanup(curl);
          return;
      }

      // 4. Immediate Download
      std::string remote_url = base_url + latest_file;
      std::string local_path = target_dir + "/" + latest_file; 

      RCLCPP_INFO(this->get_logger(), "[Async] Downloading Latest: %s", latest_file.c_str());

      FILE* fp = fopen(local_path.c_str(), "wb");
      if (fp) {
          curl_easy_setopt(curl, CURLOPT_URL, remote_url.c_str());
          curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFileCallback);
          curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
          // Just get it.
          curl_easy_perform(curl);
          fclose(fp);
          RCLCPP_INFO(this->get_logger(), "[Async] Saved to: %s", local_path.c_str());
      } else {
          RCLCPP_ERROR(this->get_logger(), "[Async] Failed to write local file.");
      }

      curl_easy_cleanup(curl);
  }

  // --- Initialization ---
  bool setupSocket()
  {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) return false;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000; 
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(CAMERA_PORT);
    inet_pton(AF_INET, CAMERA_IP.c_str(), &servaddr_.sin_addr);
    return true;
  }

  void initializeCamera()
  {
    if (isRecording()) {
      RCLCPP_WARN(this->get_logger(), "Camera is recording. Stopping...");
      std::vector<uint8_t> stop_payload = {0x02};
      sendCmd(CMD_PHOTO_VIDEO, stop_payload, true);
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
    is_recording_ = false;

    RCLCPP_INFO(this->get_logger(), "Setting Split Screen Mode...");
    std::vector<uint8_t> split_payload = {0x00}; 
    sendCmd(CMD_THERMAL, split_payload, true);
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Initializing Gimbal to Right (90 deg)...");
    auto home_msg = std::make_shared<geometry_msgs::msg::Pose>();
    home_msg->orientation.w = 1.0; 
    poseCallback(home_msg);
  }

  bool isRecording()
  {
    std::vector<uint8_t> empty;
    std::vector<uint8_t> response;
    if (sendCmd(CMD_SYS_STATUS, empty, true, &response)) {
      if (response.size() > 11) return response[11] == 1;
    }
    return false;
  }

  // --- Protocol Helpers ---
  uint16_t calculateCRC16(const std::vector<uint8_t>& data)
  {
    uint16_t crc = 0;
    for (uint8_t b : data) {
      uint16_t temp = (crc >> 8) & 0xFF;
      crc = ((crc << 8) & 0xFFFF) ^ crc16_tab[(b ^ temp) & 0xFF];
    }
    return crc & 0xFFFF;
  }

  bool sendCmd(uint8_t cmd_id, const std::vector<uint8_t>& payload, bool wait_ack, std::vector<uint8_t>* out_response = nullptr)
  {
    seq_++;
    uint16_t data_len = payload.size();
    std::vector<uint8_t> frame;
    frame.reserve(10 + data_len);
    frame.push_back(HEADER_L);
    frame.push_back(HEADER_H);
    frame.push_back(CTRL_NEED_ACK); 
    frame.push_back(data_len & 0xFF);
    frame.push_back((data_len >> 8) & 0xFF);
    frame.push_back(seq_ & 0xFF);
    frame.push_back((seq_ >> 8) & 0xFF);
    frame.push_back(cmd_id);
    frame.insert(frame.end(), payload.begin(), payload.end());
    uint16_t crc = calculateCRC16(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    int attempts = wait_ack ? 3 : 1;
    for (int i = 0; i < attempts; ++i) {
      sendto(sockfd_, frame.data(), frame.size(), 0, (const struct sockaddr *)&servaddr_, sizeof(servaddr_));
      if (!wait_ack) return true;
      uint8_t buffer[256];
      struct sockaddr_in from;
      socklen_t len = sizeof(from);
      int n = recvfrom(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &len);
      if (n > 0) {
        if (n >= 8 && buffer[0] == HEADER_L && buffer[1] == HEADER_H && buffer[7] == cmd_id) {
          if (out_response) out_response->assign(buffer, buffer + n);
          return true;
        }
      }
    }
    return false;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriver>());
  rclcpp::shutdown();
  return 0;
}