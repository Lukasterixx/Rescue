#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <filesystem>
#include <fstream>
#include <curl/curl.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Configuration
const std::string CAMERA_IP = "192.168.144.25";
const int WEB_PORT = 82;
const std::string VIDEO_PATH = "/photo/100SIYI_VID/";
const std::string BASE_URL = "http://" + CAMERA_IP + ":" + std::to_string(WEB_PORT) + VIDEO_PATH;

namespace fs = std::filesystem;

// --- Libcurl Write Callback ---
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// --- Libcurl File Write Callback ---
size_t FileWriteCallback(void* ptr, size_t size, size_t nmemb, FILE* stream) {
    return fwrite(ptr, size, nmemb, stream);
}

// --- Fetch HTML Content ---
std::string fetchURL(const std::string& url) {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L); // 10s timeout for directory listing
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            std::cerr << "[Error] Failed to fetch URL: " << url << " (" << curl_easy_strerror(res) << ")" << std::endl;
            return "";
        }
    }
    return readBuffer;
}

// --- Download File ---
bool downloadFile(const std::string& url, const std::string& output_path) {
    CURL* curl;
    FILE* fp;
    CURLcode res;

    curl = curl_easy_init();
    if (curl) {
        fp = fopen(output_path.c_str(), "wb");
        if (!fp) {
            std::cerr << "[Error] Cannot open file for writing: " << output_path << std::endl;
            return false;
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, FileWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L); // Enable progress meter
        
        // Basic progress output
        std::cout << "   > Downloading..." << std::flush;

        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        fclose(fp);

        if (res == CURLE_OK) {
            std::cout << " Done." << std::endl;
            return true;
        } else {
            std::cerr << " Failed: " << curl_easy_strerror(res) << std::endl;
            // Remove partial file
            fs::remove(output_path); 
            return false;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    // Silence unused parameter warnings
    (void)argc;
    (void)argv;

    // 1. Determine Storage Directory
    std::string share_dir;
    try {
        share_dir = ament_index_cpp::get_package_share_directory("go2_control_cpp");
    } catch (...) {
        std::cerr << "[Error] Could not find package share directory." << std::endl;
        return 1;
    }
    
    std::string storage_base = share_dir + "/photos/";
    
    // Ensure base directory exists
    if (!fs::exists(storage_base)) {
        fs::create_directories(storage_base);
    }

    std::cout << "--- SIYI Video Downloader ---" << std::endl;
    std::cout << "Target URL: " << BASE_URL << std::endl;
    std::cout << "Save Path:  " << storage_base << "<id>/" << std::endl;

    // 2. Fetch File List
    std::string html_content = fetchURL(BASE_URL);
    if (html_content.empty()) return 1;

    // 3. Parse Filenames (Simple Regex for href="*.mp4")
    std::regex mp4_regex("href=\"([^\"]+\\.mp4)\"");
    std::sregex_iterator next(html_content.begin(), html_content.end(), mp4_regex);
    std::sregex_iterator end;

    std::vector<std::string> video_files;
    while (next != end) {
        std::smatch match = *next;
        video_files.push_back(match[1].str());
        next++;
    }

    if (video_files.empty()) {
        std::cout << "[Info] No MP4 files found on camera." << std::endl;
        return 0;
    }

    std::cout << "[Info] Found " << video_files.size() << " video files." << std::endl;

    // 4. Download Loop
    for (const auto& filename : video_files) {
        // Logic: Save to "downloads" subfolder
        std::string folder_name = "downloads";
        std::string target_dir = storage_base + folder_name;
        
        if (!fs::exists(target_dir)) fs::create_directories(target_dir);

        std::string local_path = target_dir + "/" + filename;
        std::string remote_url = BASE_URL + filename;

        // Skip if file exists and has size > 0
        if (fs::exists(local_path) && fs::file_size(local_path) > 0) {
            std::cout << "[Skip] " << filename << " (Already exists)" << std::endl;
            continue;
        }

        std::cout << "[Get] " << filename << " -> " << folder_name << "/" << std::endl;
        downloadFile(remote_url, local_path);
    }

    std::cout << "--- All Operations Complete ---" << std::endl;
    return 0;
}