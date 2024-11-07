#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
// #include <glob.h>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <string>

//namespace fs = std::filesystem;

// 定义瓦片结构
struct Tile {
    std::pair<double, double> top_left;
    std::pair<double, double> bottom_right;
};

// 定义函数原型
std::pair<int, int> find_tile_for_gps(double lon, double lat, const std::vector<std::vector<Tile>>& coordinates);
cv::Mat rotate_image_and_mask(const cv::Mat& image, double angle);
cv::Mat create_mask(const cv::Mat& gray);
std::vector<std::string> list_images_sorted_by_name(const std::string& directory);
std::pair<double, double> pixel_to_gps(int center_x, int center_y, const std::pair<int, int>& tile_index, const std::vector<std::vector<Tile>>& coordinates, const cv::Size& img_shape);   
std::pair<double, double> template_demo(cv::Mat template_img, double angle, std::pair<double, double> last_gps);
