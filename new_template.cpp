#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
// #include <glob.h>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <string> 
namespace fs = std::filesystem;

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

std::pair<int, int> find_tile_for_gps(double lon, double lat, const std::vector<std::vector<Tile>>& coordinates) {
    for (int i = 0; i < coordinates.size(); ++i) {
        for (int j = 0; j < coordinates[i].size(); ++j) {
            const auto& tile = coordinates[i][j];
            if (tile.top_left.first <= lon && lon <= tile.bottom_right.first &&
                tile.bottom_right.second <= lat && lat <= tile.top_left.second) {
                return {i, j};
            }
        }
    }
    return {-1, -1}; // 返回 -1, -1 表示没有找到
}

cv::Mat rotate_image_and_mask(const cv::Mat& image, double angle) {
    cv::Mat rotated;
    cv::Point2f center(image.cols / 2.0F, image.rows / 2.0F);
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(image, rotated, rotationMatrix, image.size());
    return rotated;
}

cv::Mat create_mask(const cv::Mat& gray) {
    cv::Mat mask;
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
    return mask;
}

std::vector<std::string> list_images_sorted_by_name(const std::string& directory) {
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (filename.find(".png") != std::string::npos || 
                filename.find(".jpg") != std::string::npos ||
                filename.find(".jpeg") != std::string::npos ||
                filename.find(".bmp") != std::string::npos ||
                filename.find(".gif") != std::string::npos ||
                filename.find(".tiff") != std::string::npos) {
                files.push_back(entry.path().string());
            }
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::pair<double, double> pixel_to_gps(int center_x, int center_y, const std::pair<int, int>& tile_index, const std::vector<std::vector<Tile>>& coordinates, const cv::Size& img_shape) {
    if (tile_index.first >= 0 && tile_index.second >= 0) {
        const auto& tile = coordinates[tile_index.first][tile_index.second];
        double tile_width_gps = tile.bottom_right.first - tile.top_left.first;
        double tile_height_gps = tile.top_left.second - tile.bottom_right.second;
        double gps_x = tile.top_left.first + (center_x / img_shape.width) * tile_width_gps;
        double gps_y = tile.top_left.second - (center_y / img_shape.height) * tile_height_gps;
        return {gps_x, gps_y};
    }
    return {-1, -1}; // 返回 -1, -1 表示无效
}


int main() {
    // 填充 coordinates 数据
    std::vector<std::vector<Tile>> coordinates = {
        {
            {{109.15612488985062, 38.60411435365677}, {109.1717728972435, 38.593712747097015}},
            {{109.1717728972435, 38.60411435365677}, {109.18742090463638, 38.593712747097015}},
            {{109.18742090463638, 38.60411435365677}, {109.20306891202927, 38.593712747097015}},
            {{109.20306891202927, 38.60411435365677}, {109.21871691942215, 38.593712747097015}},
            {{109.21871691942215, 38.60411435365677}, {109.23436492681503, 38.593712747097015}},
            {{109.23436492681503, 38.60411435365677}, {109.25001293420792, 38.593712747097015}}
        },
        {
            {{109.15612488985062, 38.593712747097015}, {109.1717728972435, 38.58331114053726}},
            {{109.1717728972435, 38.593712747097015}, {109.18742090463638, 38.58331114053726}},
            {{109.18742090463638, 38.593712747097015}, {109.20306891202927, 38.58331114053726}},
            {{109.20306891202927, 38.593712747097015}, {109.21871691942215, 38.58331114053726}},
            {{109.21871691942215, 38.593712747097015}, {109.23436492681503, 38.58331114053726}},
            {{109.23436492681503, 38.593712747097015}, {109.25001293420792, 38.58331114053726}}
        },
        {
            {{109.15612488985062, 38.58331114053726}, {109.1717728972435, 38.57290953397751}},
            {{109.1717728972435, 38.58331114053726}, {109.18742090463638, 38.57290953397751}},
            {{109.18742090463638, 38.58331114053726}, {109.20306891202927, 38.57290953397751}},
            {{109.20306891202927, 38.58331114053726}, {109.21871691942215, 38.57290953397751}},
            {{109.21871691942215, 38.58331114053726}, {109.23436492681503, 38.57290953397751}},
            {{109.23436492681503, 38.58331114053726}, {109.25001293420792, 38.57290953397751}}
        },
        {
            {{109.15612488985062, 38.57290953397751}, {109.1717728972435, 38.562507927417755}},
            {{109.1717728972435, 38.57290953397751}, {109.18742090463638, 38.562507927417755}},
            {{109.18742090463638, 38.57290953397751}, {109.20306891202927, 38.562507927417755}},
            {{109.20306891202927, 38.57290953397751}, {109.21871691942215, 38.562507927417755}},
            {{109.21871691942215, 38.57290953397751}, {109.23436492681503, 38.562507927417755}},
            {{109.23436492681503, 38.57290953397751}, {109.25001293420792, 38.562507927417755}}
        },
        {	
           {{109.15612488985062, 38.562507927417755}, {109.1717728972435, 38.552106320858}},
           {{109.1717728972435, 38.562507927417755}, {109.18742090463638, 38.552106320858}},
           {{109.18742090463638, 38.562507927417755}, {109.20306891202927, 38.552106320858}},
           {{109.20306891202927, 38.562507927417755}, {109.21871691942215, 38.552106320858}},
           {{109.21871691942215,38.562507927417755}, {109.23436492681503, 38.552106320858}},
           {{109.23436492681503, 38.562507927417755}, {109.25001293420792, 38.552106320858}}
       },
       {
           {{109.15612488985062, 38.552106320858}, {109.1717728972435, 38.54170471429825}},
           {{109.1717728972435, 38.552106320858}, {109.18742090463638, 38.54170471429825}},
           {{109.18742090463638, 38.552106320858}, {109.20306891202927, 38.54170471429825}},
           {{109.20306891202927, 38.552106320858}, {109.21871691942215, 38.54170471429825}},
           {{109.21871691942215, 38.552106320858}, {109.23436492681503, 38.54170471429825}},
           {{109.23436492681503, 38.552106320858}, {109.25001293420792, 38.54170471429825}}
       }
    };

    std::string directory_path = "data/1014_img";
    std::vector<std::string> sorted_images = list_images_sorted_by_name(directory_path);
    std::string map_dir = "data/output_tif";

    std::pair<double, double> last_gps = {109.22046882, 38.56923820};
    int i = 0;
    for (const auto& tpl_path : sorted_images) {
        auto tile_index = find_tile_for_gps(last_gps.first, last_gps.second, coordinates);
        if (tile_index.first != -1) {
            int row = tile_index.first;
            int col = tile_index.second;
            std::string tile_filename = "tile_" + std::to_string(row) + "_" + std::to_string(col) + ".jpg";
            std::string tile_path = map_dir + "/" + tile_filename;
            cv::Mat img = cv::imread(tile_path, cv::IMREAD_GRAYSCALE);
            cv::Mat img2 = img.clone();

            cv::Mat template_img = cv::imread(tpl_path, cv::IMREAD_GRAYSCALE);
            double angle = -15;
            template_img = rotate_image_and_mask(template_img, angle); // 假设这个函数已定义

            double ex = 0.6;
            cv::resize(template_img, template_img, cv::Size(template_img.cols * ex, template_img.rows * ex));
            cv::Mat mask = create_mask(template_img); // 假设这个函数已定义

            std::vector<cv::TemplateMatchModes> methods = {cv::TM_CCORR_NORMED};
            for (const auto& meth : methods) {
                cv::Mat res;
                // 注意这里的改变，我们创建了一个cv::Mat来接收结果
                if (meth == cv::TM_SQDIFF || meth == cv::TM_SQDIFF_NORMED) {
                    cv::matchTemplate(img, template_img, res, meth, mask);
                } else {
                    cv::matchTemplate(img, template_img, res, meth, mask);
                }

                double min_val, max_val;
                cv::Point min_loc, max_loc;
                cv::minMaxLoc(res, &min_val, &max_val, &min_loc, &max_loc);

                cv::Point top_left;
                cv::Point bottom_right;
                if (meth == cv::TM_SQDIFF || meth == cv::TM_SQDIFF_NORMED) {
                    top_left = min_loc;
                } else {
                    top_left = max_loc;
                }
                bottom_right = cv::Point(top_left.x + template_img.cols, top_left.y + template_img.rows);

                cv::rectangle(img2, top_left, bottom_right, cv::Scalar(255), 2);

                cv::imwrite("output/1.jpg", img2);
                int center_x = (top_left.x + bottom_right.x) / 2;
                int center_y = (top_left.y + bottom_right.y) / 2;
                auto center_gps = pixel_to_gps(center_x, center_y, tile_index, coordinates, img2.size()); 
                last_gps = center_gps;
                if (center_gps.first != -1) {
                    std::cout << "中心GPS坐标：(" << center_gps.first << ", " << center_gps.second << ")" << std::endl;
                }
                std::string output_filename = "./output/detected_" + std::to_string(meth) + ".jpg";
                cv::imwrite(output_filename, img2);
            }
        } else {
            std::cout << "未找到对应的小地图。" << std::endl;
            continue;
        }
    }
    return 0;
}