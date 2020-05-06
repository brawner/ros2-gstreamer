#ifndef __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_
#define __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ros_formats {

constexpr char RGB8[] = "rgb8";
constexpr char RGBA8[] = "rgba8";
constexpr char BGR8[] = "bgr8";
constexpr char BGRA8[] = "bgra8";
constexpr char MONO8[] = "mono8";
constexpr char MONO16[] = "mono16";

constexpr char RGB16[] = "rgb16";
constexpr char RGBA16[] = "rgba16";
constexpr char BGR16[] = "bgr16";
constexpr char BGRA16[] = "bgra16";

// OpenCV CvMat types
constexpr char TYPE_8UC1[] = "8UC1";
constexpr char TYPE_8UC2[] = "8UC2";
constexpr char TYPE_8UC3[] = "8UC3";
constexpr char TYPE_8UC4[] = "8UC4";
constexpr char TYPE_8SC1[] = "8SC1";
constexpr char TYPE_8SC2[] = "8SC2";
constexpr char TYPE_8SC3[] = "8SC3";
constexpr char TYPE_8SC4[] = "8SC4";
constexpr char TYPE_16UC1[] = "16UC1";
constexpr char TYPE_16UC2[] = "16UC2";
constexpr char TYPE_16UC3[] = "16UC3";
constexpr char TYPE_16UC4[] = "16UC4";
constexpr char TYPE_16SC1[] = "16SC1";
constexpr char TYPE_16SC2[] = "16SC2";
constexpr char TYPE_16SC3[] = "16SC3";
constexpr char TYPE_16SC4[] = "16SC4";
constexpr char TYPE_32SC1[] = "32SC1";
constexpr char TYPE_32SC2[] = "32SC2";
constexpr char TYPE_32SC3[] = "32SC3";
constexpr char TYPE_32SC4[] = "32SC4";
constexpr char TYPE_32FC1[] = "32FC1";
constexpr char TYPE_32FC2[] = "32FC2";
constexpr char TYPE_32FC3[] = "32FC3";
constexpr char TYPE_32FC4[] = "32FC4";
constexpr char TYPE_64FC1[] = "64FC1";
constexpr char TYPE_64FC2[] = "64FC2";
constexpr char TYPE_64FC3[] = "64FC3";
constexpr char TYPE_64FC4[] = "64FC4";

// Miscellaneous
// YUV 4:2:2 encodings with an 8-bit depth
// UYUV version: http://www.fourcc.org/pixel-format/yuv-uyvy
constexpr char YUV422[] = "yuv422";
// YUYV version: http://www.fourcc.org/pixel-format/yuv-yuy2/
constexpr char YUV422_YUY2[] = "yuv422_yuy2";

constexpr char JPEG[] = "jpeg";
constexpr char PNG[] = "png";
}  // namespace ros_formats

// https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c
namespace gst_formats {

constexpr char I420[] = "I420";
constexpr char YV12[] = "YV12";
constexpr char YUY2[] = "YUY2";
constexpr char YVYU[] = "YVYU";
constexpr char UYVY[] = "UYVY";
constexpr char AYUV[] = "AYUV";

constexpr char RGBx[] = "RGBx";
constexpr char BGRx[] = "BGRx";
constexpr char xRGB[] = "xRGB";
constexpr char xBGR[] = "xBGR";
constexpr char RGBA[] = "RGBA";
constexpr char BGRA[] = "BGRA";
constexpr char ARGB[] = "ARGB";
constexpr char ABGR[] = "ABGR";
constexpr char RGB[] = "RGB";
constexpr char BGR[] = "BGR";

constexpr char Y41B[] = "Y41B";
constexpr char Y42B[] = "Y42B";
constexpr char Y444[] = "Y444";
constexpr char V210[] = "v210";
constexpr char V216[] ="v216";
constexpr char NV12[] = "NV12";
constexpr char NV21[] = "NV21";

constexpr char GRAY8[] = "GRAY8";
constexpr char GRAY16_BE[] = "GRAY16_BE";
constexpr char GRAY16_LE[] = "GRAY16_LE";
constexpr char Y16[] = "Y16";
constexpr char V308[] = "V308";
constexpr char IYU2[] = "IYU2";
constexpr char RGB16[] = "RGB16";
constexpr char BGR16[] = "BGR16";
constexpr char RGB15[] = "RGB15";
// etc, the list goes on

constexpr char IMAGE_PNG[] = "image/png";
constexpr char IMAGE_JPEG[] = "image/jpeg";

}  // namespace gst_formats

const std::map<std::string, std::string> ros_gst_conversions = {
    {ros_formats::RGB8, gst_formats::RGB},
    {ros_formats::RGBA8, gst_formats::RGBA},
    {ros_formats::BGR8, gst_formats::BGR},
    {ros_formats::BGRA8, gst_formats::RGBA},
    {ros_formats::MONO8, gst_formats::GRAY8},
    {ros_formats::TYPE_8UC1, gst_formats::GRAY8},
    {ros_formats::MONO16, gst_formats::GRAY16_LE},
    {ros_formats::TYPE_16UC1, gst_formats::GRAY16_LE},
    {ros_formats::YUV422, gst_formats::UYVY},
    {ros_formats::YUV422_YUY2, gst_formats::YUY2},
};

const std::map<std::string, std::string> gst_ros_conversions = {
    {gst_formats::RGB, ros_formats::RGB8},
    {gst_formats::RGBA, ros_formats::RGBA8},
    {gst_formats::BGR, ros_formats::BGR8},
    {gst_formats::RGBA, ros_formats::BGRA8},
    {gst_formats::GRAY8, ros_formats::MONO8},
    {gst_formats::GRAY16_BE, ros_formats::MONO16},
    {gst_formats::GRAY16_LE, ros_formats::MONO16},
    {gst_formats::UYVY, ros_formats::YUV422},
    {gst_formats::YUY2, ros_formats::YUV422_YUY2},
};

const std::map<std::string, std::string> gst_ros_compressed_conversions = {
  {gst_formats::IMAGE_PNG, ros_formats::PNG},
  {gst_formats::IMAGE_JPEG, ros_formats::JPEG},
};

const std::map<std::string, std::string> ros_gst_compressed_conversions = {
  {ros_formats::PNG, gst_formats::IMAGE_PNG},
  {ros_formats::JPEG, gst_formats::IMAGE_JPEG},
};


class EncodingConversions {
public:
  static std::vector<std::string> supported_gst_encodings() {
    std::vector<std::string> encodings;
    for (auto entry : gst_ros_conversions) {
      encodings.push_back(entry.first.c_str());
    }
    // encodings.push_back(gst_formats::IMAGE_PNG);
    // encodings.push_back(gst_formats::IMAGE_JPEG);
    return encodings;
  }

  static std::vector<std::string> supported_ros_encodings() {
    std::vector<std::string> encodings;
    for (auto entry : ros_gst_conversions) {
      encodings.push_back(entry.first.c_str());
    }
    // encodings.push_back(ros_formats::PNG);
    // encodings.push_back(ros_formats::JPEG);
    return encodings;
  }

  static std::vector<std::string> supported_gst_compressed_encodings() {
    return {gst_formats::IMAGE_PNG, gst_formats::IMAGE_JPEG};
  }

  static std::vector<std::string> supported_ros_compressed_encodings() {
    return {ros_formats::PNG, ros_formats::JPEG};
  }

  static std::string gst_to_ros(std::string gst_format) {
    auto result = gst_ros_conversions.find(gst_format);
    if (result != gst_ros_conversions.end()) {
      return result->second;
    }

    auto compressed_result = gst_ros_compressed_conversions.find(gst_format);
    if (compressed_result != gst_ros_compressed_conversions.end()) {
      return result->second;
    }

    std::cout << "GST format '" << gst_format << "' not found." << std::endl;
    for (auto entry : gst_ros_conversions) {
      std::cout << "\t" << entry.first << " -> " << entry.second << std::endl;
    }

    return "";
  }

  static std::string ros_to_gst(std::string ros_format) {
    auto result = ros_gst_conversions.find(ros_format);
    if (result != ros_gst_conversions.end()) {
      return result->second;
    }

    auto compressed_result = ros_gst_compressed_conversions.find(ros_format);
    if (compressed_result != ros_gst_compressed_conversions.end()) {
      return result->second;
    }

    std::cout << "ROS format '" << ros_format << "' not found." << std::endl;
    for (auto entry : ros_gst_conversions) {
      std::cout << "\t" << entry.first << " -> " << entry.second << std::endl;
    }

    return "";
  }
};

#endif  // __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_
