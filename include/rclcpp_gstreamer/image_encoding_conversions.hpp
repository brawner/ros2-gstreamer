#ifndef __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_
#define __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_

namespace ros_encodings {

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
}

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
constexpr char RBG[] = "RGB";
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
}

const std::map<std::string, std::string> ros_gst_conversions = {
    {ros_format::RGB8, gst_format::RGB},
    {ros_format::RGBA8, gst_format::RGBA},
    {ros_format::BGR8, gst_format::BGR},
    {ros_format::BGRA8, gst_format::RGBA},
    {ros_format::MONO8, gst_format::GRAY8},
    {ros_format::MONO16, gst_format::GRAY16},
    {ros_format::YUV422, gst_format::UYVY},
    {ros_format::YUV422_YUY2, gst_format::YUY2},
};

const std::map<std::string, std::string> gst_ros_conversions = {
    {gst_format::RGB, ros_format::RGB8},
    {gst_format::RGBA, ros_format::RGBA8},
    {gst_format::BGR, ros_format::BGR8},
    {gst_format::RGBA, ros_format::BGRA8},
    {gst_format::GRAY8, ros_format::MONO8},
    {gst_format::GRAY16, ros_format::MONO16},
    {gst_format::UYVY, ros_format::YUV422},
    {gst_format::YUY2, ros_format::YUV422_YUY2},
};

class EncodingConversions {
public:
  static std::vector<std::string> supported_gst_encodings() {
    std::vector<std::string> encodings(gst_ros_conversions.size());
    for (auto entry : gst_ros_conversions) {
      encodings.push_back(entry.first);
    }
    return encodings;
  }

  static std::vector<std::string> supported_ros_encodings() {
    std::vector<std::string> encodings(ros_gst_encodings.size());
    for (auto entry : ros_gst_encodings) {
      encodings.push_back(entry.first);
    }
    return encodings;
  }

  static std::optional<std::string> gst_to_ros(const std::string& gst_format) {
    auto result = gst_ros_conversions.find(gst_format);
    if (result == gst_ros_conversions.end()) {
      return {};
    }
    return *result;
  }

  static std::optional<std::string> ros_to_gst(const std::string& ros_format) {
    auto result = ros_gst_conversions.find(ros_format);
    if (result == ros_gst_conversions.end()) {
      return {};
    }
    return *result;
  }
}

#endif  // __ROS2_GSTREAMER__IMAGE_ENCODING_CONVERSIONS_HPP_
