/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <string>
#include <stdexcept>

namespace eduart {
namespace camera {
namespace video_stream {

class Codec
{
public:
  enum class Type {
    H264,
    H265,
    MJPEG,
    NV12,
    YUYV,
    BGR,
  };

  Codec() = default;
  Codec(Type type) : type_(type) {}
  Codec(const std::string& code_name) {
    if (code_name == "H264") {
      type_ = Type::H264;
    } else if (code_name == "H265") {
      type_ = Type::H265;
    } else if (code_name == "MJPEG") {
      type_ = Type::MJPEG;
    } else if (code_name == "NV12") {
      type_ = Type::NV12;
    } else if (code_name == "YUYV") {
      type_ = Type::YUYV;
    } else if (code_name == "BGR") {
      type_ = Type::BGR;
    } else {
      throw std::invalid_argument("Unsupported codec: " + code_name);
    }
  }

  Type type() const { return type_; }
  std::string to_string() const {
    switch (type_) {
      case Type::H264: return "H264";
      case Type::H265: return "H265";
      case Type::MJPEG: return "MJPEG";
      case Type::NV12: return "NV12";
      case Type::YUYV: return "YUYV";
      case Type::BGR: return "BGR";
      default: return "Unknown";
    }
  }

private:
  Type type_ = Type::BGR;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
