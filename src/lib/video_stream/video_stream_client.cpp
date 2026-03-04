#include "edu_camera/video_stream/video_stream_client.hpp"
#include <iostream>
#include <chrono>

namespace edu_camera {
namespace video_stream {

VideoStreamClient::VideoStreamClient()
  : _is_initialized(false)
{

}

VideoStreamClient::~VideoStreamClient()
{
  shutdown();
}

bool VideoStreamClient::initialize()
{
  if (_is_initialized) {
    return true;
  }
  
  _is_initialized = true;
  return true;
}

void VideoStreamClient::shutdown()
{
  if (!_is_initialized) {
    return;
  }
  _is_initialized = false;
}



} // namespace video_stream
} // namespace edu_camera
