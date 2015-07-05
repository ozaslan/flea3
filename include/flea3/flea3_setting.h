#ifndef FLEA3_SETTING_H_
#define FLEA3_SETTING_H_

#include <flycapture/FlyCapture2.h>

#define PGERROR(err, msg) flea3::HandleError(err, msg, __func__)

namespace flea3 {

using namespace FlyCapture2;

void HandleError(const Error& error, const std::string& message = "",
                 const std::string& func_name = "");

void PrintPropertyInfo(const PropertyInfo& prop_info,
                       const std::string& prop_name);
void PrintProperty(const Property& prop, const std::string& prop_name);

std::string BayerFormatToEncoding(const BayerTileFormat& bayer_format);
std::string PixelFormatToEncoding(unsigned bits_per_pixel);

PropertyInfo GetPropertyInfo(Camera& camera, const PropertyType& prop_type);
Property GetProperty(Camera& camera, const PropertyType& prop_type);
std::pair<Format7Info, bool> GetFormat7Info(Camera& camera, const Mode& mode);
Mode GetFirstFormat7Mode(Camera& camera);
CameraInfo GetCameraInfo(Camera& camera);
float GetCameraFrameRate(Camera& camera);
float GetCameraTemperature(Camera& camera);
FrameRate GetMaxFrameRate(Camera& camera, const VideoMode& video_mode);
std::pair<VideoMode, FrameRate> GetVideoModeAndFrameRate(Camera& camera);

void SetProperty(Camera& camera, const PropertyType& prop_type, bool& auto_on,
                 double& value);
void SetProperty(Camera& camera, const PropertyType& prop_type, double& value);

unsigned ReadRegister(Camera& camera, unsigned address);
void WriteRegister(Camera& camera, unsigned address, unsigned value);

void EnableMetadata(Camera& camera);

bool IsAutoWhiteBalanceSupported(Camera& camera);
bool IsFormat7Supported(Camera& camera);
std::pair<Format7PacketInfo, bool> IsFormat7SettingsValid(
    Camera& camera, const Format7ImageSettings& fmt7_settings);
bool IsVideoModeSupported(Camera& camera, const VideoMode& video_mode);
bool IsVideoModeAndFrameRateSupported(Camera& camera,
                                      const VideoMode& video_mode,
                                      const FrameRate& frame_rate);

}  // namespace flea3

#endif  // FLEA3_SETTING_H_