#include "geomagic_touch_x/device.hpp"

namespace geomagic_touch_x {
void Device::open(const std::string &device_name) {
  if (is_open_)
    return;
  // Initialize haptic device
  // USB 연결 지원: 빈 device_name 또는 "HD_DEFAULT_DEVICE"는 HD_DEFAULT_DEVICE (NULL) 사용
  const char* device_config = nullptr;
  if (device_name.empty() || device_name == "HD_DEFAULT_DEVICE") {
    device_config = HD_DEFAULT_DEVICE;  // HD_DEFAULT_DEVICE는 NULL로 정의됨
  } else {
    device_config = device_name.c_str();
  }
  hHD_ptr_ = std::make_unique<HHD>(hdInitDevice(device_config));
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    // 더 자세한 오류 정보 출력
    char errorString[256];
    snprintf(errorString, sizeof(errorString), 
             "Failed to initialize haptic device. Error code: %d", error.errorCode);
    hduPrintError(stderr, &error, "Device initialization error");
    throw std::runtime_error(errorString);
  }
  // Enable force output, i.e. all motors are turned on.
  if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
    hdEnable(HD_FORCE_OUTPUT);
  } else {
    throw std::runtime_error("Failed to enable force output.");
  }
  // Setup device state
  for (int i = 0; i < 3; ++i)
    state_.cmd_force[i] = 0.;
  // Start state publisher
  hdScheduleAsynchronous(this->on_device_state_, &state_, HD_DEFAULT_SCHEDULER_PRIORITY);
  // Start the scheduler
  hdStartScheduler();
  is_open_ = true;
}

void Device::close() {
  if (!is_open_)
    return;
  hdStopScheduler();
  hdDisable(HD_FORCE_OUTPUT);
  hdDisableDevice(*hHD_ptr_);
  hHD_ptr_.reset();
  is_open_ = false;
}

HDCallbackCode HDCALLBACK Device::on_device_state_(void *data) {
  // Setup
  State *state = (State *)data;

  // Begin frame
  hdBeginFrame(hdGetCurrentDevice());

  // Get device state
  hdGetDoublev(HD_CURRENT_POSITION, state->position);
  hdGetDoublev(HD_CURRENT_VELOCITY, state->velocity);
  hdGetDoublev(HD_CURRENT_TRANSFORM, state->transform);
  hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, state->angular_velocity);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state->joint_angles);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, state->gimbal_angles);
  hdGetDoublev(HD_CURRENT_FORCE, state->force);
  hdGetDoublev(HD_CURRENT_TORQUE, state->torque);
  hdGetDoublev(HD_CURRENT_JOINT_TORQUE, state->joint_torque);
  hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, state->gimbal_torque);

  // Set device force
  hdSetDoublev(HD_CURRENT_FORCE, state->cmd_force);

  // End frame
  hdEndFrame(hdGetCurrentDevice());

  // Check for error
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback\n");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  return HD_CALLBACK_CONTINUE;
}
} // namespace geomagic_touch_x