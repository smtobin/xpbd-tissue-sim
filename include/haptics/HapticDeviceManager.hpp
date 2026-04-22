#ifndef __HAPTIC_DEVICE_MANAGER_HPP
#define __HAPTIC_DEVICE_MANAGER_HPP

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "common/colors.hpp"
#include <iostream>
#include <cassert>
#include <mutex>
#include <optional>
#include <map>

#include "common/types.hpp"

struct HapticDeviceOutputData
{
    HDboolean button1_state;
    HDboolean button2_state;
    hduVector3Dd device_position;
    HDdouble device_transform[16];
    HDErrorInfo error;

    HapticDeviceOutputData()
        : button1_state(false), button2_state(false), device_position(), error()
    {}
};

struct HapticDeviceInputData
{
    Vec3r device_force;
};

struct CopiedHapticDeviceOutputData
{
    Vec3r last_position;
    Vec3r position;
    Mat3r last_orientation;
    Mat3r orientation;
    bool button1_pressed;
    bool button2_pressed;
    bool stale;

    CopiedHapticDeviceOutputData()
        : position(Vec3r::Zero()), orientation(Mat3r::Identity()), button1_pressed(false), button2_pressed(false), stale(false)
    {}
};

class HapticDeviceManager
{
    public:
    explicit HapticDeviceManager();
    explicit HapticDeviceManager(const std::string& device_name);
    explicit HapticDeviceManager(const std::string& device_name1, const std::string& device_name2);
    

    ~HapticDeviceManager();

    Vec3r lastPosition(HHD handle);
    Vec3r position(HHD handle);
    Mat3r lastOrientation(HHD handle);
    Mat3r orientation(HHD handle);
    bool button1Pressed(HHD handle);
    bool button2Pressed(HHD handle);

    const std::vector<HHD>& deviceHandles() const { return _device_handles; }

    void setForce(HHD handle, const Vec3r& force);
    const Vec3r& force(HHD handle) const { return _device_forces.at(handle); }

    void setForceScaling(Real scaling) { _force_scaling_factor = scaling; }

    private:
    static HDCallbackCode HDCALLBACK _updateCallback(void *data);

    // void copyInputState(HHD handle);
    void copyOutputState(HHD handle);
    void setStale(const bool stale) { _stale = stale; }
    inline HapticDeviceInputData getDeviceInputData(HHD handle);
    inline void setDeviceOutputData(HHD handle, const HDboolean& b1_state, const HDboolean& b2_state, const hduVector3Dd& position, const HDdouble* transform);

    void _initDeviceWithName(std::optional<std::string> device_name);
    void _startUpdateCallback();

    std::vector<HHD> _device_handles;
    std::map<HHD, HapticDeviceOutputData> _device_data;
    std::map<HHD, CopiedHapticDeviceOutputData> _copied_device_data;

    bool _stale;

    /** Tracks the forces applied on the haptic device. */
    std::map<HHD, Vec3r> _device_forces;

    /** Internally scales applied haptic forces before sending them to the device.
     */
    Real _force_scaling_factor = 1.0;

    std::mutex _input_mtx;
    std::mutex _state_mtx;
};

#endif // __HAPTIC_DEVICE_MANAGER_HPP