#include "haptics/HapticDeviceManager.hpp"

HapticDeviceManager::HapticDeviceManager()
{
    /* Initialize the device, must be done before attempting to call any hd 
    functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
    initialized. */
    _initDeviceWithName(std::nullopt);

    _startUpdateCallback();
}

HapticDeviceManager::HapticDeviceManager(const std::string& device_name)
{
    _initDeviceWithName(device_name);

    _startUpdateCallback();
}

HapticDeviceManager::HapticDeviceManager(const std::string& device_name1, const std::string& device_name2)
{
    _initDeviceWithName(device_name1);
    _initDeviceWithName(device_name2);

    _startUpdateCallback();
}

void HapticDeviceManager::_startUpdateCallback()
{
    HDSchedulerHandle update_callback_handle = hdScheduleAsynchronous(_updateCallback, this, HD_MAX_SCHEDULER_PRIORITY);

    hdStartScheduler();

    /* Check for errors and abort if so. */
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
    }
}

void HapticDeviceManager::_initDeviceWithName(std::optional<std::string> device_name)
{
    
    HDErrorInfo error;

    HHD hHD;
    // if device name specified, use that
    // otherwise use HD_DEFAULT_DEVICE
    if (device_name.has_value())
    {
        std::cout << BOLD << " === Initializing haptic device with name " << device_name.value() << " ===" << RST << std::endl;
        hHD = hdInitDevice(device_name.value().c_str());
    }
    else
    {
        std::cout << BOLD << " === Initializing default device ===" << RST << std::endl;
        hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    }

    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        std::cerr << KRED;
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        std::cerr << RST;
        assert(0);
    }

    std::cout << "Found device model: " << BOLD << hdGetString(HD_DEVICE_MODEL_TYPE) << RST << std::endl;

    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);

    std::cout << "New handle: " << hHD << std::endl;

    _device_handles.push_back(hHD);
    _device_data[hHD] = HapticDeviceOutputData{};
    _copied_device_data[hHD] = CopiedHapticDeviceOutputData{};
    _device_forces[hHD] = Vec3r::Zero();

}

HapticDeviceManager::~HapticDeviceManager()
{
    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    // hdUnschedule(position_callback_handle);

    /* Disable the device. */
    for (const auto& handle : _device_handles)
        hdDisableDevice(handle);
}

HDCallbackCode HDCALLBACK HapticDeviceManager::_updateCallback(void *data)
{
    HapticDeviceManager* device_manager = static_cast<HapticDeviceManager*>(data);
    if (!device_manager)
    {
        std::cerr << "Update callback failed!" << std::endl;
        assert(0);
    }

    for (const auto& handle : device_manager->deviceHandles())
    {
        hduVector3Dd position_hd;
        HDdouble transform_hd[16];
        int nButtons;
        HDboolean button1_pressed_hd;
        HDboolean button2_pressed_hd;

        HapticDeviceInputData input_data = device_manager->getDeviceInputData(handle);

        /* Begin haptics frame.  ( In general, all state-related haptics calls
        should be made within a frame. ) */
        hdBeginFrame(handle);

        /* Get the current position of the device. */
        hdGetDoublev(HD_CURRENT_POSITION, position_hd);

        /* Retrieve the current button(s). */
        hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);

        hdGetDoublev(HD_CURRENT_TRANSFORM, transform_hd);
        
        /* In order to get the specific button 1 state, we use a bitmask to
        test for the HD_DEVICE_BUTTON_1 bit. */
        button1_pressed_hd = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
        button2_pressed_hd = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

        /** Update the force */
        hduVector3Dd force_hd;
        const Vec3r& force = input_data.device_force;
        force_hd[0] = device_manager->_force_scaling_factor*force[0];
        force_hd[1] = device_manager->_force_scaling_factor*force[1];
        force_hd[2] = device_manager->_force_scaling_factor*force[2];

        hdSetDoublev(HD_CURRENT_FORCE, force_hd);

        /* End haptics frame. */
        hdEndFrame(handle);

        HDErrorInfo error;
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "_updateCallback");
        }

        device_manager->setDeviceOutputData(handle, button1_pressed_hd, button2_pressed_hd, position_hd, transform_hd);
    }
    

    return HD_CALLBACK_CONTINUE;
}

void HapticDeviceManager::copyOutputState(HHD handle)
{
    std::lock_guard<std::mutex> guard(_state_mtx);

    CopiedHapticDeviceOutputData& copied_data = _copied_device_data.at(handle);
    const HapticDeviceOutputData& device_data = _device_data.at(handle);
    copied_data.last_position = copied_data.position;
    copied_data.position[0] = device_data.device_position[0];
    copied_data.position[1] = device_data.device_position[1];
    copied_data.position[2] = device_data.device_position[2];

    copied_data.orientation(0,0) = device_data.device_transform[0];
    copied_data.orientation(0,1) = device_data.device_transform[1];
    copied_data.orientation(0,2) = device_data.device_transform[2];
    copied_data.orientation(1,0) = device_data.device_transform[4];
    copied_data.orientation(1,1) = device_data.device_transform[5];
    copied_data.orientation(1,2) = device_data.device_transform[6];
    copied_data.orientation(2,0) = device_data.device_transform[8];
    copied_data.orientation(2,1) = device_data.device_transform[9];
    copied_data.orientation(2,2) = device_data.device_transform[10];

    copied_data.button1_pressed = device_data.button1_state;
    copied_data.button2_pressed = device_data.button2_state;

    copied_data.stale = false; 
}

void HapticDeviceManager::setForce(HHD handle, const Vec3r& force)
{
    std::lock_guard<std::mutex> guard(_input_mtx);
    _device_forces.at(handle) = force;
}

HapticDeviceInputData HapticDeviceManager::getDeviceInputData(HHD handle)
{
    std::lock_guard<std::mutex> guard(_input_mtx);
    HapticDeviceInputData input_data;
    input_data.device_force = _device_forces.at(handle);
    return input_data;
}

void HapticDeviceManager::setDeviceOutputData(HHD handle, const HDboolean& b1_state, const HDboolean& b2_state, const hduVector3Dd& position, const HDdouble* transform)
{
    std::lock_guard<std::mutex> guard(_state_mtx);
    HapticDeviceOutputData& device_data = _device_data.at(handle);
    device_data.button1_state = b1_state;
    device_data.button2_state = b2_state;
    device_data.device_position = position;
    std::memcpy(device_data.device_transform, transform, sizeof(HDdouble)*16);
    
    _copied_device_data.at(handle).stale = true;
}

Vec3r HapticDeviceManager::lastPosition(HHD handle)
{
    if (_copied_device_data.at(handle).stale)
    {
        copyOutputState(handle);
    }

    return _copied_device_data.at(handle).last_position;
}

Vec3r HapticDeviceManager::position(HHD handle)
{
    if (_copied_device_data.at(handle).stale)
    {
        // hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
        copyOutputState(handle);
    }

    return _copied_device_data.at(handle).position;
}

Mat3r HapticDeviceManager::orientation(HHD handle)
{
    if (_copied_device_data.at(handle).stale)
    {
        // hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
        copyOutputState(handle);
    }

    return _copied_device_data.at(handle).orientation;
}

bool HapticDeviceManager::button1Pressed(HHD handle)
{
    if (_copied_device_data.at(handle).stale)
    {
        // hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
        copyOutputState(handle);
    }

    return _copied_device_data.at(handle).button1_pressed;
}

bool HapticDeviceManager::button2Pressed(HHD handle)
{
    if (_copied_device_data.at(handle).stale)
    {
        // hdScheduleSynchronous(_copyCallback, this, HD_MIN_SCHEDULER_PRIORITY);
        copyOutputState(handle);
    }

    return _copied_device_data.at(handle).button2_pressed;
}