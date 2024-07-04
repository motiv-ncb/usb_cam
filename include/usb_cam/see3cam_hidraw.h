extern "C" {
    #include <libudev.h>
} 
#include <iostream>

namespace usb_cam {

class See3CamHidraw {
public:
    See3CamHidraw(
        int vendor_id, int product_id) : 
            vendor_id_(vendor_id), product_id_(product_id), subsystem_("hidraw"), dev_path_(NULL)  {
        dev_path_ = getDevicePath();
        if (dev_path_) {
            std::cout << "Found device: " << dev_path_ << std::endl;
        }
    }

    ~See3CamHidraw() = default; 
    const char* getDevicePath();
    bool setExposureCompensation(uint32_t value);
    bool sendHidCommand(int fd, unsigned char* inBuf, unsigned char* outBuf, size_t len);
protected:
    int product_id_;
    int vendor_id_;
    std::string subsystem_;
    const char* dev_path_;
};

}