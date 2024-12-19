extern "C" {
    #include <libudev.h>
} 
#include <iostream>
#include <cstdint>

namespace usb_cam {
class See3Cam {
public:
    See3Cam(int vendor_id, int product_id) : product_id_(product_id), vendor_id_(vendor_id), subsystem_("hidraw"), dev_path_(NULL) {

    }
    const char* getDevicePath();

protected:
    int product_id_;
    int vendor_id_;
    std::string subsystem_;
    const char* dev_path_;

};

class See3CamHidraw : public See3Cam {
public:
    See3CamHidraw(
        int vendor_id, int product_id) : See3Cam(vendor_id, product_id)
             {
        dev_path_ = getDevicePath();
        if (dev_path_) {
            std::cout << "Found device: " << dev_path_ << std::endl;
        }
    }

    ~See3CamHidraw() = default; 
    
    bool setExposureCompensation(std::uint32_t value);
    bool getFrameRate(std::uint8_t& value);
    bool setQFactor(std::uint8_t value);
    bool sendHidCommand(int fd, unsigned char* inBuf, unsigned char* outBuf, size_t len); 
};

}