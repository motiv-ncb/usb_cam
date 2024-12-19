#include <usb_cam/see3cam_hidraw.h>

#include <linux/hidraw.h>
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define CAMERA_CONTROL_CU135             0x81

#define SET_EXPOSURE_COMPENSATION_CU135  0x1A
#define GET_EXPOSURE_COMPENSATION_CU135  0x19

#define SET_FRAME_RATE_CU135             0x1C
#define GET_FRAME_RATE_CU135             0x1B

#define GET_Q_FACTOR_CU135               0x0D
#define SET_Q_FACTOR_CU135				 0x0E

#define SET_FAIL		0x00
#define SET_SUCCESS		0x01


namespace usb_cam {


const char* See3Cam::getDevicePath() {
    struct udev* udev = udev_new();  
	struct udev_enumerate* enumerate; 
    const char* node_path = NULL;
    bool found = false;

    if (!udev) {
        std::cerr << "udev_new() failed" << std::endl;
        return node_path;
    }


    enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        std::cerr << "udev_enumerate_new() failed" << std::endl;
        return node_path;
    }

    udev_enumerate_add_match_subsystem(enumerate, subsystem_.c_str());
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        const char* action = udev_device_get_action(dev); 

        if (action) {
            continue;
        }

        if (found) {
            continue;
        }

        node_path = udev_device_get_devnode(dev); 
        int fd = open(node_path, O_RDWR|O_NONBLOCK); 
        if(fd < 0) {
            std::cerr << "Error opening device: " << node_path << std::endl;
            break;
        }

	    struct hidraw_devinfo info; 
		memset(&info, 0x0, sizeof(info)); 
        if(ioctl(fd, HIDIOCGRAWINFO, &info) >= 0){ 
            if((info.vendor & 0xffff) == vendor_id_ && (info.product & 0xffff) == product_id_)
            {    
                found = true; 
            }
        }
        else {
            std::cerr << "HIDIOCGRAWINFO failed" << std::endl;
        }

        close(fd);

        if (found) {
            break;
        }
    }

    udev_enumerate_unref(enumerate);

    udev_unref(udev);

    if (!found) {
        node_path = NULL;
    }

    return node_path;
}

bool See3CamHidraw::setExposureCompensation(uint32_t value) {
    if (!dev_path_) {
        std::cerr << "HIDRAW Device not initialized." << std::endl;
        return false;
    }

    if (value < 8000 || value > 1000000) {
        std::cerr << "exposure compensation range error" << std::endl;
        return false;
    }


    bool ret = false;
 
    uint8_t buf[64],in_buf[64];
    memset(buf, 0x00, sizeof(buf));
    memset(in_buf, 0x00, sizeof(in_buf));
 
    int fd = open(dev_path_, O_RDWR|O_NONBLOCK);
    if(fd < 0) {
        std::cerr << "Error opening device: " << dev_path_ << std::endl;
        return false;
    }

    buf[0] = CAMERA_CONTROL_CU135;
    buf[1] = SET_EXPOSURE_COMPENSATION_CU135;
    buf[2] = ((value >> 24)&0xFF);
    buf[3] = ((value >> 16)&0xFF);
    buf[4] = ((value >> 8)&0xFF);
    buf[5] = value & 0xFF;
 
    if(sendHidCommand(fd, buf, in_buf, 64)){
        if (in_buf[6]==SET_FAIL) { 
            ret = false;
        } else if(in_buf[0] == CAMERA_CONTROL_CU135 &&
            in_buf[1]==SET_EXPOSURE_COMPENSATION_CU135 &&
            in_buf[6]==SET_SUCCESS) { 
            ret = true;
        }
    }
    close(fd);

    return ret;
}


bool See3CamHidraw::getFrameRate(uint8_t& framerate) {
    if (!dev_path_) {
        std::cerr << "HIDRAW Device not initialized." << std::endl;
        return false;
    }

    bool ret = false;
 
    uint8_t buf[64],in_buf[64];
    memset(buf, 0x00, sizeof(buf));
    memset(in_buf, 0x00, sizeof(in_buf));
 
    int fd = open(dev_path_, O_RDWR|O_NONBLOCK);
    if(fd < 0) {
        std::cerr << "Error opening device: " << dev_path_ << std::endl;
        return false;
    }
    
    buf[0] = CAMERA_CONTROL_CU135;
    buf[1] = GET_FRAME_RATE_CU135; 
 
    if(sendHidCommand(fd, buf, in_buf, 64))
    {
        if (in_buf[6]==SET_FAIL) 
        { 
            ret = false;
        } 
        else if(
            in_buf[0] == CAMERA_CONTROL_CU135 &&
            in_buf[1] == GET_FRAME_RATE_CU135 &&
            in_buf[6] == SET_SUCCESS) 
        { 
                
            framerate = static_cast<uint8_t>(in_buf[2]);
            ret = true;
        }
    }
    close(fd);

    return ret;
}

bool See3CamHidraw::setQFactor(uint8_t value) {
    if (!dev_path_) {
        std::cerr << "HIDRAW Device not initialized." << std::endl;
        return false;
    }

    if (value > 100) {
        std::cerr << "Q-factor range error" << std::endl;
        return false;
    }


    bool ret = false;
 
    uint8_t buf[64],in_buf[64];
    memset(buf, 0x00, sizeof(buf));
    memset(in_buf, 0x00, sizeof(in_buf));
 
    int fd = open(dev_path_, O_RDWR|O_NONBLOCK);
    if(fd < 0) {
        std::cerr << "Error opening device: " << dev_path_ << std::endl;
        return false;
    }

    buf[0] = CAMERA_CONTROL_CU135;
    buf[1] = SET_Q_FACTOR_CU135;
    buf[2] = value; 

    if(sendHidCommand(fd, buf, in_buf, 64)){
        if (in_buf[6]==SET_FAIL) { 
            ret = false;
        } else if(in_buf[0] == CAMERA_CONTROL_CU135 &&
            in_buf[1]==SET_Q_FACTOR_CU135 &&
            in_buf[6]==SET_SUCCESS) { 
            ret = true;
        }
    }
    close(fd);

    return ret;
}

bool See3CamHidraw::sendHidCommand(int fd, unsigned char* outBuf, unsigned char* inBuf, size_t len) {
    // Write data into camera
    int ret = write(fd, outBuf, len);
    if (ret < 0) {
        return false;
    }

    struct timeval tv;
    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    /* Wait up to 5 seconds. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    // Monitor read file descriptor for 5 secs
    if(0 > select(1, &rfds, NULL, NULL, &tv)){

      perror("select");
        return false;
    }

    // Read data from camera
    int retval = read(fd, inBuf, len);
    if (retval < 0) {
        return false;
    }
    else{
        return true;
    }
}

}