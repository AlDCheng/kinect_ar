#include "imageStream.h"

using namespace std;
using namespace cv;

// Constructor: Read from JSON to get image stream method
ImageStream::ImageStream(json j)
{
    // Get image stream method
    type = j["type"];

    // If we want to read from dataset
    if (type == "dataset") {
        dataset = j["dataset"];

        // TUM RGB-D Dataset
        if (dataset == "TUM") {
            path = j["filepath"];
            assoc_list.open(path + "/assoc.txt");
            if (!assoc_list.is_open()) {
                throw std::invalid_argument("invalid folder path");
            }
        }

    // If we want live stream data from Kinect V2
    } else if (type == "Kinect2") {
        InitKinect2();

    // Invalid arg passed; throw error
    } else {
        throw std::invalid_argument("invalid input type");
    }
}

// Deconstructor: Close files or close Kinect
ImageStream::~ImageStream()
{
    delete listener;
    if (type == "dataset") {
        assoc_list.close();
    } else if (type == "Kinect2") {
        dev->stop();
        dev->close();
        delete registration;
    }
}

// Initialization of Kinect V2
void ImageStream::InitKinect2()
{
    // Device discovery
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "SERIAL: " << serial << std::endl;

    // Enable OpenCL for faster capture if possible
    if(!pipeline) {
        pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
    }

    // Open Kinect to comms
    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }

    // Return if failed to opent Kinect
    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return;
    }

    //! [listeners]
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                 libfreenect2::Frame::Depth |
                                                 libfreenect2::Frame::Ir);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    // Start Kinect Device
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    // Registration using camera intrinsics
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

bool ImageStream::GetNext()
{
    if (type == "dataset") {
        if (dataset == "TUM") {
            return GetNextTUM();
        }
    } else if (type == "Kinect2") {
        return GetNextKinect2();
    }

    return false;   
}

bool ImageStream::GetNextTUM()
{
    string line;

    while (getline(assoc_list, line)) {
        stringstream ss(line);
        string s;

        if (line.empty()) return false;

        else {
            ss >> timestamp;
            ss >> s;
            rgb = imread(path + "/" + s, CV_LOAD_IMAGE_UNCHANGED);
            ss >> s;
            ss >> s;
            depth = imread(path + "/" + s, CV_LOAD_IMAGE_UNCHANGED);
            return true;
        }
    }

    return false;
}

bool ImageStream::GetNextKinect2()
{
    listener->waitForNewFrame(frames);
    libfreenect2::Frame *rgbFr = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *irFr = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depthFr = frames[libfreenect2::Frame::Depth];

    timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    cv::Mat(irFr->height, irFr->width, CV_32FC1, irFr->data).copyTo(ir);
    cv::Mat(depthFr->height, depthFr->width, CV_32FC1, depthFr->data).copyTo(depth);

    registration->apply(rgbFr, depthFr, &undistorted, &registered, true, &depth2rgb);
    cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgb);

    cv::cvtColor(rgb, rgb, CV_BGRA2BGR);
    depth.convertTo(depth, CV_16UC1);
    ir.convertTo(ir, CV_8UC1, 255 / (pow(2,16)-1));

    listener->release(frames);
    return true;
}