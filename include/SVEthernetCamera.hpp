/**
 * SVEthernetCamera.hpp
 * Replacement for SVCamera.hpp - Uses Ethernet H.264 cameras instead of MIPI CSI
 * 
 * Drop-in replacement maintaining the same interface as the original SVCamera
 */

#ifndef SV_ETHERNET_CAMERA_HPP
#define SV_ETHERNET_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <array>
#include <vector>
#include <string>
#include <cuda_runtime.h>

// Configuration
#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 800


#define MMAP_BUFFERS_COUNT 4
constexpr int CAM_NUMS = 4;

/**
 * @brief Frame structure - matches original SVCamera interface
 */
struct Frame {
    cv::cuda::GpuMat gpuFrame;
    cv::Mat cpuFrame;  // Optional CPU copy
};

/**
 * @brief Camera calibration parameters - matches original
 */
struct InternalCameraParams {
    std::array<double, 9> K;           // Camera matrix
    std::array<double, 14> distortion; // Distortion coefficients
    cv::Size resolution;
    cv::Size captureResolution;
    
    bool read(const std::string& filepath, const int num, 
              const cv::Size& resol, const cv::Size& camResol);
};

/**
 * @brief Undistortion data for each camera - matches original
 */
struct CameraUndistortData {
    cv::cuda::GpuMat undistFrame;
    cv::cuda::GpuMat remapX;
    cv::cuda::GpuMat remapY;
    cv::Rect roiFrame;
};

/**
 * @brief Single Ethernet camera source using GStreamer
 */
class EthernetCameraSource {
public:
    EthernetCameraSource(const std::string& sourceIP, int sourcePort, 
                         const std::string& destIP, const std::string& name);
    ~EthernetCameraSource();
    
    bool init(const cv::Size& frameSize);
    bool deinit();
    bool startStream();
    bool stopStream();
    bool capture(cv::cuda::GpuMat& frame, size_t timeout = 1000);
    
    const std::string& getCameraName() const { return cameraName; }
    
private:
    // GStreamer elements
    GstElement* pipeline;
    GstElement* appsink;
    GstBus* bus;
    
    // Camera configuration
    std::string sourceIP;
    int sourcePort;
    std::string destIP;
    std::string cameraName;
    
    // Frame buffer
    cv::Size frameSize;
    uchar* cuda_out_buffer;
    bool isInit;
    bool isStreaming;
    
    // Helper methods
    std::string createPipelineString() const;
    static GstFlowReturn newSampleCallback(GstElement* sink, gpointer data);
};

/**
 * @brief Multi-camera synchronized source - matches original interface
 */
class MultiCameraSource {
public:
    MultiCameraSource();
    ~MultiCameraSource();
    
    // Interface matching original SVCamera
    int init(const std::string& param_filepath, const cv::Size& calibSize, 
             const cv::Size& undistSize, const bool useUndist = false);
    bool startStream();
    bool stopStream();
    bool capture(std::array<Frame, CAM_NUMS>& frames);
    bool setFrameSize(const cv::Size& size);
    
    void close();
    
    // Getters matching original interface
    const EthernetCameraSource& getCamera(int index) const { return _cams[index]; }
    size_t getCamerasCount() const { return CAM_NUMS; }
    cv::Size getFramesize() const { return frameSize; }
    const CameraUndistortData& getUndistortData(const size_t idx) const { 
        return undistFrames[idx]; 
    }
    
    bool _undistort = true;
    std::array<cv::Mat, CAM_NUMS> Ks;  // Camera matrices
    
private:
    // Camera sources - 4 Ethernet cameras
    std::array<EthernetCameraSource, CAM_NUMS> _cams;
    
    // Frame processing
    cv::Size frameSize;
    std::array<InternalCameraParams, CAM_NUMS> camIparams;
    std::array<CameraUndistortData, CAM_NUMS> undistFrames;
    
    // CUDA streams for parallel processing
    cudaStream_t _cudaStream[CAM_NUMS];
    cv::cuda::Stream cudaStreamObj;
    
    // Camera configuration
    struct CameraConfig {
        std::string sourceIP;
        int sourcePort;
        std::string name;
    };
    
    std::array<CameraConfig, CAM_NUMS> cameraConfigs;
    std::string destIP;
};

#endif // SV_ETHERNET_CAMERA_HPP
