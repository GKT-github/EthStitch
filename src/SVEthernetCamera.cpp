/**
 * SVEthernetCamera.cpp
 * Implementation of Ethernet camera source for SVS project
 */

#include "SVEthernetCamera.hpp"
#include <opencv2/cudawarping.hpp>  // For cv::cuda::remap
#include <opencv2/cudaimgproc.hpp>  // ADD THIS LINE for cv::cuda::cvtColor
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream>

// Logging macros (matching original SVCamera.cpp)
#define LOG_DEBUG(msg, ...)   printf("DEBUG:   " msg "\n", ##__VA_ARGS__)
#define LOG_WARNING(msg, ...) printf("WARNING: " msg "\n", ##__VA_ARGS__)
#define LOG_ERROR(msg, ...)   printf("ERROR:   " msg "\n", ##__VA_ARGS__)

using namespace std::chrono_literals;

// CUDA kernel declaration (from original project)
extern void gpuConvertUYVY2RGB_async(const uchar* src, uchar* d_src, uchar* dst,
                                     int width, int height, cudaStream_t stream);

// ============================================================================
// InternalCameraParams Implementation (unchanged from original)
// ============================================================================

bool InternalCameraParams::read(const std::string& filepath, const int num,
                                const cv::Size& resol, const cv::Size& camResol)
{
    std::ifstream ifstrK{filepath + std::to_string(num) + ".K"};
    std::ifstream ifstrDist{filepath + std::to_string(num) + ".dist"};
    
    if (!ifstrK.is_open() || !ifstrDist.is_open()){
        LOG_ERROR("Can't open file with internal camera params");
        return false;
    }
    
    for(size_t i = 0; i < 9; i++)
        ifstrK >> K[i];
    for(size_t j = 0; j < 14; ++j)
        ifstrDist >> distortion[j];
    
    captureResolution = camResol;
    resolution = resol;
    
    ifstrK.close();
    ifstrDist.close();
    
    return true;
}

// ============================================================================
// EthernetCameraSource Implementation
// ============================================================================

EthernetCameraSource::EthernetCameraSource(const std::string& sourceIP, int sourcePort,
                                           const std::string& destIP, const std::string& name)
    : sourceIP(sourceIP)
    , sourcePort(sourcePort)
    , destIP(destIP)
    , cameraName(name)
    , pipeline(nullptr)
    , appsink(nullptr)
    , bus(nullptr)
    , cuda_out_buffer(nullptr)
    , isInit(false)
    , isStreaming(false)
{
}

EthernetCameraSource::~EthernetCameraSource() {
    deinit();
}

std::string EthernetCameraSource::createPipelineString() const {
    std::ostringstream pipeline;
    
    pipeline << "udpsrc address=" << destIP 
             << " port=" << sourcePort
             << " ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 "
             << " ! rtpjitterbuffer drop-on-latency=true latency=200 "
             << " ! rtph264depay "
             << " ! h264parse "
             << " ! nvv4l2decoder enable-max-performance=1 "
             << " ! nvvidconv "
             << " ! video/x-raw(memory:NVMM),format=RGBA,width=" << frameSize.width 
             << ",height=" << frameSize.height
             << " ! nvvidconv "
             << " ! video/x-raw,format=BGRx "  // Use BGRx instead of BGR
             << " ! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";
    
    return pipeline.str();
}

bool EthernetCameraSource::init(const cv::Size& frameSize_) {
    if (isInit) {
        LOG_WARNING("Camera %s already initialized", cameraName.c_str());
        return true;
    }
    
    frameSize = frameSize_;
    
    // Initialize GStreamer (only once globally)
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
    }
    
    LOG_DEBUG("Initializing Ethernet camera %s (%s:%d)...", 
              cameraName.c_str(), sourceIP.c_str(), sourcePort);
    
    // Create pipeline
    std::string pipelineStr = createPipelineString();
    GError* error = nullptr;
    pipeline = gst_parse_launch(pipelineStr.c_str(), &error);
    
    if (!pipeline || error) {
        LOG_ERROR("Failed to create pipeline for camera %s: %s", 
                  cameraName.c_str(), error ? error->message : "unknown");
        if (error) g_error_free(error);
        return false;
    }
    
    // Get appsink element
    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    if (!appsink) {
        LOG_ERROR("Failed to get appsink for camera %s", cameraName.c_str());
        gst_object_unref(pipeline);
        pipeline = nullptr;
        return false;
    }
    
    // Get bus for error monitoring
    bus = gst_element_get_bus(pipeline);
    
    // Allocate CUDA output buffer
    size_t size = frameSize.width * frameSize.height * 4;
    if (cudaMalloc(&cuda_out_buffer, size) != cudaSuccess) {
        LOG_ERROR("Failed to allocate CUDA memory for camera %s", cameraName.c_str());
        gst_object_unref(appsink);
        gst_object_unref(pipeline);
        return false;
    }
    
    isInit = true;
    LOG_DEBUG("Camera %s initialized successfully", cameraName.c_str());
    
    return true;
}

bool EthernetCameraSource::deinit() {
    if (!isInit) return true;
    
    stopStream();
    
    if (cuda_out_buffer) {
        cudaFree(cuda_out_buffer);
        cuda_out_buffer = nullptr;
    }
    
    if (bus) {
        gst_object_unref(bus);
        bus = nullptr;
    }
    
    if (appsink) {
        gst_object_unref(appsink);
        appsink = nullptr;
    }
    
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
    
    isInit = false;
    return true;
}

bool EthernetCameraSource::startStream() {
    if (!isInit) {
        LOG_ERROR("Camera %s not initialized", cameraName.c_str());
        return false;
    }
    
    if (isStreaming) return true;
    
    LOG_DEBUG("Starting stream for camera %s...", cameraName.c_str());
    
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    if (ret == GST_STATE_CHANGE_FAILURE) {
        LOG_ERROR("Failed to start stream for camera %s", cameraName.c_str());
        return false;
    }
    
    isStreaming = true;
    LOG_DEBUG("Camera %s stream started", cameraName.c_str());
    
    return true;
}

bool EthernetCameraSource::stopStream() {
    if (!isStreaming) return true;
    
    LOG_DEBUG("Stopping stream for camera %s...", cameraName.c_str());
    
    gst_element_set_state(pipeline, GST_STATE_NULL);
    isStreaming = false;
    
    return true;
}

bool EthernetCameraSource::capture(cv::cuda::GpuMat& frame, size_t timeout) {
    if (!isStreaming) {
        LOG_WARNING("Camera %s: capture called while not streaming", cameraName.c_str());
        return false;
    }
    
    // Pull sample from appsink
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 
                                                       timeout * 1000000); // ns
    
    if (!sample) {
        // Check for errors on bus
        GstMessage* msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
        if (msg) {
            GError* err;
            gchar* debug;
            gst_message_parse_error(msg, &err, &debug);
            LOG_ERROR("Camera %s error: %s", cameraName.c_str(), err->message);
            g_error_free(err);
            g_free(debug);
            gst_message_unref(msg);
        }
        return false;
    }
    
    // Get buffer from sample
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return false;
    }
    
    // Map buffer for reading
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }
    
    // Copy data to CUDA buffer
    cudaMemcpy(cuda_out_buffer, map.data, map.size, cudaMemcpyHostToDevice);
    
    // ✅ ADD THIS LINE: Create GpuMat wrapper around CUDA buffer (BGRx = 4 channels)
    cv::cuda::GpuMat temp(frameSize, CV_8UC4, cuda_out_buffer);
    
    // old Create GpuMat wrapper around CUDA buffer
    //frame = cv::cuda::GpuMat(frameSize, CV_8UC4, cuda_out_buffer);
    
    cv::cuda::cvtColor(temp, frame, cv::COLOR_BGRA2BGR);  // Convert 4-channel to 3-channel

    
    // Cleanup
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return true;
}

// ============================================================================
// MultiCameraSource Implementation
// ============================================================================

MultiCameraSource::MultiCameraSource()
    : _cams({
          EthernetCameraSource("192.168.45.10", 5020, "192.168.45.3", "Front"),
          EthernetCameraSource("192.168.45.11", 5021, "192.168.45.3", "Left"),
          EthernetCameraSource("192.168.45.12", 5022, "192.168.45.3", "Rear"),
          EthernetCameraSource("192.168.45.13", 5023, "192.168.45.3", "Right")
          
        //   EthernetCameraSource("192.168.45.11", 5021, "192.168.45.3", "Right"),   // Index 0 ✅
        //   EthernetCameraSource("192.168.45.12", 5022, "192.168.45.3", "Front"),  // Index 1 ✅
        //   EthernetCameraSource("192.168.45.13", 5023, "192.168.45.3", "Left"),   // Index 2 ✅ 
        //   EthernetCameraSource("192.168.45.10", 5020, "192.168.45.3", "Rear")    // Index 3 ✅ MUST BE LAST!


          
      })
    , destIP("192.168.45.3")
    , cudaStreamObj(cv::cuda::Stream::Null())
{
    // Initialize CUDA streams
    for (int i = 0; i < CAM_NUMS; ++i) {
        if (cudaStreamCreate(&_cudaStream[i]) != cudaSuccess) {
            _cudaStream[i] = nullptr;
            LOG_ERROR("Failed to create CUDA stream %d", i);
        }
    }
}

MultiCameraSource::~MultiCameraSource() {
    close();
}

int MultiCameraSource::init(const std::string& param_filepath, const cv::Size& calibSize,
                            const cv::Size& undistSize, const bool useUndist)
{
    LOG_DEBUG("Initializing multi-camera Ethernet source...");
    
    frameSize = undistSize;
    _undistort = useUndist;
    
    // Initialize all cameras
    bool allCamsOk = true;
    for (size_t i = 0; i < CAM_NUMS; ++i) {
        LOG_DEBUG("Initializing camera %zu: %s...", i, _cams[i].getCameraName().c_str());
        bool res = _cams[i].init(frameSize);
        LOG_DEBUG("Camera %zu init %s", i, res ? "OK" : "FAILED");
        allCamsOk &= res;
    }
    
    if (!allCamsOk) {
        LOG_ERROR("One or more cameras failed to initialize");
        return -1;
    }
    
    // ✅ ONLY load calibration if undistortion is enabled AND path is provided
    if (_undistort && !param_filepath.empty()) {
        LOG_DEBUG("Loading calibration files from: %s", param_filepath.c_str());
        
        for (size_t i = 0; i < CAM_NUMS; ++i) {
            if (!camIparams[i].read(param_filepath, i, calibSize, frameSize)) {
                LOG_ERROR("Failed to read calibration for camera %zu", i);
                LOG_WARNING("Disabling undistortion due to missing calibration files");
                _undistort = false;  // ✅ Disable undistortion if calibration fails
                break;
            }
            
            // Create camera matrix
            cv::Mat K(3, 3, CV_64FC1);
            for (size_t k = 0; k < camIparams[i].K.size(); ++k)
                K.at<double>(k) = camIparams[i].K[k];
            
            cv::Mat D(camIparams[i].distortion);
            
            // Compute undistortion maps
            cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, undistSize, 1, undistSize,
                                                         &undistFrames[i].roiFrame);
            Ks[i] = newK;
            
            cv::Mat mapX, mapY;
            cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, undistSize,
                                       CV_32FC1, mapX, mapY);
            
            undistFrames[i].remapX.upload(mapX);
            undistFrames[i].remapY.upload(mapY);
            
            LOG_DEBUG("Generated undistortion maps for camera %zu", i);
        }
    } else {
        LOG_DEBUG("Undistortion disabled - using raw camera frames");
    }
    
    LOG_DEBUG("Multi-camera source initialized successfully");
    return 0;
}

bool MultiCameraSource::startStream() {
    LOG_DEBUG("Starting all camera streams...");
    
    bool allStarted = true;
    for (auto& cam : _cams) {
        allStarted &= cam.startStream();
    }
    
    return allStarted;
}

bool MultiCameraSource::stopStream() {
    LOG_DEBUG("Stopping all camera streams...");
    
    bool allStopped = true;
    for (auto& cam : _cams) {
        allStopped &= cam.stopStream();
    }
    
    return allStopped;
}

bool MultiCameraSource::capture(std::array<Frame, CAM_NUMS>& frames) {
    bool allCaptured = true;
    
    // Capture from all cameras in parallel
    #pragma omp parallel for
    for (size_t i = 0; i < CAM_NUMS; ++i) {
        cv::cuda::GpuMat rawFrame;
        
        if (!_cams[i].capture(rawFrame, 5000)) {
            LOG_WARNING("Failed to capture from camera %zu", i);
            frames[i].gpuFrame = cv::cuda::GpuMat();  // ✅ ADD: Set to empty GpuMat
            allCaptured = false;
            continue;
        }
        
        // Check if frame is valid before processing
        if (rawFrame.empty()) {
            LOG_WARNING("Camera %zu returned empty frame", i);
            frames[i].gpuFrame = cv::cuda::GpuMat();  // ✅ ADD: Set to empty GpuMat
            allCaptured = false;
            continue;
        }
        
        // Apply undistortion if enabled
        if (_undistort && !undistFrames[i].remapX.empty()) {
            cv::cuda::remap(rawFrame, undistFrames[i].undistFrame,
                           undistFrames[i].remapX, undistFrames[i].remapY,
                           cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
            
            // Validate ROI before cropping
            if (undistFrames[i].roiFrame.x >= 0 && 
                undistFrames[i].roiFrame.y >= 0 &&
                undistFrames[i].roiFrame.x + undistFrames[i].roiFrame.width <= undistFrames[i].undistFrame.cols &&
                undistFrames[i].roiFrame.y + undistFrames[i].roiFrame.height <= undistFrames[i].undistFrame.rows) {
                
                frames[i].gpuFrame = undistFrames[i].undistFrame(undistFrames[i].roiFrame);
            } else {
                LOG_WARNING("Invalid ROI for camera %zu, using full undistorted frame", i);
                frames[i].gpuFrame = undistFrames[i].undistFrame;
            }
        } else {
            frames[i].gpuFrame = rawFrame;
        }
    }
    
    return allCaptured;
}

bool MultiCameraSource::setFrameSize(const cv::Size& size) {
    frameSize = size;
    
    // Reinitialize cameras with new size
    for (auto& cam : _cams) {
        cam.stopStream();
        cam.deinit();
        cam.init(size);
    }
    
    return true;
}

void MultiCameraSource::close() {
    stopStream();
    
    for (auto& cam : _cams) {
        cam.deinit();
    }
    
    for (size_t i = 0; i < CAM_NUMS; ++i) {
        if (_cudaStream[i]) {
            cudaStreamDestroy(_cudaStream[i]);
            _cudaStream[i] = nullptr;
        }
    }
}
