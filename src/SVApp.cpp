#include <SVApp.hpp>

#include <csignal>

#include <omp.h>
using namespace std::chrono_literals;  // ADD THIS LINE

#define GL_USE

#ifndef GL_USE
#include <opencv2/highgui.hpp>
#endif


#define LOG_USE

bool finish = false;

static void addCar(std::shared_ptr<SVRender>& view_, const SVAppConfig& svcfg)
{
    glm::mat4 transform_car(1.f);
#ifdef HEMISPHERE
     transform_car = glm::translate(transform_car, glm::vec3(0.f, 0.09f, 0.f));
#else
     transform_car = glm::translate(transform_car, glm::vec3(0.f, 1.01f, 0.f));
#endif
	// ADD THIS LINE: Rotate -90 degrees around X-axis (Pitch)
    transform_car = glm::rotate(transform_car, glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));
    
    // ADD THIS LINE: Rotate 0 degrees around Y-axis (Roll)
    transform_car = glm::rotate(transform_car, glm::radians(0.f), glm::vec3(0.f, 1.f, 0.f));
    
    // ADD THIS LINE: Rotate 180 degrees around Z-axis (Yaw)
    transform_car = glm::rotate(transform_car, glm::radians(180.f), glm::vec3(0.f, 0.f, 1.f));
    
    transform_car = glm::scale(transform_car, glm::vec3(0.002f));

    bool is_Add = view_->addModel(svcfg.car_model, svcfg.car_vert_shader,
                    svcfg.car_frag_shader, transform_car);
    if (!is_Add)
      std::cerr << "Error can't add model\n";
}

static void addBowlConfig(ConfigBowl& cbowl)
{
    /* Bowl parameter */
    glm::mat4 transform_bowl(1.f);
    cbowl.transformation = transform_bowl;
    cbowl.disk_radius = 0.4f;
    cbowl.parab_radius = 0.55f;
    cbowl.hole_radius = 0.08f;
    cbowl.a = 0.4f; cbowl.b = 0.4f; cbowl.c = 0.2f;
    cbowl.vertices_num  = 750.f;
    cbowl.y_start = 1.0f;
}


SVApp::SVApp(const SVAppConfig& svcfg) :
    cameraSize(svcfg.cam_width, svcfg.cam_height), undistSize(svcfg.cam_width, svcfg.cam_height),
    calibSize(svcfg.calib_width, svcfg.calib_height), limit_iteration_show(-1), threadpool(svcfg.num_pool_threads)
{
    svappcfg = svcfg;
    cameradata = std::move(std::vector<cv::cuda::GpuMat>(CAM_NUMS + 1));
    pedestrian_rect = std::move(std::vector<std::vector<cv::Rect>>(CAM_NUMS));
}

SVApp::~SVApp()
{
    release();
}


void SVApp::release()
{
    source->stopStream();
}


bool SVApp::init(const int limit_iteration_init_)
{

#ifndef NO_OMP
        omp_set_num_threads(svappcfg.num_pool_threads);
#endif
        limit_iteration_init = limit_iteration_init_;

        source = std::make_shared<MultiCameraSource>();

        source->setFrameSize(cameraSize);

        int code = source->init(svappcfg.undist_folder, calibSize, undistSize, false);
        if (code < 0){
                std::cerr << "source init failed " << code << "\n";
                return false;
        }

        usePedDetect = svappcfg.usePedestrianDetection;

#ifndef GL_USE
        cv::namedWindow(svappcfg.win1, cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
#endif
        source->startStream();

        view_scene = std::make_shared<SVRender>(cameraSize.width, cameraSize.height);
        dp = std::make_shared<SVDisplayView>();
        svtitch = std::make_shared<SVStitcher>(svappcfg.numbands, svappcfg.scale_factor);
        if (usePedDetect)
            sv_ped_det = std::make_shared<SVPedDetect>(CAM_NUMS, svappcfg.scale_factor);

        auto init = false;
        while (!svtitch->getInit() && limit_iteration_init != 0 && !finish){

                if (!source->capture(frames)){
                        std::cerr << "capture failed\n";
                        std::this_thread::sleep_for(1ms);
                        continue;
                }

                // std::vector<cv::cuda::GpuMat> datas {cv::cuda::GpuMat(), frames[0].gpuFrame, frames[1].gpuFrame, frames[2].gpuFrame, frames[3].gpuFrame};

                std::vector<cv::cuda::GpuMat> datas { frames[0].gpuFrame, frames[1].gpuFrame, frames[2].gpuFrame, frames[3].gpuFrame};
                //init = svtitch->init(datas); // this part include autocalibration with features detection

		//Debug After capturing frames, before calling svtitch->initFromFile
		// for (size_t i = 0; i < frames.size(); ++i) {
		//     cv::Mat cpu_frame;
		//     frames[i].gpuFrame.download(cpu_frame);
		//     cv::imwrite("debug_camera_" + std::to_string(i) + ".jpg", cpu_frame);
		//     std::cout << "Saved debug_camera_" << i << ".jpg" << std::endl;
            // }
               init = svtitch->initFromFile(svappcfg.calib_folder, datas, false);
#ifdef GL_USE
                if (init){
                    addBowlConfig(svappcfg.cbowl);
                    dp->init(cameraSize.width, cameraSize.height, view_scene);

                    view_scene->init(svappcfg.cbowl, svappcfg.surroundshadervert, svappcfg.surroundshaderfrag,
                                     svappcfg.screenshadervert, svappcfg.screenshaderfrag,
                                     svappcfg.blackrectshadervert, svappcfg.blackrectshaderfrag);

                    addCar(view_scene, svappcfg);
                }
#else
                if (cv::waitKey(1) > 0)
                    break;
#endif
                if (limit_iteration_init > 0)
                    limit_iteration_init -= 1;
        }

        return init;
}


void SVApp::run()
{
    auto lastTick = std::chrono::high_resolution_clock::now();
    time_recompute_gain = 0;
    int consecutiveFailures = 0;
    
    while (!finish){

            if (!source->capture(frames)){
                    std::cerr << "capture failed\n";
                    consecutiveFailures++;
                    
                    if (consecutiveFailures > 100) {
                        std::cerr << "Too many consecutive capture failures, exiting\n";
                        break;
                    }
                    
                    std::this_thread::sleep_for(1ms);
                    continue;
            }
            
            consecutiveFailures = 0;

            // ✅ ADD: Validate all frames before processing
            bool allFramesValid = true;
            for (auto i = 0; i < frames.size(); ++i) {
                if (frames[i].gpuFrame.empty()) {
                    std::cerr << "Frame " << i << " is empty, skipping this iteration\n";
                    allFramesValid = false;
                    break;
                }
            }
            
            // ✅ ADD: Skip processing if any frame is invalid
            if (!allFramesValid) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            for (auto i = 1; i <= frames.size(); ++i)
              cameradata[i] = frames[i - 1].gpuFrame;

            if (usePedDetect)
                sv_ped_det->detect(cameradata, pedestrian_rect);

            svtitch->stitch(cameradata, stitch_frame);

#ifdef GL_USE
            view_scene->setWhiteLuminance(svtitch->getWhiteLuminance());
            view_scene->setToneLuminance(svtitch->getLuminance());

            bool okRender = dp->render(stitch_frame);
            if (!okRender)
              break;
            std::this_thread::sleep_for(3ms);
#else
           cv::imshow(svappcfg.win1, stitch_frame);
           if (cv::waitKey(1) > 0)
               break;
#endif

            const auto now = std::chrono::high_resolution_clock::now();
            const auto dt = now - lastTick;
            lastTick = now;
            const int dtMs = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
            eventTask(dtMs, cameradata, stitch_frame);
#ifdef LOG_USE
            //std::cout << "dt = " << dtMs << " ms\n";
#endif
    }
}



void SVApp::eventTask(int dtms, const std::vector<cv::cuda::GpuMat>& datas, const cv::cuda::GpuMat& stitched_img)
{
    time_recompute_gain += dtms;
    if (std::chrono::milliseconds(time_recompute_gain) >= svappcfg.time_recompute_photometric_gain){
           time_recompute_gain = 0;
           threadpool.enqueue([=](){
                svtitch->recomputeGain(datas);
                std::this_thread::sleep_for(1ms);
           });
    }
    time_recompute_max_luminance += dtms;
    if (std::chrono::milliseconds(time_recompute_max_luminance) >= svappcfg.time_recompute_photometric_luminance){
           time_recompute_max_luminance = 0;
           threadpool.enqueue([=](){
                svtitch->recomputeToneLuminance(stitched_img);
                std::this_thread::sleep_for(1ms);
           });
    }

}

