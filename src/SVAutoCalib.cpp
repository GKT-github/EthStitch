#include <SVAutoCalib.hpp>

#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/imgproc.hpp>  // ✅ ADD THIS LINE for equalizeHist and CLAHE
#include <opencv2/imgcodecs.hpp>   // ✅ ADD THIS for imwrite


#include <iostream>


bool SVAutoCalib::calibrate(const std::vector<cv::Mat>& imgs, const bool savedata)
{
	if (isInit){
		std::cerr << "Autocalibrator already initialize...\n";
		return isInit;
	}

	if (imgs.size() > imgs_num)
		return false;


	std::vector<cv::detail::ImageFeatures> features(imgs_num);
	std::vector<cv::detail::MatchesInfo> pairwise_matches;

	if (!computeImageFeaturesAndMatches_(imgs, pairwise_matches, features)){
		std::cerr << "Error can't find pairwise features...\n";
		return false;
	}

	//std::vector<int> indxs = cv::detail::leaveBiggestComponent(features, pairwise_matches, conf_thresh);

	if (pairwise_matches.size() < (imgs_num*imgs_num)){
		std::cout << pairwise_matches.size() << "\n";
		std::cerr << "Error not enough calibrates images...\n";
		return false;
	}

	if (!computeCameraParameters(features, pairwise_matches))
		return false;

	if (savedata)
	    saveData();

	isInit = true;

	return isInit;
}

bool SVAutoCalib::computeImageFeaturesAndMatches_(
    const std::vector<cv::Mat>& imgs, 
    std::vector<cv::detail::MatchesInfo>& pairwise_matches, 
    std::vector<cv::detail::ImageFeatures>& features)
{
    // Use SIFT for robust feature detection
    cv::Ptr<cv::Feature2D> finder = cv::SIFT::create(
        5000,      // Increase max features
        3,         // Octave layers  
        0.03,      // Lower contrast threshold = more features
        10,        // Edge threshold
        1.6        // Sigma
    );
    
    // Matcher with relaxed threshold for SIFT
    cv::Ptr<cv::detail::FeaturesMatcher> matcher = 
        cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, 0.3f);
    
    // Gamma correction lookup table
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    constexpr auto gamma = 0.45;
    for (int i = 0; i < 256; ++i){
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    
    // Create CLAHE for contrast enhancement
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
    
    std::cout << "\n=== Feature Detection ===" << std::endl;
    for (int i = 0; i < imgs_num; ++i){
        cv::Mat processed = imgs[i].clone();
        
        // Convert to grayscale if needed
        if (processed.channels() == 3){
            cv::cvtColor(processed, processed, cv::COLOR_BGR2GRAY);
        }
        
        // Apply gamma correction
        cv::Mat corrected;
        cv::LUT(processed, lookUpTable, corrected);
        
        // Enhance contrast with CLAHE
        cv::Mat enhanced;
        clahe->apply(corrected, enhanced);
        
        // Detect features on enhanced image
        cv::detail::computeImageFeatures(finder, enhanced, features[i]);
        
        std::cout << "Camera " << i << ": " << features[i].keypoints.size() 
                  << " features detected" << std::endl;
        
        // ================================Save debug image (optional - comment out if not needed)
        // cv::Mat debug_img;
        // cv::drawKeypoints(imgs[i], features[i].keypoints, debug_img, 
        //                  cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::imwrite("debug_features_cam" + std::to_string(i) + ".jpg", debug_img);
    }
    
    // Match features
    (*matcher)(features, pairwise_matches);
    
    // Print detailed results
    std::cout << "\n=== Feature Matching Results ===" << std::endl;
    int total_good_matches = 0;
    for(const auto& m : pairwise_matches){
        if (m.matches.size() > 0){
            std::cout << "Camera " << m.src_img_idx << " <-> " << m.dst_img_idx 
                      << ": " << m.matches.size() << " matches, confidence: " 
                      << m.confidence << std::endl;
            total_good_matches++;
            
            // ==========================Save match visualization (optional)
            // cv::Mat match_img;
            // cv::drawMatches(imgs[m.src_img_idx], features[m.src_img_idx].keypoints,
            //                imgs[m.dst_img_idx], features[m.dst_img_idx].keypoints,
            //                m.matches, match_img);
            // cv::imwrite("debug_matches_" + std::to_string(m.src_img_idx) + "_" + 
            //             std::to_string(m.dst_img_idx) + ".jpg", match_img);
        }
    }
    
    std::cout << "Total good match pairs: " << total_good_matches << std::endl;
    
    if (total_good_matches < 4){
        std::cerr << "\n⚠️  WARNING: Only " << total_good_matches 
                  << " camera pairs have matches!" << std::endl;
        std::cerr << "Need at least 4 pairs for good calibration." << std::endl;
    }
    
    return true;
}

// bool SVAutoCalib::computeImageFeaturesAndMatches_(const std::vector<cv::Mat>& imgs, std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::ImageFeatures>& features)
// {
// 	cv::Ptr<cv::Feature2D> finder = cv::ORB::create(maxpoints, 1.2, pyr_levels, patch_size, 0, 3, cv::ORB::HARRIS_SCORE,
// 							patch_size, threshold_features);

// 	cv::Ptr<cv::detail::FeaturesMatcher> matcher = cv::makePtr<cv::detail::BestOf2NearestMatcher>(true, match_conf);

// #ifdef GAMMA_CORRECTION_CALIB
// 	cv::Mat lookUpTable(1, 256, CV_8U);
// 	uchar* p = lookUpTable.ptr();
// 	constexpr auto gamma = 0.45;
// 	for (int i = 0; i < 256; ++i){
// 	   p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
// 	}
// #endif
// 	for (int i = 0; i < imgs_num; ++i){
// #ifdef GAMMA_CORRECTION_CALIB
// 	      cv::Mat res;
// 	      res = imgs[i].clone();
// 	      cv::LUT(res, lookUpTable, res);
// #endif
// 	      cv::detail::computeImageFeatures(finder, imgs[i], features[i]);
// 	}


// 	(*matcher)(features, pairwise_matches);

// #ifdef DEBUG_P
// 	for(const auto& m : pairwise_matches){
// 	  std::cerr << m.confidence << "\n";
// 	}
// #endif


// 	return true;
// }

bool SVAutoCalib::computeCameraParameters(const std::vector<cv::detail::ImageFeatures>& features, const std::vector<cv::detail::MatchesInfo>& pairwise_matches)
{

	cv::detail::HomographyBasedEstimator est;
	std::vector<cv::detail::CameraParams> cameras;

	if (!est(features, pairwise_matches, cameras)){
		std::cerr << "Error refinement camera params...\n";
		return false;
	}

	for (size_t i = 0; i < cameras.size(); i++){
		cv::Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
	}

	cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterRay>();

	adjuster->setConfThresh(conf_thresh);

	if (!(*adjuster)(features, pairwise_matches, cameras)){
		std::cerr << "Error refinement camera params...\n";
		return false;
	}


	std::vector<cv::Mat> rmats;
	for(size_t i = 0; i < cameras.size(); ++i)
		rmats.emplace_back(cameras[i].R.clone());

	std::vector<float> focals;
	cv::detail::waveCorrect(rmats, cv::detail::WAVE_CORRECT_HORIZ);
	for(size_t i = 0; i < cameras.size(); ++i){
		cameras[i].R = rmats[i];
		focals.emplace_back(cameras[i].focal);
		cv::Mat_<float> K;
		cameras[i].K().convertTo(K, CV_32F);
		Ks_f.emplace_back(K);
		R.emplace_back(cameras[i].R);
		T.emplace_back(cameras[i].t);
	}

	std::sort(focals.begin(), focals.end());
	size_t focals_size = focals.size();
	if (focals_size % 2)
		warped_image_scale = static_cast<float>(focals[focals_size / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals_size / 2 - 1] + focals[focals_size / 2]) * 0.5f;

	return true;
}

void SVAutoCalib::saveData(const std::string& strpath) const
{

    for(auto i = 0; i < imgs_num; ++i){
           std::string KRpath{"Camparam" + std::to_string(i) + ".yaml"};
           cv::FileStorage KRfout(KRpath, cv::FileStorage::WRITE);
           KRfout << "FocalLength" << warped_image_scale;
           KRfout << "Intrisic" << Ks_f[i];
           KRfout << "Rotation" << R[i];
           KRfout << "Translation" << T[i];

		   // ADD THIS: Print calibration results
		    
		   std::cout << "\nCamera " << i << ":\n";
           std::cout << "K (Intrinsic):\n" << Ks_f[i] << "\n";
           std::cout << "R (Rotation):\n" << R[i] << "\n";
           std::cout << "T (Translation):\n" << T[i] << "\n";
    }

}



