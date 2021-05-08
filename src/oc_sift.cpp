/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_sift.h"
#include <random>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace opencorr
{

	SIFT2D::SIFT2D() {
		this->SIFT_config.n_features = 0;
		this->SIFT_config.n_octave_layers = 3;
		this->SIFT_config.contrast_threshold = 0.04f;
		this->SIFT_config.edge_threshold = 10.f;
		this->SIFT_config.sigma = 1.6f;
		this->match_ratio = 0.8f;
	}

	SIFT2D::~SIFT2D() {
	}

	void SIFT2D::prepare() {
		this->ref_mat = &this->ref_img->cv_mat;
		this->tar_mat = &this->tar_img->cv_mat;
	}

	void SIFT2D::compute() {
		//initialization, refer to opencv document for details
		std::vector<cv::KeyPoint> ref_kp;
		cv::Mat ref_descriptor;
		cv::Ptr<cv::Feature2D> ref_SIFT = cv::xfeatures2d::SIFT::create(
			this->SIFT_config.n_features,
			this->SIFT_config.n_octave_layers,
			this->SIFT_config.contrast_threshold,
			this->SIFT_config.edge_threshold,
			this->SIFT_config.sigma);

		std::vector<cv::KeyPoint> tar_kp;
		cv::Mat tar_descriptor;
		cv::Ptr<cv::Feature2D> tar_SIFT = cv::xfeatures2d::SIFT::create(
			this->SIFT_config.n_features,
			this->SIFT_config.n_octave_layers,
			this->SIFT_config.contrast_threshold,
			this->SIFT_config.edge_threshold,
			this->SIFT_config.sigma);

		//extract features and construct their descriptor
#pragma omp parallel sections
		{
#pragma omp section
			{
				ref_SIFT->detect(*this->ref_mat, ref_kp);
				ref_SIFT->compute(*this->ref_mat, ref_kp, ref_descriptor);
			}
#pragma omp section
			{
				tar_SIFT->detect(*this->tar_mat, tar_kp);
				tar_SIFT->compute(*this->tar_mat, tar_kp, tar_descriptor);
			}
		}
		//match the keypoints in the reference image and the target image
		//refer to Yang et al. Opt Laser Eng (2020), 127, 105964 for details
		cv::FlannBasedMatcher matcher;
		std::vector<std::vector<cv::DMatch>> matches;
		matcher.knnMatch(ref_descriptor, tar_descriptor, matches, 2);
		for (int i = 0; i < matches.size(); ++i) {
			if (matches[i][0].distance < this->match_ratio * matches[i][1].distance) {
				Point2D ref_candidate(ref_kp[i].pt.x, ref_kp[i].pt.y);
				this->ref_matched_kp.push_back(ref_candidate);
				Point2D tar_candidate(tar_kp[matches[i][0].trainIdx].pt.x, tar_kp[matches[i][0].trainIdx].pt.y);
				this->tar_matched_kp.push_back(tar_candidate);
			}
		}
	}

	SIFTconfig SIFT2D::getSIFTconfig() const {
		return this->SIFT_config;
	}

	float SIFT2D::getMatchRatio() const {
		return this->match_ratio;
	}

	void SIFT2D::setExtraction(SIFTconfig SIFT_config) {
		//refer to opencv document for detailed information
		this->SIFT_config = SIFT_config;
	}

	void SIFT2D::setMatch(float match_ratio) {
		//refer to Yang et al. Opt Laser Eng (2020), 127, 105964
		this->match_ratio = match_ratio;
	}

}//namespace opencorr
