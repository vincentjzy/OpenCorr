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

#include <omp.h>
#include "oc_fftcc.h"

namespace opencorr {

	FFTW* FFTW::allocate(int subset_radius_x, int subset_radius_y) {
		int width = 2 * subset_radius_x;
		int height = 2 * subset_radius_y;
		int length = subset_radius_y + 1;

		FFTW* FFTW_instance = new FFTW;

		FFTW_instance->ref_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * width * length);
		FFTW_instance->tar_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * width * length);
		FFTW_instance->ZNCC_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * width * length);

		FFTW_instance->ref_subset = new float[width * height];
		FFTW_instance->tar_subset = new float[width * height];
		FFTW_instance->ZNCC = new float[width * height];

#pragma omp critical 
		{
			FFTW_instance->ref_plan = fftwf_plan_dft_r2c_2d(width, height, FFTW_instance->ref_subset, FFTW_instance->ref_freq, FFTW_ESTIMATE);
			FFTW_instance->tar_plan = fftwf_plan_dft_r2c_2d(width, height, FFTW_instance->tar_subset, FFTW_instance->tar_freq, FFTW_ESTIMATE);
			FFTW_instance->ZNCC_plan = fftwf_plan_dft_c2r_2d(width, height, FFTW_instance->ZNCC_freq, FFTW_instance->ZNCC, FFTW_ESTIMATE);
		}
		return FFTW_instance;
	}

	void FFTW::release(FFTW* instance) {
		delete[] instance->ref_subset;
		delete[] instance->tar_subset;
		delete[] instance->ZNCC;
		fftw_free(instance->ref_freq);
		fftw_free(instance->tar_freq);
		fftw_free(instance->ZNCC_freq);
		fftwf_destroy_plan(instance->ref_plan);
		fftwf_destroy_plan(instance->tar_plan);
		fftwf_destroy_plan(instance->ZNCC_plan);
	}

	FFTCC2D::FFTCC2D(int subset_radius_x, int subset_radius_y, int thread_number) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->thread_number = thread_number;

		for (int i = 0; i < thread_number; i++) {
			FFTW* instance = FFTW::allocate(this->subset_radius_x, this->subset_radius_y);
			this->instance_pool.push_back(instance);
		}
	}

	FFTCC2D::~FFTCC2D() {
		for (auto& instance : this->instance_pool) {
			FFTW::release(instance);
			delete instance;
		}
		this->instance_pool.clear();
	}

	FFTW* FFTCC2D::getInstance(int tid) {
		if (tid >= (int)this->instance_pool.size()) {
			throw std::string("CPU thread ID over limit");
		}

		return this->instance_pool[tid];
	}

	void FFTCC2D::compute(POI2D* POI) {
		//set instance w.r.t. thread id 
		FFTW* current_instance = this->getInstance(omp_get_thread_num());

		int subset_width = this->subset_radius_x * 2;
		int subset_height = this->subset_radius_y * 2;
		int subset_size = subset_width * subset_height;

		//set initial guess of displacement as offset
		Point2D initial_displacement(POI->deformation.u, POI->deformation.v);

		//create reference subset and target subset
		float ref_mean = 0;
		float tar_mean = 0;
		float ref_norm = 0;
		float tar_norm = 0;

		for (int r = 0; r < subset_height; r++) {
			for (int c = 0; c < subset_width; c++) {
				Point2D current_point(POI->x + c - this->subset_radius_x, POI->y + r - this->subset_radius_y);
				current_instance->ref_subset[r * subset_width + c] = this->ref_img->eg_mat(current_point.y, current_point.x);
				ref_mean += current_instance->ref_subset[r * subset_width + c];
				//fill the target subset with initial guess
				current_instance->tar_subset[r * subset_width + c] = this->tar_img->eg_mat(current_point.y, current_point.x);
				tar_mean += current_instance->tar_subset[r * subset_width + c];
			}
		}
		ref_mean /= subset_size;
		tar_mean /= subset_size;

		for (int i = 0; i < subset_size; i++) {
			current_instance->ref_subset[i] -= ref_mean;
			current_instance->tar_subset[i] -= tar_mean;
			ref_norm += powf(current_instance->ref_subset[i], 2);
			tar_norm += powf(current_instance->tar_subset[i], 2);
		}

		fftwf_execute(current_instance->ref_plan);
		fftwf_execute(current_instance->tar_plan);

		for (int n = 0; n < subset_width * (this->subset_radius_y + 1); n++) {
			current_instance->ZNCC_freq[n][0] = (current_instance->ref_freq[n][0] * current_instance->tar_freq[n][0])
				+ (current_instance->ref_freq[n][1] * current_instance->tar_freq[n][1]);
			current_instance->ZNCC_freq[n][1] = (current_instance->ref_freq[n][0] * current_instance->tar_freq[n][1])
				- (current_instance->ref_freq[n][1] * current_instance->tar_freq[n][0]);
		}

		fftwf_execute(current_instance->ZNCC_plan);

		//search for max ZCC, then normalize it to get ZNCC
		float max_ZNCC = -2;
		int max_ZNCC_index = 0;
		for (int i = 0; i < subset_size; i++)
		{
			if (current_instance->ZNCC[i] > max_ZNCC)
			{
				max_ZNCC = current_instance->ZNCC[i];
				max_ZNCC_index = i;
			}
		}
		int local_displacement_u = max_ZNCC_index % subset_width;
		int local_displacement_v = max_ZNCC_index / subset_width;

		if (local_displacement_u > this->subset_radius_x)
			local_displacement_u -= subset_width;
		if (local_displacement_v > this->subset_radius_y)
			local_displacement_v -= subset_height;

		float displacement_u = (float)local_displacement_u + initial_displacement.x;
		float displacement_v = (float)local_displacement_v + initial_displacement.y;

		//store the final result
		POI->deformation.u = displacement_u;
		POI->deformation.v = displacement_v;

		//save the result for output
		POI->result.u0 = initial_displacement.x;
		POI->result.v0 = initial_displacement.y;
		POI->result.u = displacement_u;
		POI->result.v = displacement_v;
		POI->result.ZNCC = max_ZNCC / (sqrtf(ref_norm * tar_norm) * subset_size);
	}

	void FFTCC2D::compute(std::vector<POI2D>& POI_queue) {
#pragma omp parallel for
		for (int i = 0; i < POI_queue.size(); ++i) {
			this->compute(&POI_queue[i]);
		}
	}

}//namespace opencorr