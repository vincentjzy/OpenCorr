/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2025, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_fftcc.h"

namespace opencorr
{
	std::unique_ptr<FFTW> FFTW::allocate(int subset_radius_x, int subset_radius_y)
	{
		int width = 2 * subset_radius_x;
		int height = 2 * subset_radius_y;
		int buffer_length = width * (subset_radius_y + 1);
		unsigned int subset_size = width * height;

		std::unique_ptr<FFTW> FFTW_instance = std::make_unique<FFTW>();

		FFTW_instance->ref_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		FFTW_instance->tar_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		FFTW_instance->zncc_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);

		FFTW_instance->ref_subset = new float[subset_size];
		FFTW_instance->tar_subset = new float[subset_size];
		FFTW_instance->zncc = new float[subset_size];

#pragma omp critical 
		{
			FFTW_instance->ref_plan = fftwf_plan_dft_r2c_2d(width, height, FFTW_instance->ref_subset, FFTW_instance->ref_freq, FFTW_ESTIMATE);
			FFTW_instance->tar_plan = fftwf_plan_dft_r2c_2d(width, height, FFTW_instance->tar_subset, FFTW_instance->tar_freq, FFTW_ESTIMATE);
			FFTW_instance->zncc_plan = fftwf_plan_dft_c2r_2d(width, height, FFTW_instance->zncc_freq, FFTW_instance->zncc, FFTW_ESTIMATE);
		}

		return FFTW_instance;
	}

	std::unique_ptr<FFTW> FFTW::allocate(int subset_radius_x, int subset_radius_y, int subset_radius_z)
	{
		int dim_x = 2 * subset_radius_x;
		int dim_y = 2 * subset_radius_y;
		int dim_z = 2 * subset_radius_z;
		int buffer_length = dim_x * dim_y * (subset_radius_z + 1);
		unsigned int subset_size = dim_x * dim_y * dim_z;

		std::unique_ptr<FFTW> FFTW_instance = std::make_unique<FFTW>();

		FFTW_instance->ref_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		FFTW_instance->tar_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		FFTW_instance->zncc_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);

		FFTW_instance->ref_subset = new float[subset_size];
		FFTW_instance->tar_subset = new float[subset_size];
		FFTW_instance->zncc = new float[subset_size];

#pragma omp critical 
		{
			FFTW_instance->ref_plan = fftwf_plan_dft_r2c_3d(dim_x, dim_y, dim_z, FFTW_instance->ref_subset, FFTW_instance->ref_freq, FFTW_ESTIMATE);
			FFTW_instance->tar_plan = fftwf_plan_dft_r2c_3d(dim_x, dim_y, dim_z, FFTW_instance->tar_subset, FFTW_instance->tar_freq, FFTW_ESTIMATE);
			FFTW_instance->zncc_plan = fftwf_plan_dft_c2r_3d(dim_x, dim_y, dim_z, FFTW_instance->zncc_freq, FFTW_instance->zncc, FFTW_ESTIMATE);
		}

		return FFTW_instance;
	}

	void FFTW::release(std::unique_ptr<FFTW>& instance)
	{
		delete[] instance->ref_subset;
		delete[] instance->tar_subset;
		delete[] instance->zncc;
		fftw_free(instance->ref_freq);
		fftw_free(instance->tar_freq);
		fftw_free(instance->zncc_freq);
		fftwf_destroy_plan(instance->ref_plan);
		fftwf_destroy_plan(instance->tar_plan);
		fftwf_destroy_plan(instance->zncc_plan);
	}

	void FFTW::update(std::unique_ptr<FFTW>& instance, int subset_radius_x, int subset_radius_y)
	{
		release(instance);

		int width = 2 * subset_radius_x;
		int height = 2 * subset_radius_y;
		int buffer_length = width * (subset_radius_y + 1);
		unsigned int subset_size = width * height;

		instance->ref_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		instance->tar_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		instance->zncc_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);

		instance->ref_subset = new float[subset_size];
		instance->tar_subset = new float[subset_size];
		instance->zncc = new float[subset_size];

#pragma omp critical 
		{
			instance->ref_plan = fftwf_plan_dft_r2c_2d(width, height, instance->ref_subset, instance->ref_freq, FFTW_ESTIMATE);
			instance->tar_plan = fftwf_plan_dft_r2c_2d(width, height, instance->tar_subset, instance->tar_freq, FFTW_ESTIMATE);
			instance->zncc_plan = fftwf_plan_dft_c2r_2d(width, height, instance->zncc_freq, instance->zncc, FFTW_ESTIMATE);
		}
	}

	void FFTW::update(std::unique_ptr<FFTW>& instance, int subset_radius_x, int subset_radius_y, int subset_radius_z)
	{
		release(instance);

		int dim_x = 2 * subset_radius_x;
		int dim_y = 2 * subset_radius_y;
		int dim_z = 2 * subset_radius_z;
		int buffer_length = dim_x * dim_y * (subset_radius_z + 1);
		unsigned int subset_size = dim_x * dim_y * dim_z;

		instance->ref_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		instance->tar_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);
		instance->zncc_freq = (fftwf_complex*)fftw_malloc(sizeof(fftwf_complex) * buffer_length);

		instance->ref_subset = new float[subset_size];
		instance->tar_subset = new float[subset_size];
		instance->zncc = new float[subset_size];

#pragma omp critical 
		{
			instance->ref_plan = fftwf_plan_dft_r2c_3d(dim_x, dim_y, dim_z, instance->ref_subset, instance->ref_freq, FFTW_ESTIMATE);
			instance->tar_plan = fftwf_plan_dft_r2c_3d(dim_x, dim_y, dim_z, instance->tar_subset, instance->tar_freq, FFTW_ESTIMATE);
			instance->zncc_plan = fftwf_plan_dft_c2r_3d(dim_x, dim_y, dim_z, instance->zncc_freq, instance->zncc, FFTW_ESTIMATE);
		}
	}

	//FFT accelerated cross correlation 2D
	std::unique_ptr<FFTW>& FFTCC2D::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	FFTCC2D::FFTCC2D(int subset_radius_x, int subset_radius_y, int thread_number)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = FFTW::allocate(subset_radius_x, subset_radius_y);
		}
	}

	FFTCC2D::~FFTCC2D()
	{
		for (auto& instance : instance_pool)
		{
			FFTW::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<FFTW >>().swap(instance_pool);
	}

	void FFTCC2D::prepare() {}

	void FFTCC2D::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<FFTW>& fftcc = getInstance(omp_get_thread_num());

		int subset_width = subset_radius_x * 2;
		int subset_height = subset_radius_y * 2;
		int subset_size = subset_width * subset_height;

		//set initial guess of displacement
		Point2D initial_displacement(poi->deformation.u, poi->deformation.v);

		//check if the position of POI is too close to the boundary
		if ((int)poi->x < subset_radius_x || (int)poi->x >= ref_img->width - subset_radius_x
			|| (int)poi->y < subset_radius_y || (int)poi->y >= ref_img->height - subset_radius_y
			|| int(poi->x + initial_displacement.x) < subset_radius_x || int(poi->x + initial_displacement.x) >= ref_img->width - subset_radius_x
			|| int(poi->y + initial_displacement.y) < subset_radius_y || int(poi->y + initial_displacement.y) >= ref_img->height - subset_radius_y)
		{
			return;
		}

		//initialize mean and norm in subsets
		float ref_mean = 0.f;
		float tar_mean = 0.f;
		float ref_norm = 0.f;
		float tar_norm = 0.f;

		for (int r = 0; r < subset_height; r++)
		{
			for (int c = 0; c < subset_width; c++)
			{
				//fill reference subset
				Point2D ref_point(poi->x + c - subset_radius_x, poi->y + r - subset_radius_y);
				float value = ref_img->eg_mat((int)ref_point.y, (int)ref_point.x);
				fftcc->ref_subset[r * subset_width + c] = value;
				ref_mean += value;

				//fill the target subset with initial guess of displacement
				Point2D tar_point = ref_point + initial_displacement;
				value = tar_img->eg_mat((int)tar_point.y, (int)tar_point.x);
				fftcc->tar_subset[r * subset_width + c] = value;
				tar_mean += value;
			}
		}
		ref_mean /= subset_size;
		tar_mean /= subset_size;

		//zero-mean operation of gray-scale values in the two subsets
		for (int i = 0; i < subset_size; i++)
		{
			fftcc->ref_subset[i] -= ref_mean;
			fftcc->tar_subset[i] -= tar_mean;
			ref_norm += fftcc->ref_subset[i] * fftcc->ref_subset[i];
			tar_norm += fftcc->tar_subset[i] * fftcc->tar_subset[i];
		}

		fftwf_execute(fftcc->ref_plan);
		fftwf_execute(fftcc->tar_plan);

		int buffer_length = subset_width * (subset_radius_y + 1);
		for (int n = 0; n < buffer_length; n++)
		{
			fftcc->zncc_freq[n][0] = (fftcc->ref_freq[n][0] * fftcc->tar_freq[n][0]) + (fftcc->ref_freq[n][1] * fftcc->tar_freq[n][1]);
			fftcc->zncc_freq[n][1] = (fftcc->ref_freq[n][0] * fftcc->tar_freq[n][1]) - (fftcc->ref_freq[n][1] * fftcc->tar_freq[n][0]);
		}

		fftwf_execute(fftcc->zncc_plan);

		//search for max ZCC
		float max_zncc = -2.f;
		int max_zncc_index = 0;
		for (int i = 0; i < subset_size; i++)
		{
			if (fftcc->zncc[i] > max_zncc)
			{
				max_zncc = fftcc->zncc[i];
				max_zncc_index = i;
			}
		}
		int local_displacement_u = max_zncc_index % subset_width;
		int local_displacement_v = max_zncc_index / subset_width;

		if (local_displacement_u > subset_radius_x)
		{
			local_displacement_u -= subset_width;
		}
		if (local_displacement_v > subset_radius_y)
		{
			local_displacement_v -= subset_height;
		}

		//store the final results
		poi->deformation.u = (float)local_displacement_u + initial_displacement.x;
		poi->deformation.v = (float)local_displacement_v + initial_displacement.y;

		poi->result.u0 = initial_displacement.x;
		poi->result.v0 = initial_displacement.y;
		poi->result.zncc = max_zncc / (sqrt(ref_norm * tar_norm) * subset_size); //convert ZCC to ZNCC
	}

	void FFTCC2D::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}



	//FFT accelerated cross correlation 3D
	std::unique_ptr<FFTW>& FFTCC3D::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	FFTCC3D::FFTCC3D(int subset_radius_x, int subset_radius_y, int subset_radius_z, int thread_number)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->subset_radius_z = subset_radius_z;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = FFTW::allocate(subset_radius_x, subset_radius_y, subset_radius_z);
		}
	}

	FFTCC3D::~FFTCC3D()
	{
		for (auto& instance : instance_pool)
		{
			FFTW::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<FFTW>>().swap(instance_pool);
	}

	void FFTCC3D::prepare() {}

	void FFTCC3D::compute(POI3D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<FFTW>& fftcc = getInstance(omp_get_thread_num());

		int subset_dim_x = subset_radius_x * 2;
		int subset_dim_y = subset_radius_y * 2;
		int subset_dim_z = subset_radius_z * 2;
		int subset_size = subset_dim_x * subset_dim_y * subset_dim_z;

		//set initial guess of displacement
		Point3D initial_displacement(poi->deformation.u, poi->deformation.v, poi->deformation.w);

		//initialize mean and norm in subsets
		float ref_mean = 0.f;
		float tar_mean = 0.f;
		float ref_norm = 0.f;
		float tar_norm = 0.f;

		for (int i = 0; i < subset_dim_z; i++)
		{
			for (int j = 0; j < subset_dim_y; j++)
			{
				for (int k = 0; k < subset_dim_x; k++)
				{
					//fill the reference subset
					Point3D ref_point(poi->x + k - subset_radius_x, poi->y + j - subset_radius_y, poi->z + i - subset_radius_z);
					float value = ref_img->vol_mat[(int)ref_point.z][(int)ref_point.y][(int)ref_point.x];
					fftcc->ref_subset[(i * subset_dim_y + j) * subset_dim_x + k] = value;
					ref_mean += value;

					//fill the target subset with initial guess of displacement
					Point3D tar_point = ref_point + initial_displacement;
					value = tar_img->vol_mat[(int)tar_point.z][(int)tar_point.y][(int)tar_point.x];
					fftcc->tar_subset[(i * subset_dim_y + j) * subset_dim_x + k] = value;
					tar_mean += value;
				}
			}
		}
		ref_mean /= subset_size;
		tar_mean /= subset_size;

		//zero-mean operation of gray-scale values in the two subsets
		for (int i = 0; i < subset_size; i++)
		{
			fftcc->ref_subset[i] -= ref_mean;
			fftcc->tar_subset[i] -= tar_mean;
			ref_norm += fftcc->ref_subset[i] * fftcc->ref_subset[i];
			tar_norm += fftcc->tar_subset[i] * fftcc->tar_subset[i];
		}

		fftwf_execute(fftcc->ref_plan);
		fftwf_execute(fftcc->tar_plan);

		unsigned int buffer_length = subset_dim_x * subset_dim_y * (subset_radius_z + 1);
		for (unsigned int n = 0; n < buffer_length; n++)
		{
			fftcc->zncc_freq[n][0] = (fftcc->ref_freq[n][0] * fftcc->tar_freq[n][0]) + (fftcc->ref_freq[n][1] * fftcc->tar_freq[n][1]);
			fftcc->zncc_freq[n][1] = (fftcc->ref_freq[n][0] * fftcc->tar_freq[n][1]) - (fftcc->ref_freq[n][1] * fftcc->tar_freq[n][0]);
		}

		fftwf_execute(fftcc->zncc_plan);

		//search for max ZCC
		float max_zncc = -2.f;
		int max_zncc_index = 0;
		for (int i = 0; i < subset_size; i++)
		{
			if (fftcc->zncc[i] > max_zncc)
			{
				max_zncc = fftcc->zncc[i];
				max_zncc_index = i;
			}
		}
		int local_displacement_u = max_zncc_index % subset_dim_x;
		int local_displacement_v = (max_zncc_index / subset_dim_x) % subset_dim_y;
		int local_displacement_w = max_zncc_index / (subset_dim_x * subset_dim_y);

		if (local_displacement_u > subset_radius_x)
		{
			local_displacement_u -= subset_dim_x;
		}
		if (local_displacement_v > subset_radius_y)
		{
			local_displacement_v -= subset_dim_y;
		}
		if (local_displacement_w > subset_radius_z)
		{
			local_displacement_w -= subset_dim_z;
		}

		//store the final results
		poi->deformation.u = (float)local_displacement_u + initial_displacement.x;
		poi->deformation.v = (float)local_displacement_v + initial_displacement.y;
		poi->deformation.w = (float)local_displacement_w + initial_displacement.z;

		poi->result.u0 = initial_displacement.x;
		poi->result.v0 = initial_displacement.y;
		poi->result.w0 = initial_displacement.z;
		poi->result.zncc = max_zncc / (sqrt(ref_norm * tar_norm) * subset_size); //convert ZCC to ZNCC
	}

	void FFTCC3D::compute(std::vector<POI3D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr