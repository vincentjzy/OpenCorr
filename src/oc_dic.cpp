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

#include "oc_dic.h"

namespace opencorr
{
	//DIC
	DIC::DIC() {}

	void DIC::setImages(Image2D& ref_img, Image2D& tar_img)
	{
		this->ref_img = &ref_img;
		this->tar_img = &tar_img;
	}

	void DIC::setSubset(int radius_x, int radius_y)
	{
		subset_radius_x = radius_x;
		subset_radius_y = radius_y;
	}

	void DIC::setSelfAdaptive(bool is_self_adaptive)
	{
		self_adaptive = is_self_adaptive;
	}


	//DVC
	DVC::DVC() {}

	void DVC::setImages(Image3D& ref_img, Image3D& tar_img)
	{
		this->ref_img = &ref_img;
		this->tar_img = &tar_img;
	}

	void DVC::setSubset(int radius_x, int radius_y, int radius_z)
	{
		subset_radius_x = radius_x;
		subset_radius_y = radius_y;
		subset_radius_z = radius_z;
	}


	bool sortByZNCC(const POI2D& p1, const POI2D& p2)
	{
		return p1.result.zncc > p2.result.zncc;
	}

	bool sortByDistance(const KeypointIndex& kp1, const KeypointIndex& kp2)
	{
		return kp1.distance < kp2.distance;
	}

}//namespace opencorr
