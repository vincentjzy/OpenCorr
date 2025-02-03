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

#include "oc_feature.h"

namespace opencorr
{
	void Feature2D::setImages(Image2D& ref_img, Image2D& tar_img)
	{
		this->ref_img = &ref_img;
		this->tar_img = &tar_img;
	}

	void Feature3D::setImages(Image3D& ref_img, Image3D& tar_img)
	{
		this->ref_img = &ref_img;
		this->tar_img = &tar_img;
	}

}//namespace opencorr
