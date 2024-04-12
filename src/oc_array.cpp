/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2024, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_array.h"

namespace opencorr
{
	float** new2D(int dimension1, int dimension2)
	{
		float** ptr = nullptr;
		createPtr(ptr, dimension1, dimension2);
		return ptr;
	}

	void delete2D(float**& ptr)
	{
		if (ptr == nullptr) return;
		destroyPtr(ptr);
	}

	float*** new3D(int dimension1, int dimension2, int dimension3)
	{
		float*** ptr = nullptr;
		createPtr(ptr, dimension1, dimension2, dimension3);
		return ptr;
	}

	void delete3D(float***& ptr)
	{
		if (ptr == nullptr) return;
		destroyPtr(ptr);
	}

	float**** new4D(int dimension1, int dimension2, int dimension3, int dimension4)
	{
		float**** ptr = nullptr;
		createPtr(ptr, dimension1, dimension2, dimension3, dimension4);
		return ptr;
	}

	void delete4D(float****& ptr)
	{
		if (ptr == nullptr) return;
		destroyPtr(ptr);
	}

}//namespace opencorr

