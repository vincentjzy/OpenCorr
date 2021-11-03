/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#pragma once

#ifndef  _IO_H_
#define  _IO_H_

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen>

#include "oc_poi.h"

using std::vector;
using std::string;

namespace opencorr
{
	class IO2D {
	private:
		string file_path;
		string delimiter;
		int width, height;

	public:
		IO2D();
		~IO2D();

		string getPath() const;
		string getDilimiter() const;
		int getWidth() const;
		int getHeight() const;
		void setPath(string file_path);
		void setDelimiter(string delimiter);
		void setWidth(int width);
		void setHeight(int height);

		//load deformation of POIs from saved date table
		vector<POI2D> loadTable2D();

		//load locations of POIs from csv table
		vector<Point2D> loadPOI2D();

		void saveTable2D(vector<POI2D>& poi_queue);
		void saveDeformationTable2D(vector<POI2D>& poi_queue);
		//variable: 'u', 'v', 'z'(zncc), 'c'(convergence), 'i'(iteration), 'f'(feature), 'x' (exx), 'y' (eyy), 'g' (exy)
		void saveMap2D(vector<POI2D>& poi_queue, char variable);

		void saveTable2DS(vector<POI2DS>& poi_queue);

	};

}//namespace opencorr

#endif //_IO_H_