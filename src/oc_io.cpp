/*
 * This file is part of OpenCorr, an open-source C++ library for
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

#include <iomanip>

#include "oc_io.h"

using std::ofstream;
using std::ifstream;

namespace opencorr
{
	IO2D::IO2D() {
	}

	IO2D::~IO2D() {
	}

	string IO2D::getPath() const {
		return this->file_path;
	}

	string IO2D::getDilimiter() const {
		return this->delimiter;
	}

	int IO2D::getWidth() const {
		return this->width;
	}

	int IO2D::getHeight()const {
		return this->height;
	}

	void IO2D::setPath(string file_path) {
		this->file_path = file_path;
	}

	void IO2D::setDelimiter(string delimiter) {
		this->delimiter = delimiter;
	}

	void IO2D::setWidth(int width) {
		this->width = width;
	}

	void IO2D::setHeight(int height) {
		this->height = height;
	}

	vector<POI2D> IO2D::loadTable2D() {
		ifstream file_in(this->file_path);
		if (!file_in) {
			std::cerr << "failed to read file " << this->file_path;
		}

		string data_line;
		getline(file_in, data_line);
		vector<POI2D> POI_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;

		while (getline(file_in, data_line)) {
			position1 = 0;
			position2 = 0;
			vector<float> empty_buffer;
			key_buffer.swap(empty_buffer);
			do {
				position2 = data_line.find(this->delimiter, position1);
				if (position2 == string::npos)
					position2 = data_line.length();

				variable = data_line.substr(position1, position2 - position1);
				if (!variable.empty())
					key_buffer.push_back(std::stof(variable));

				position1 = position2 + this->delimiter.length();
			} while (position2 < data_line.length() && position1 < data_line.length());

			float x = key_buffer[0];
			float y = key_buffer[1];
			POI2D current_POI(x, y);

			for (int i = 0; i < 11; i++) {
				current_POI.result.r[i] = key_buffer[2 + i];
			}
			POI_queue.push_back(current_POI);
		}
		return POI_queue;
	}

	void IO2D::saveTable2D(vector<POI2D> POI_queue) {
		std::ofstream file_out(this->file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);
		if (file_out.is_open()) {
			file_out << "x" << this->delimiter;
			file_out << "y" << this->delimiter;

			file_out << "u0" << this->delimiter;
			file_out << "v0" << this->delimiter;
			file_out << "u" << this->delimiter;
			file_out << "v" << this->delimiter;

			file_out << "exx" << this->delimiter;
			file_out << "eyy" << this->delimiter;
			file_out << "exy" << this->delimiter;

			file_out << "zncc" << this->delimiter;
			file_out << "iteration" << this->delimiter;
			file_out << "convergence" << this->delimiter;
			file_out << "feature" << this->delimiter;
			file_out << std::endl;

			for (vector<POI2D>::iterator iter = POI_queue.begin(); iter != POI_queue.end(); ++iter) {
				file_out << iter->x << this->delimiter;
				file_out << iter->y << this->delimiter;

				for (int i = 0; i < 11; i++) {
					file_out << iter->result.r[i] << this->delimiter;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO2D::saveDeformationTable2D(vector<POI2D> POI_queue) {
		std::ofstream file_out(this->file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);
		if (file_out.is_open()) {
			file_out << "x" << this->delimiter;
			file_out << "y" << this->delimiter;

			file_out << "u" << this->delimiter;
			file_out << "ux" << this->delimiter;
			file_out << "uy" << this->delimiter;
			file_out << "uxx" << this->delimiter;
			file_out << "uxy" << this->delimiter;
			file_out << "uyy" << this->delimiter;

			file_out << "v" << this->delimiter;
			file_out << "vx" << this->delimiter;
			file_out << "vy" << this->delimiter;
			file_out << "vxx" << this->delimiter;
			file_out << "vxy" << this->delimiter;
			file_out << "vyy" << this->delimiter;
			file_out << std::endl;

			for (vector<POI2D>::iterator iter = POI_queue.begin(); iter != POI_queue.end(); ++iter) {
				file_out << iter->x << this->delimiter;
				file_out << iter->y << this->delimiter;

				for (int i = 0; i < 12; i++) {
					file_out << iter->deformation.p[i] << this->delimiter;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO2D::saveMap2D(vector<POI2D> POI_queue, int variable) {
		int height = this->getHeight();
		int width = this->getWidth();
		Eigen::MatrixXf output_map = Eigen::MatrixXf::Zero(height, width);

		for (int i = 0; i < POI_queue.size(); ++i) {
			output_map((int)POI_queue[i].y, (int)POI_queue[i].x) = POI_queue[i].result.r[variable];
		}

		std::ofstream file_out(this->file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);
		if (file_out.is_open()) {
			for (int r = 0; r < height; r++) {
				for (int c = 0; c < width; c++) {
					file_out << output_map(r,c) << this->delimiter;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}

}//namespace opencorr