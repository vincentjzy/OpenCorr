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

#include "oc_strain.h"

namespace opencorr
{
	Strain2D::Strain2D(int subregion_radius, int min_neighor_num, std::vector<POI2D>& poi_queue) {
		this->subregion_radius = subregion_radius;
		this->min_neighbor_num = min_neighbor_num;
		this->poi2d_queue = poi_queue;

		this->zncc_threshold = 0.9f;
		this->description = 1;
	}

	Strain2D::Strain2D(int subregion_radius, int min_neighbor_num, std::vector<POI2DS>& poi_queue) {
		this->subregion_radius = subregion_radius;
		this->min_neighbor_num = min_neighbor_num;
		this->poi2ds_queue = poi_queue;

		this->zncc_threshold = 0.9f;
		this->description = 1;
	}

	Strain2D::~Strain2D() {
	}

	int Strain2D::getSubregionRadius() const {
		return subregion_radius;
	}

	int Strain2D::getMinNeighborNumber() const {
		return min_neighbor_num;
	}

	float Strain2D::getZNCCThreshold() const {
		return zncc_threshold;
	}

	void Strain2D::setSubregionRadius(int subresion_radius) {
		this->subregion_radius = subregion_radius;
	}

	void Strain2D::setMinNeighborNumer(int min_neighbor_num) {
		this->min_neighbor_num = min_neighbor_num;
	}

	void Strain2D::setZNCCthreshold(float zncc_threshold) {
		this->zncc_threshold = zncc_threshold;
	}

	void Strain2D::setDescription(int description) {
		this->description = description;
	}

	void Strain2D::prepare() {
	}

	void Strain2D::setPOIQueue(std::vector<POI2D>& poi_queue) {
		this->poi2d_queue = poi_queue;
	}

	void Strain2D::compute(POI2D* poi) {
		//calculation of Green-Lagrangian strain
		//brutal force search for adjacent POIs in subregion
		std::vector<PointIndex2D> pois_sorted_index;
		std::vector<POI2D> pois_for_fit;

		//sort the poi queue in a descendent order of distance to the POI
		int queue_size = (int)poi2d_queue.size();
		for (int i = 0; i < queue_size; ++i) {
			Point2D distance = poi2d_queue[i] - (Point2D)*poi;
			PointIndex2D current_poi_idx;
			current_poi_idx.index_in_queue = i;
			current_poi_idx.distance_to_poi = distance.vectorNorm();
			pois_sorted_index.push_back(current_poi_idx);
		}

		std::sort(pois_sorted_index.begin(), pois_sorted_index.end(), sortByDistance);

		//pick the neighbor POIs facet fitting
		int i = 0;
		while ((pois_sorted_index[i].distance_to_poi < subregion_radius || pois_for_fit.size() <= min_neighbor_num)) {
			if (poi2d_queue[pois_sorted_index[i].index_in_queue].result.zncc >= zncc_threshold) {
				pois_for_fit.push_back(poi2d_queue[pois_sorted_index[i].index_in_queue]);
			}
			i++;
		}

		int fit_queue_size = (int)pois_for_fit.size();

		//create matrices of displacments
		Eigen::VectorXf u_vector(fit_queue_size);
		Eigen::VectorXf v_vector(fit_queue_size);
		Eigen::VectorXf w_vector(fit_queue_size);

		//construct coefficient matrix
		Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(fit_queue_size, 3);

		//fill coefficient matrix, u_vector, v_vector, and w_vector
		for (int j = 0; j < fit_queue_size; j++) {
			coefficient_matrix(j, 0) = 1.f;
			coefficient_matrix(j, 1) = pois_for_fit[j].x - poi->x;
			coefficient_matrix(j, 2) = pois_for_fit[j].y - poi->y;

			u_vector(j) = pois_for_fit[j].deformation.u;
			v_vector(j) = pois_for_fit[j].deformation.v;
		}

		//solve the equations to obtain gradients of u, v, and w
		Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
		Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
		float ux = u_gradient(1, 0);
		float uy = u_gradient(2, 0);
		float vx = v_gradient(1, 0);
		float vy = v_gradient(2, 0);

		//calculate the strains and save them for output
		poi->strain.exx = ux + 0.5f * (ux * ux + vx * vx);
		poi->strain.eyy = vy + 0.5f * (uy * uy + vy * vy);
		poi->strain.exy = 0.5f * (uy + vx + uy * ux + vy * vx);
	}

	void Strain2D::compute(std::vector<POI2D>& poi_queue) {
#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}


	void Strain2D::setPOIQueue(std::vector<POI2DS>& poi_queue) {
		this->poi2ds_queue = poi_queue;
	}

	void Strain2D::compute(POI2DS* poi) {
		//calculation of Green-Lagrangian strain
		//brutal force search for adjacent POIs in subregion
		std::vector<PointIndex2D> pois_sorted_index;
		std::vector<POI2DS> pois_for_fit;

		//sort the poi queue in a descendent order of distance to the POI
		int queue_size = (int)poi2ds_queue.size();
		for (int i = 0; i < queue_size; ++i) {
			Point2D distance = poi2ds_queue[i] - (Point2D)*poi;
			PointIndex2D current_poi_idx;
			current_poi_idx.index_in_queue = i;
			current_poi_idx.distance_to_poi = distance.vectorNorm();
			pois_sorted_index.push_back(current_poi_idx);
		}

		std::sort(pois_sorted_index.begin(), pois_sorted_index.end(), sortByDistance);

		//pick the neighbor POIs facet fitting
		int i = 0;
		while (pois_sorted_index[i].distance_to_poi < subregion_radius || pois_for_fit.size() <= min_neighbor_num) {
			if (poi2ds_queue[pois_sorted_index[i].index_in_queue].result.r1r2_zncc >= zncc_threshold
				&& poi2ds_queue[pois_sorted_index[i].index_in_queue].result.r1t1_zncc >= zncc_threshold
				&& poi2ds_queue[pois_sorted_index[i].index_in_queue].result.r1t2_zncc >= zncc_threshold) {
				pois_for_fit.push_back(poi2ds_queue[pois_sorted_index[i].index_in_queue]);
			}
			i++;
		}

		int fit_queue_size = (int)pois_for_fit.size();

		//create matrices of displacments
		Eigen::VectorXf u_vector(fit_queue_size);
		Eigen::VectorXf v_vector(fit_queue_size);
		Eigen::VectorXf w_vector(fit_queue_size);

		//construct coefficient matrix
		Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(fit_queue_size, 4);

		//fill coefficient matrix, u_vector, v_vector, and w_vector
		for (int j = 0; j < fit_queue_size; j++) {
			Point3D current_pt_3d = pois_for_fit[j].ref_coor - poi->ref_coor;
			coefficient_matrix(j, 0) = 1.f;
			coefficient_matrix(j, 1) = current_pt_3d.x;
			coefficient_matrix(j, 2) = current_pt_3d.y;
			coefficient_matrix(j, 3) = current_pt_3d.z;

			u_vector(j) = pois_for_fit[j].deformation.u;
			v_vector(j) = pois_for_fit[j].deformation.v;
			w_vector(j) = pois_for_fit[j].deformation.w;
		}

		//solve the equations to obtain gradients of u, v, and w
		Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
		Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
		Eigen::VectorXf w_gradient = coefficient_matrix.colPivHouseholderQr().solve(w_vector);
		float ux = u_gradient(1, 0);
		float uy = u_gradient(2, 0);
		float uz = u_gradient(3, 0);
		float vx = v_gradient(1, 0);
		float vy = v_gradient(2, 0);
		float vz = v_gradient(3, 0);
		float wx = w_gradient(1, 0);
		float wy = w_gradient(2, 0);
		float wz = w_gradient(3, 0);

		//calculate the strains and save them for output
		poi->strain.exx = ux + 0.5f * (ux * ux + vx * vx + wx * wx);
		poi->strain.eyy = vy + 0.5f * (uy * uy + vy * vy + wy * wy);
		poi->strain.ezz = wz + 0.5f * (uz * uz + vz * vz + wz * wz);
		poi->strain.exy = 0.5f * (uy + vx + uy * ux + vy * vx + wy * wx);
		poi->strain.eyz = 0.5f * (vz + wy + uz * uy + vz * vy + wz * wy);
		poi->strain.ezx = 0.5f * (wx + uz + ux * uz + vx * vz + wx * wz);
	}

	void Strain2D::compute(std::vector<POI2DS>& poi_queue) {
#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}

	bool sortByDistance(const PointIndex2D& p1, const PointIndex2D& p2) {
		return p1.distance_to_poi < p2.distance_to_poi;
	}

}//namespace opencorr
