#pragma once

#include <fstream>
#include <sophus/se3.hpp>
#include "Eigen.h"

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame() {}

    Frame(float* depthMap, const Matrix3d& depthIntrinsics, const Matrix4d& depthExtrinsics, const unsigned width, const unsigned height, unsigned downsampleFactor = 1, float maxDistance = 0.1f) {
		// Get depth intrinsics.
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);
        intrinsic_matrix = depthIntrinsics;

		// Compute inverse depth extrinsics.
		Matrix4d depthExtrinsicsInv = depthExtrinsics.inverse();
		Matrix3d rotationInv = depthExtrinsicsInv.block(0, 0, 3, 3);
		Vector3d translationInv = depthExtrinsicsInv.block(0, 3, 3, 1);

		// Back-project the pixel depths into the camera space.
		std::vector<Vector3d> pointsTmp(width * height);

		// For every pixel row.
		#pragma omp parallel for
		for (int v = 0; v < height; ++v) {
			// For every pixel in a row.
			for (int u = 0; u < width; ++u) {
				unsigned int idx = v*width + u; // linearized index
				double depth = depthMap[idx];
				if (depth == MINF) {
					pointsTmp[idx] = Vector3d(MINF, MINF, MINF);
				}
				else {
					// Back-projection to camera space.
					pointsTmp[idx] = rotationInv * Vector3d((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth) + translationInv;
				}
			}
		}

		// We need to compute derivatives and then the normalized normal vector (for valid pixels).
		std::vector<Vector3d> normalsTmp(width * height);

		#pragma omp parallel for
		for (int v = 1; v < height - 1; ++v) {
			for (int u = 1; u < width - 1; ++u) {
				unsigned int idx = v*width + u; // linearized index

				const double du = depthMap[idx + 1] - depthMap[idx - 1];
				const double dv = depthMap[idx + width] - depthMap[idx - width];
				if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistance || abs(dv) > maxDistance) {
					normalsTmp[idx] = Vector3d(MINF, MINF, MINF);
					continue;
				}

				// TODO: Compute the normals using the cross product of approximate tangent vectors.

                const double xu = 1;
                const double xv = 0;
                const double yu = 0;
                const double yv = 1;

                normalsTmp[idx] = Vector3d(yu*dv - yv*du, xv*du - xu*dv, xu*yv - yu*xv);
				normalsTmp[idx].normalize();
			}
		}

		// We set invalid normals for border regions.
		for (int u = 0; u < width; ++u) {
			normalsTmp[u] = Vector3d(MINF, MINF, MINF);
			normalsTmp[u + (height - 1) * width] = Vector3d(MINF, MINF, MINF);
		}
		for (int v = 0; v < height; ++v) {
			normalsTmp[v * width] = Vector3d(MINF, MINF, MINF);
			normalsTmp[(width - 1) + v * width] = Vector3d(MINF, MINF, MINF);
		}

		// We filter out measurements where either point or normal is invalid.
		const unsigned nPoints = pointsTmp.size();
		m_points.reserve(std::floor(double(nPoints) / downsampleFactor));
		m_normals.reserve(std::floor(double(nPoints) / downsampleFactor));

		for (int i = 0; i < nPoints; i = i + downsampleFactor) {
			const auto& point = pointsTmp[i];
			const auto& normal = normalsTmp[i];

			if (point.allFinite() && normal.allFinite()) {
				m_points.push_back(point);
				m_normals.push_back(normal);
			}
		}
	}

	bool readFromFile(const std::string& filename) {
		std::ifstream is(filename, std::ios::in | std::ios::binary);
		if (!is.is_open()) {
			std::cout << "ERROR: unable to read input file!" << std::endl;
			return false;
		}

		char nBytes;
		is.read(&nBytes, sizeof(char));

		unsigned int n;
		is.read((char*)&n, sizeof(unsigned int));

		if (nBytes == sizeof(float)) {
			float* ps = new float[3 * n];

			is.read((char*)ps, 3 * sizeof(float) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3d p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(float) * n);
			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3d p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}
		else {
			double* ps = new double[3 * n];

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3d p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3d p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}

		return true;
	}

	std::vector<Vector3d>& getPoints() {
		return m_points;
	}

	const std::vector<Vector3d>& getPoints() const {
		return m_points;
	}

	std::vector<Vector3d>& getNormals() {
		return m_normals;
	}

	const std::vector<Vector3d>& getNormals() const {
		return m_normals;
	}

    std::vector<Vector3d>& getGlobalPoints() {
        return m_points_global;
    }

    const Sophus::SE3d& getGlobalPose() const{
        return global_pose;
    }

    const std::vector<double>& getDepthMap() const{
        return m_depth_map;
    }

    const Matrix3d& getIntrinsics() const{
        return intrinsic_matrix;
    }

    const unsigned int getWidth() const{
        return width;
    }

    const unsigned int getHeight() const{
        return height;
    }

private:
	std::vector<Vector3d> m_points;
	std::vector<Vector3d> m_normals;
    std::vector<double> m_depth_map;
    std::vector<Vector3d> m_points_global;
    Sophus::SE3d global_pose;
    Matrix3d intrinsic_matrix;
    size_t width;
    size_t height;

};
