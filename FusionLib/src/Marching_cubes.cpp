//
// Created by phili on 20.07.2019.
//

#include "Marching_cubes.hpp"
//#include <lookup_tables.hpp>


struct VoxelWCoords {
	Voxel _data;
	int _x;
	int _y;
	int _z;
};
struct triangleShape {
	Eigen::Vector3d _idx1;
	Eigen::Vector3d _idx2;
	Eigen::Vector3d _idx3;
	Vector4uc color;

};

Eigen::Vector3d interpolate(VoxelWCoords v1, VoxelWCoords v2) {
	double t = (0 - v1._data.tsdf) / (v2._data.tsdf - v1._data.tsdf);
	Eigen::Vector3d vec1, vec2;
	vec1 << v1._x, v1._y, v1._z;
	vec2 << v2._x, v2._y, v2._z;
//	if(t!=0.5)std::cout<<"t:"<<t<<"tsdf1: "<<v1._data.tsdf<<"tsdf2: "<<v2._data.tsdf<<std::endl;
	return vec1 + t * (vec2 - vec1);
}

const int calculateCoordinateToIndex(int x, int y, int z, Eigen::Vector3i volumeSize) {
	return x + (y * volumeSize.x()) + (z * volumeSize.x() * volumeSize.y());
}

std::string printTriangleVertices(triangleShape tri, double voxelScale) {
	std::stringstream color;
//	std::cout<<static_cast<unsigned>(tri.color.x())<<" "<<static_cast<unsigned>(tri.color.y())<<" "
//																							 ""<<static_cast<unsigned>(tri.color.z())<<" "
//																												""<<std::endl;

	color << static_cast<unsigned>(tri.color.x()) << " " << static_cast<unsigned>(tri.color.y()) << " " << static_cast<unsigned>(tri
	.color.z());
	std::stringstream ss;

	ss << tri._idx1.x() << " " << tri._idx1.y() << " " << tri._idx1.z() << " " << color.str() << std::endl;
	ss << tri._idx2.x() << " " << tri._idx2.y() << " " << tri._idx2.z() << " " << color.str() << std::endl;
	ss << tri._idx3.x() << " " << tri._idx3.y() << " " << tri._idx3.z() << " " << color.str() << std::endl;

	return ss.str();
}

std::string printTriangleFaces(triangleShape tri, double number) {
	std::stringstream ss;
	ss << "3 " << number * 3 << " " << number * 3 + 1 << " " << number * 3 + 2 << " " << std::endl;

	return ss.str();
}

void MarchingCubes::extractMesh(Volume &volume, std::string fileName) {
	std::vector<triangleShape> faces;
	auto volumeSize = volume.getVolumeSize();
	auto voxelScale = volume.getVoxelScale();
	//iterate over all cubes in the volume
	int idx = 1;
	for (int z = 0; z < volumeSize.z() - 1; z++) {
		for (int y = 0; y < volumeSize.y() - 1; y++) {
			for (int x = 0; x < volumeSize.x() - 1; x++) {
				//get all corners of each cube
				std::vector<VoxelWCoords> points;
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x, y, z, volumeSize)], x, y, z});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x + 1, y, z, volumeSize)],
								  x + 1, y, z});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x + 1, y, z + 1, volumeSize)],
								  x + 1, y, z + 1});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x, y, z + 1, volumeSize)], x, y,
								  z + 1});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x, y + 1, z, volumeSize)], x,
								  y + 1, z});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x + 1, y + 1, z, volumeSize)],
								  x + 1, y + 1, z});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x + 1, y + 1, z + 1, volumeSize)],
								  x + 1, y + 1,
								  z + 1});
				points.push_back({volume.getVoxelData()[calculateCoordinateToIndex(x, y + 1, z + 1, volumeSize)], x,
								  y + 1, z + 1});

				//calculate Table Index
				int cubeIndex = 0;
				double isoLevel = 0.0;
				Vector4uc averageColor;
				int contributors = 0;
				bool valid = true;
				for(int i = 0;i<8;i++){
					if(points[i]._data.weight  ==0.)valid = false;
				}
				if(!valid) continue;
				double tsdf =-10;
//				double tsdf =-10;

				if (points[0]._data.tsdf <= isoLevel && points[0]._data.weight != 0) {
					cubeIndex |= 1;
					if(points[0]._data.color.x()<20 && points[0]._data.color.y()<20 && points[0]._data.color.z()<20 ){
//						std::cout<<points[0]._data.tsdf<<std::endl;
					}
					if(points[0]._data.tsdf>tsdf){
						averageColor = points[0]._data.color;
						contributors++;
						tsdf = points[0]._data.tsdf;

					}


				}
				if (points[1]._data.tsdf <= isoLevel && points[1]._data.weight != 0) {
					if(points[1]._data.color.x()<20 && points[1]._data.color.y()<20 && points[1]._data.color.z()<20 ){
//						std::cout<<points[1]._data.tsdf<<std::endl;
					}
					cubeIndex |= 2;
					if(points[1]._data.tsdf>tsdf){
						averageColor = points[1]._data.color;
						contributors++;
						tsdf = points[1]._data.tsdf;

					}
				}
				if (points[2]._data.tsdf <= isoLevel && points[2]._data.weight != 0) {
					if(points[2]._data.color.x()<20 && points[2]._data.color.y()<20 && points[2]._data.color.z()<20 ){
//						std::cout<<points[2]._data.tsdf<<std::endl;
					}
					cubeIndex |= 4;
					if(points[2]._data.tsdf>tsdf){
						averageColor = points[2]._data.color;
						contributors++;
						tsdf = points[2]._data.tsdf;

					}
				}
				if (points[3]._data.tsdf <= isoLevel && points[3]._data.weight != 0) {
					if(points[3]._data.color.x()<20 && points[3]._data.color.y()<20 && points[3]._data.color.z()<20 ){
//						std::cout<<points[3]._data.tsdf<<std::endl;
					}
					cubeIndex |= 8;
					if(points[3]._data.tsdf>tsdf){
						averageColor = points[3]._data.color;
						contributors++;
						tsdf = points[3]._data.tsdf;

					}
				}
				if (points[4]._data.tsdf <= isoLevel && points[4]._data.weight != 0) {
					if(points[4]._data.color.x()<20 && points[4]._data.color.y()<20 && points[4]._data.color.z()<20 ){
//						std::cout<<points[4]._data.tsdf<<std::endl;
					}
					cubeIndex |= 16;
					if(points[4]._data.tsdf>tsdf){
						averageColor = points[4]._data.color;
						contributors++;
						tsdf = points[4]._data.tsdf;

					}
				}
				if (points[5]._data.tsdf <= isoLevel && points[5]._data.weight != 0) {
					if(points[5]._data.color.x()<20 && points[5]._data.color.y()<20 && points[5]._data.color.z()<20 ){
//						std::cout<<points[5]._data.tsdf<<std::endl;
					}
					cubeIndex |= 32;
					if(points[5]._data.tsdf>tsdf){
						averageColor = points[5]._data.color;
						contributors++;
						tsdf = points[5]._data.tsdf;

					}
				}
				if (points[6]._data.tsdf <= isoLevel && points[6]._data.weight != 0) {
					if(points[6]._data.color.x()<20 && points[6]._data.color.y()<20 && points[6]._data.color.z()<20 ){
//						std::cout<<points[6]._data.tsdf<<std::endl;
					}
					cubeIndex |= 64;
					if(points[6]._data.tsdf>tsdf){
						averageColor = points[6]._data.color;
						contributors++;
						tsdf = points[6]._data.tsdf;

					}
				}
				if (points[7]._data.tsdf <= isoLevel && points[7]._data.weight != 0) {
					if(points[7]._data.color.x()<20 && points[7]._data.color.y()<20 && points[7]._data.color.z()<20 ){
//						std::cout<<points[7]._data.tsdf<<std::endl;
					}
					cubeIndex |= 128;
					if(points[7]._data.tsdf>tsdf){
						averageColor = points[7]._data.color;
						contributors++;
						tsdf = points[7]._data.tsdf;
					}
				}
				/*if (contributors != 0) {
					averageColor /= contributors;
				}*/

				//create triangles for printing out
				// Create triangles for current cube configuration
				for (int i = 0; triangulation[cubeIndex][i] != -1; i += 3) {
					// Get indices of corner points A and B for each of the three edges
					// of the cube that need to be joined to form the triangle.
					int a0 = cornerIndexAFromEdge[triangulation[cubeIndex][i]];
					int b0 = cornerIndexBFromEdge[triangulation[cubeIndex][i]];

					int a1 = cornerIndexAFromEdge[triangulation[cubeIndex][i + 1]];
					int b1 = cornerIndexBFromEdge[triangulation[cubeIndex][i + 1]];

					int a2 = cornerIndexAFromEdge[triangulation[cubeIndex][i + 2]];
					int b2 = cornerIndexBFromEdge[triangulation[cubeIndex][i + 2]];

					triangleShape tri;
//					tri._idx1 = interpolate(points[a0], points[b0]) + volume.getOrigin();
//					tri._idx2 = interpolate(points[a1], points[b1]) + volume.getOrigin();
//					tri._idx3 = interpolate(points[a2], points[b2]) + volume.getOrigin();
					tri.color = averageColor;
					tri._idx1 = interpolate(points[a0], points[b0])*voxelScale+volume.getOrigin();
					tri._idx2 = interpolate(points[a1], points[b1])*voxelScale+volume.getOrigin();
					tri._idx3 = interpolate(points[a2], points[b2])*voxelScale+volume.getOrigin();
					faces.push_back(tri);
				}


			}
		}
	}
	std::cout << "idx:" << idx << std::endl;
	std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

	// Write off file.

	std::ofstream outFile(filenameBaseOut + fileName + ".off");
	if (!outFile.is_open()) { std::cout << "Could open File:" << filenameBaseOut << fileName << ".off" << std::endl; };

	// Write header.
	outFile << "COFF" << std::endl;
	outFile << faces.size() * 3 << " " << faces.size() << " 0" << std::endl;
	for (int i = 0; i < faces.size(); i++) {
		outFile << printTriangleVertices(faces[i], volume.getVoxelScale());
	}
	for (int i = 0; i < faces.size(); i++) {
		outFile << printTriangleFaces(faces[i], i);
	}


	outFile.close();

}