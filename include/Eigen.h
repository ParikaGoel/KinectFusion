#pragma once

#ifndef VERBOSE
//#define VERBOSE(msg) {std::cout << msg << std::endl;}
#define VERBOSE(msg)
#endif

#ifndef ASSERT
#define ASSERT(a) {if (!a) { std::cerr << "Error:\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\nFunction: " << __FUNCTION__ << std::endl; while(1); }}
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(ptr) {if(ptr!=nullptr) {delete ptr; ptr = nullptr;}}
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) {if(ptr!=nullptr) {delete[] ptr; ptr = nullptr;}}
#endif

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

#ifndef M_PI
#define M_PI 3.14159265359
#endif


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

typedef Eigen::Matrix<unsigned char, 4, 1> Vector4uc;


EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector4uc)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::MatrixXf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternionf)



using namespace Eigen;

template<typename T,unsigned int n,unsigned m>
std::istream &operator>>(std::istream &in, Matrix<T,n,m> &other)
{
	for(unsigned int i=0; i<other.rows(); i++)
		for(unsigned int j=0; j<other.cols(); j++)
			in >> other(i,j);
	return in;
}

template<typename T,unsigned int n,unsigned m>
std::ostream &operator<<(std::ostream &out, const Matrix<T,n,m> &other)
{
	std::fixed(out);
	for(int i=0; i<other.rows(); i++) {
		out << other(i,0);
		for(int j=1; j<other.cols(); j++) {
			out << "\t" << other(i,j);
		}
		out << std::endl;
	}
	return out;
}

template<typename T>
std::istream &operator>>(std::istream &in, Eigen::Quaternion<T> &other)
{
	in >> other.x() >> other.y() >> other.z() >> other.w();
	return in;
}

template<typename T>
std::ostream &operator<<(std::ostream &out, const Eigen::Quaternion<T> &other)
{
	std::fixed(out);
	out << other.x() << "\t" << other.y() << "\t" << other.z() << "\t" << other.w();
	return out;
}
