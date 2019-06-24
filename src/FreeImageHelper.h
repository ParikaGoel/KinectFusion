#pragma once

#undef min
#undef max

#include <string>
#include <algorithm>

#include <FreeImage.h>

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

struct FreeImage {

	FreeImage();
	FreeImage(unsigned int width, unsigned int height, unsigned int nChannels = 4);
	FreeImage(const FreeImage& img);
	FreeImage(const std::string& filename);

	~FreeImage();

	void operator=(const FreeImage& other);

	void SetDimensions(unsigned int width, unsigned int height, unsigned int nChannels = 4);

	FreeImage ConvertToIntensity() const;

	bool LoadImageFromFile(const std::string& filename, unsigned int width = 0, unsigned int height = 0);

	bool SaveImageToFile(const std::string& filename, bool flipY = false);

	unsigned int w;
	unsigned int h;
	unsigned int nChannels;
	float* data;
};


struct FreeImageB {

	FreeImageB();
	FreeImageB(unsigned int width, unsigned int height, unsigned int nChannels = 4);
	FreeImageB(const FreeImage& img);
	FreeImageB(const std::string& filename);

	~FreeImageB();

	void operator=(const FreeImageB& other);

	void SetDimensions(unsigned int width, unsigned int height, unsigned int nChannels = 4);

	bool LoadImageFromFile(const std::string& filename, unsigned int width = 0, unsigned int height = 0);

	bool SaveImageToFile(const std::string& filename, bool flipY = false);

	unsigned int w;
	unsigned int h;
	unsigned int nChannels;
	BYTE* data;
};

struct FreeImageU16F {

	FreeImageU16F();
	FreeImageU16F(const std::string& filename);

	~FreeImageU16F();

	bool LoadImageFromFile(const std::string& filename, unsigned int width = 0, unsigned int height = 0);

	unsigned int w;
	unsigned int h;
	unsigned int nChannels;
	float* data;
};
