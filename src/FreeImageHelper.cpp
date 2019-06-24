#include "FreeImageHelper.h"

#include <iostream>
#include <cstring>

//#pragma comment(lib, "FreeImage.lib")

FreeImage::FreeImage() : w(0), h(0), nChannels(0), data(nullptr)
{
}

FreeImage::FreeImage(unsigned int width, unsigned int height, unsigned int nChannels) :
	w(width), h(height), nChannels(nChannels), data(new float[nChannels * width*height])
{
}

FreeImage::FreeImage(const FreeImage& img) :
	w(img.w), h(img.h), nChannels(img.nChannels), data(new float[nChannels * img.w*img.h])
{
	memcpy(data, img.data, sizeof(float) * nChannels * w*h);
}

FreeImage::FreeImage(const std::string& filename) : w(0), h(0), nChannels(0), data(nullptr)
{
	LoadImageFromFile(filename);
}

FreeImage::~FreeImage()
{
	if (data != nullptr) delete[] data;
}

void FreeImage::operator=(const FreeImage& other)
{
	if (other.data != this->data)
	{
		SetDimensions(other.w, other.h, other.nChannels);
		memcpy(data, other.data, sizeof(float) * nChannels * w * h);
	}
}

void FreeImage::SetDimensions(unsigned int width, unsigned int height, unsigned int nChannels)
{
	if (data != nullptr) delete[] data;
	w = width;
	h = height;
	this->nChannels = nChannels;
	data = new float[nChannels * width * height];
}

FreeImage FreeImage::ConvertToIntensity() const
{
	FreeImage result(w, h, 1);

	for (unsigned int j = 0; j < h; ++j)
	{
		for (unsigned int i = 0; i < w; ++i)
		{
			float sum = 0.0f;
			for (unsigned int c = 0; c < nChannels; ++c)
			{
				if (data[nChannels * (i + w*j) + c] == MINF)
				{
					sum = MINF;
					break;
				}
				else
				{
					sum += data[nChannels * (i + w*j) + c];
				}
			}
			if (sum == MINF) result.data[i + w*j] = MINF;
			else result.data[i + w*j] = sum / nChannels;
		}
	}

	return result;
}

bool FreeImage::LoadImageFromFile(const std::string& filename, unsigned int width, unsigned int height)
{
	FreeImage_Initialise();
	if (data != nullptr) delete[] data;

	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	//pointer to the image, once loaded
	FIBITMAP *dib(0);

	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename.c_str(), 0);
	if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename(filename.c_str());
	if (fif == FIF_UNKNOWN) return false;

	//check that the plugin has reading capabilities and load the file
	if (FreeImage_FIFSupportsReading(fif)) dib = FreeImage_Load(fif, filename.c_str());
	if (!dib) return false;

	// Convert to RGBA float images
	FIBITMAP* hOldImage = dib;
	dib = FreeImage_ConvertToRGBAF(hOldImage); // ==> 4 channels
	FreeImage_Unload(hOldImage);

	//get the image width and height
	w = FreeImage_GetWidth(dib);
	h = FreeImage_GetHeight(dib);

	// rescale to fit width and height
	if (width != 0 && height != 0)
	{
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_Rescale(hOldImage, width, height, FILTER_CATMULLROM);
		FreeImage_Unload(hOldImage);
		w = width;
		h = height;
	}

	//retrieve the image data
	BYTE* bits = FreeImage_GetBits(dib);

	//if this somehow one of these failed (they shouldn't), return failure
	if ((bits == 0) || (w == 0) || (h == 0))
		return false;

	nChannels = 4;

	// copy image data
	data = new float[nChannels * w * h];

	// flip
	for (int y = 0; y < (int)h; ++y)
	{
		memcpy(&(data[y*nChannels * w]), &bits[sizeof(float) * (h-1-y) * nChannels * w], sizeof(float) * nChannels * w);
	}
	//memcpy(data, bits, sizeof(float) * nChannels * w * h);

	//Free FreeImage's copy of the data
	FreeImage_Unload(dib);

	return true;
}

bool FreeImage::SaveImageToFile(const std::string& filename, bool flipY)
{
	FREE_IMAGE_FORMAT fif = FIF_PNG;
	FIBITMAP *dib = FreeImage_Allocate(w, h, 24);
	RGBQUAD color;
	for (unsigned int j = 0; j < h; j++) {
		for (unsigned int i = 0; i < w; i++) {
			unsigned char col[3] = { 0, 0, 0 };

			for (unsigned int c = 0; c < nChannels && c < 3; ++c)
			{
				//col[c] = std::min(std::max(0, (int)(255.0f*data[nChannels * (w*j + i) + c])), 255);
				col[c] = std::min(std::max(0, (int)(255.0f*data[nChannels * (w*j + i) + c])), 255);
			}

			color.rgbRed = col[0];
			color.rgbGreen = col[1];
			color.rgbBlue = col[2];
			if (!flipY)	FreeImage_SetPixelColor(dib, i, h - 1 - j, &color);
			else		FreeImage_SetPixelColor(dib, i, j, &color);
		}
	}
	bool r = FreeImage_Save(fif, dib, filename.c_str(), 0) == 1;
	FreeImage_Unload(dib);
	return r;
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


FreeImageB::FreeImageB() : w(0), h(0), nChannels(0), data(nullptr)
{
}

FreeImageB::FreeImageB(unsigned int width, unsigned int height, unsigned int nChannels) :
	w(width), h(height), nChannels(nChannels), data(new BYTE[nChannels * width*height])
{
}

FreeImageB::FreeImageB(const FreeImage& img) :
	w(img.w), h(img.h), nChannels(img.nChannels), data(new BYTE[nChannels * img.w*img.h])
{
	memcpy(data, img.data, sizeof(BYTE) * nChannels * w*h);
}

FreeImageB::FreeImageB(const std::string& filename) : w(0), h(0), nChannels(0), data(nullptr)
{
	LoadImageFromFile(filename);
}

FreeImageB::~FreeImageB()
{
	if (data != nullptr) delete[] data;
}

void FreeImageB::operator=(const FreeImageB& other)
{
	if (other.data != this->data)
	{
		SetDimensions(other.w, other.h, other.nChannels);
		memcpy(data, other.data, sizeof(BYTE) * nChannels * w * h);
	}
}

void FreeImageB::SetDimensions(unsigned int width, unsigned int height, unsigned int nChannels)
{
	if (data != nullptr) delete[] data;
	w = width;
	h = height;
	this->nChannels = nChannels;
	data = new BYTE[nChannels * width * height];
}

bool FreeImageB::LoadImageFromFile(const std::string& filename, unsigned int width, unsigned int height)
{
	FreeImage_Initialise();
	if (data != nullptr) delete[] data;

	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	//pointer to the image, once loaded
	FIBITMAP *dib(0);

	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename.c_str(), 0);
	if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename(filename.c_str());
	if (fif == FIF_UNKNOWN) return false;

	//check that the plugin has reading capabilities and load the file
	if (FreeImage_FIFSupportsReading(fif)) dib = FreeImage_Load(fif, filename.c_str());
	if (!dib) return false;


	// Convert to RGBA float images
	{
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_ConvertToRGBAF(hOldImage); // ==> 4 channels
		FreeImage_Unload(hOldImage);
	}

	//get the image width and height
	w = FreeImage_GetWidth(dib);
	h = FreeImage_GetHeight(dib);

	// rescale to fit width and height
	if (width != 0 && height != 0)
	{
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_Rescale(hOldImage, width, height, FILTER_CATMULLROM);
		FreeImage_Unload(hOldImage);
		w = width;
		h = height;
	}

	//retrieve the image data
	float* bitsF = (float*)FreeImage_GetBits(dib);

	//if this somehow one of these failed (they shouldn't), return failure
	if ((bitsF == 0) || (w == 0) || (h == 0))
		return false;

	nChannels = 4;
	// copy image data
	data = new BYTE[nChannels * w * h];

	// flip
	for (int y = 0; y < (int)h; ++y)
	{
		for (int x = 0; x < (int)w; ++x)
		{
			for (int c = 0; c < (int)nChannels; ++c)
			{
				data[(y*w + x)*nChannels + c] = (unsigned char)(std::max(std::min(bitsF[((h - 1 - y)*w + x) * nChannels + c], 1.0f), 0.0f) * 255);
			}
		}
	}
	//memcpy(data, bits, sizeof(BYTE) * nChannels * w * h);

	//Free FreeImage's copy of the data
	FreeImage_Unload(dib);

	return true;
}

bool FreeImageB::SaveImageToFile(const std::string& filename, bool flipY)
{
	FREE_IMAGE_FORMAT fif = FIF_PNG;
	FIBITMAP *dib = FreeImage_Allocate(w, h, 24);
	RGBQUAD color;
	for (unsigned int j = 0; j < h; j++) {
		for (unsigned int i = 0; i < w; i++) {
			unsigned char col[3] = { 0, 0, 0 };

			for (unsigned int c = 0; c < nChannels && c < 3; ++c)
			{
				//col[c] = std::min(std::max(0, (int)(255.0f*data[nChannels * (w*j + i) + c])), 255);
				col[c] = data[nChannels * (w*j + i) + c];
			}

			color.rgbRed = col[0];
			color.rgbGreen = col[1];
			color.rgbBlue = col[2];
			if (!flipY)	FreeImage_SetPixelColor(dib, i, h - 1 - j, &color);
			else		FreeImage_SetPixelColor(dib, i, j, &color);
		}
	}
	bool r = FreeImage_Save(fif, dib, filename.c_str(), 0) == 1;
	FreeImage_Unload(dib);
	return r;
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


FreeImageU16F::FreeImageU16F() : w(0), h(0), nChannels(0), data(nullptr)
{
}

FreeImageU16F::FreeImageU16F(const std::string& filename) : w(0), h(0), nChannels(0), data(nullptr)
{
	LoadImageFromFile(filename);
}

FreeImageU16F::~FreeImageU16F()
{
	if (data != nullptr) delete[] data;
}

bool FreeImageU16F::LoadImageFromFile(const std::string& filename, unsigned int width, unsigned int height)
{
	FreeImage_Initialise();
	if (data != nullptr) delete[] data;

	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	//pointer to the image, once loaded
	FIBITMAP *dib(0);

	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename.c_str(), 0);
	if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename(filename.c_str());
	if (fif == FIF_UNKNOWN) return false;

	//check that the plugin has reading capabilities and load the file
	if (FreeImage_FIFSupportsReading(fif)) dib = FreeImage_Load(fif, filename.c_str());
	if (!dib) return false;


	// Convert to grey float images
	{
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_ConvertToFloat(hOldImage); // ==> 1 channel
		FreeImage_Unload(hOldImage);
	}

	//get the image width and height
	w = FreeImage_GetWidth(dib);
	h = FreeImage_GetHeight(dib);

	// rescale to fit width and height
	if (width != 0 && height != 0)
	{
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_Rescale(hOldImage, width, height, FILTER_CATMULLROM);
		FreeImage_Unload(hOldImage);
		w = width;
		h = height;
	}

	//retrieve the image data
	float* bitsF = (float*)FreeImage_GetBits(dib);

	//if this somehow one of these failed (they shouldn't), return failure
	if ((bitsF == 0) || (w == 0) || (h == 0))
		return false;

	nChannels = 1;
	// copy image data
	data = new float[nChannels * w * h];

	// flip
	for (int y = 0; y < (int)h; ++y)
	{
		for (int x = 0; x < (int)w; ++x)
		{
			for (int c = 0; c < (int)nChannels; ++c)
			{
				data[(y*w + x)*nChannels + c] = bitsF[((h - 1 - y)*w + x) * nChannels + c] * (256*256-1);
			}
		}
	}

	//Free FreeImage's copy of the data
	FreeImage_Unload(dib);

	return true;
}

