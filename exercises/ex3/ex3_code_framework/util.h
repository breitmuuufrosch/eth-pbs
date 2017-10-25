//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#pragma once

#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Utilities/Array2T.h"
#include "LodePNG/lodepng.h"

//-----------------------------------------------------------------------------
// timing functions
#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/time.h>
#endif

static long int getTime()
{
	long int ret = 0;
#if defined(_WIN32)
	LARGE_INTEGER liTimerFrequency;
	QueryPerformanceFrequency(&liTimerFrequency);
	LARGE_INTEGER liLastTime;
	QueryPerformanceCounter(&liLastTime);
	ret = (INT)(((double)liLastTime.QuadPart / liTimerFrequency.QuadPart) * 1000);
#else
	struct timeval tv;
	struct timezone tz;
	tz.tz_minuteswest = 0;
	tz.tz_dsttime = 0;
	gettimeofday(&tv, &tz);
	ret = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
#endif
	return ret;
}

static std::string timeToString(long int usecs)
{
	int ss = (int)(((double)usecs / 1000.0));
	int ps = (int)(((double)usecs - (double)ss*1000.0) / 10.0);
	std::ostringstream out;
	out << ss << ".";
	if (ps < 10)
		out << "0";
	out << ps << "s";
	return out.str();
}

//-----------------------------------------------------------------------------
// write png image
static void writePNG(const char *fileName, unsigned char **rowsp, int w, int h, bool normalize)
{
	std::vector<unsigned char> image;
	for (int i = 0; i < w; i++)
		for (int j = 0; j < h * 4; j++)
			image.push_back(rowsp[i][j]);

	lodepng::encode(fileName, image, w, h, LCT_RGBA);
}

static void dumpNumberedImage(int counter, std::string prefix, const Array2d &field)
{
	char buffer[256];
	sprintf(buffer, "%04i", counter);
	std::string number = std::string(buffer);

	int xRes = field.size(0);
	int yRes = field.size(1);

	unsigned char *pngbuf = new unsigned char[xRes*yRes*4];
	unsigned char **rows = new unsigned char*[yRes];
	for (int j = 0; j < yRes; j++)
	{
		for (int i = 0; i < xRes; i++)
		{
			double val = field(i, j);
			if (val > 1.)
				val = 1.;
			else if (val < 0.)
				val = 0.;
			pngbuf[(j*xRes + i) * 4 + 0] = (unsigned char)(val*255.);
			pngbuf[(j*xRes + i) * 4 + 1] = (unsigned char)(val*255.);
			pngbuf[(j*xRes + i) * 4 + 2] = (unsigned char)(val*255.);
			pngbuf[(j*xRes + i) * 4 + 3] = 255;
		}
		rows[j] = &pngbuf[(yRes - j - 1)*xRes * 4];
	}
	std::string filenamePNG = prefix + number + std::string(".png");
	writePNG(filenamePNG.c_str(), rows, xRes, yRes, false);
	printf("Writing %s\n", filenamePNG.c_str());

	delete[] pngbuf;
	delete[] rows;
}
