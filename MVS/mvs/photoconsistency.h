#pragma once

#include "basic.h"
#include <opencv2\opencv.hpp>

namespace mvs {
	template <int N> inline float ncc(cv::Vec3b* f, cv::Vec3b* t) {
		float f_mean = 0;
		float f_sdev = 0;
		float t_mean = 0;
		float t_sdev = 0;
		float result = 0;

		for (int i = 0; i < N * N; i++) {
			f_mean += f[i][2];
			t_mean += t[i][2];
		}

		f_mean /= N * N;
		t_mean /= N * N;

		for (int i = 0; i < N * N; i++) {
			f_sdev += (f[i][2] - f_mean) * (f[i][2] - f_mean);
			t_sdev += (t[i][2] - f_mean) * (t[i][2] - f_mean);
		}

		f_sdev /= (N * N);
		t_sdev /= (N * N);

		for (int i = 0; i < N * N; i++) {
			result += (f[i][2] - f_mean) * (t[i][2] - t_mean);
		}
		result /= sqrt(f_sdev * t_sdev) * N * N;

		return result;
	}


	cv::Vec3d bilinearSample(cv::Mat* source, cv::Point2f pos) {
		int x0 = (int)pos.x;
		int x1 = (int)pos.x + 1;
		int y0 = (int)pos.y;
		int y1 = (int)pos.y + 1;

		float x = pos.x - (float)x0;
		float y = pos.y - (float)y0;

		cv::Vec3b p00 = source->at<cv::Vec3b>(y0, x0);
		cv::Vec3b p01 = source->at<cv::Vec3b>(y0, x1);
		cv::Vec3b p10 = source->at<cv::Vec3b>(y1, x0);
		cv::Vec3b p11 = source->at<cv::Vec3b>(y1, x1);

		cv::Vec3b result = p00 * (1 - x) * (1 - y) + p01 * x * (1 - y) + p10 * (1 - x) * y + p11 * x * y;

		return result;
	}
}