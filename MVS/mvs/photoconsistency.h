#pragma once

#include <opencv2\opencv.hpp>

#include "basic.h"
#include "View.h"

namespace mvs {
	inline cv::Vec3d bilinearSample(cv::Mat* source, cv::Point2f pos) {
		int x0 = (int)pos.x;
		int x1 = (int)pos.x + 1;
		int y0 = (int)pos.y;
		int y1 = (int)pos.y + 1;

		double x = pos.x - (double)x0;
		double y = pos.y - (double)y0;

		cv::Vec3b p00 = source->at<cv::Vec3b>(y0, x0);
		cv::Vec3b p01 = source->at<cv::Vec3b>(y0, x1);
		cv::Vec3b p10 = source->at<cv::Vec3b>(y1, x0);
		cv::Vec3b p11 = source->at<cv::Vec3b>(y1, x1);

		cv::Vec3b result = p00 * (1 - x) * (1 - y) + p01 * x * (1 - y) + p10 * (1 - x) * y + p11 * x * y;

		return result;
	}


	template <int N> inline double ncc(cv::Vec3b* f, cv::Vec3b* t) {
		double result = 0;

		for (int c = 0; c < 3; c++) {
			double f_mean = 0;
			double f_sdev = 0;
			double t_mean = 0;
			double t_sdev = 0;
			double score = 0;

			for (int i = 0; i < N * N; i++) {
				f_mean += f[i][c];
				t_mean += t[i][c];
			}

			f_mean /= N * N;
			t_mean /= N * N;

			for (int i = 0; i < N * N; i++) {
				f_sdev += (f[i][c] - f_mean) * (f[i][c] - f_mean);
				t_sdev += (t[i][c] - f_mean) * (t[i][c] - f_mean);
			}

			f_sdev /= (N * N);
			t_sdev /= (N * N);

			for (int i = 0; i < N * N; i++) {
				score += (f[i][c] - f_mean) * (t[i][c] - t_mean);
			}
			score /= sqrt(f_sdev * t_sdev) * N * N;
			result += score;
		}

		return result / 3;
	}


	template <int N> inline double ncc(Patch* p, View* i, View* j) {
		cv::Vec3b f[N * N];
		cv::Vec3b t[N * N];

		for (int k = 0; k < N * N; k++) {
			cv::Point3d pos = p->position + cv::Point3d(p->tangent_1 * (k / N - N / 2) + p->tangent_2 * (k % N - N / 2));

			cv::Point2d projection_i = mvs::projectPoint(i, pos);
			if ((int)projection_i.x < 0 || (int)projection_i.x >= i->img.cols - 2 || (int)projection_i.y < 0 || (int)projection_i.y >= i->img.rows - 2)
				return -1;

			cv::Point2d projection_j = mvs::projectPoint(j, pos);
			if ((int)projection_j.x < 0 || (int)projection_j.x >= j->img.cols - 2 || (int)projection_j.y < 0 || (int)projection_j.y >= j->img.rows - 2)
				return -1;

			f[k] = bilinearSample(&(i->img), projection_i);
			t[k] = bilinearSample(&(j->img), projection_j);
		}

		return ncc<N>(f, t);
	}


	template <int N> inline double ncc(cv::Point3d* p, View* i, View* j) {
		cv::Vec3b f[N * N];
		cv::Vec3b t[N * N];

		for (int k = 0; k < N * N; k++) {
			cv::Point2d projection_i = mvs::projectPoint(i, p[k]);
			if ((int)projection_i.x < 0 || (int)projection_i.x >= i->img.cols - 2 || (int)projection_i.y < 0 || (int)projection_i.y >= i->img.rows - 2)
				return -1;

			cv::Point2d projection_j = mvs::projectPoint(j, p[k]);
			if ((int)projection_j.x < 0 || (int)projection_j.x >= j->img.cols - 2 || (int)projection_j.y < 0 || (int)projection_j.y >= j->img.rows - 2)
				return -1;

			f[k] = bilinearSample(&(i->img), projection_i);
			t[k] = bilinearSample(&(j->img), projection_j);
		}

		return ncc<N>(f, t);
	}
}