#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>


namespace lineutils
{
	enum class FittingType
	{
		LSM,
		RSANC
	};

	enum class DirectionType
	{
		VERTICAL,
		HORIZONTAL
	};

	enum class InterpolateType
	{
		LAGRANGE,
		NEWTON
	};

	/* @brief Line type conversion. */
	inline
	cv::Vec4d convert2d(const cv::Vec4f& line)
	{
		return cv::Vec4d(line[0], line[1], line[2], line[3]);
	}


	/* @brief Line type conversion. */
	inline
	cv::Vec4f convert2f(const cv::Vec4d& line)
	{
		return cv::Vec4f(line[0], line[1], line[2], line[3]);
	}


	/* @brief Check row and column if is in cv::Mat. */
	inline
	bool inMatrix(
		const cv::Mat& src,
		double		   row,
		double         col
	)
	{
		return row >= 0.0 && row < 1.0 * src.rows
			&& col >= 0.0 && col < 1.0 * src.cols;
	}


	/* @brief Input the y-coordinate value and a line, return the x-coordinate value.
	* @param _y: input y-coordinate value.
	* @param line: a line with (v_x, v_y, x0, y0).
	* @return _x: output x-coordinate value. */
	inline
	static float retX(float _y, const cv::Vec4f& line)
	{
		return (line[0]) / line[1] * (_y - line[3]) + line[2];
	}


	/* @brief Input the x-coordinate value and a line, return the y-coordinate value.
	* @param _x: input x-coordinate value.
	* @param line: a line with (v_x, v_y, x0, y0).
	* @return _y: output y-coordinate value. */
	inline
	static float retY(float _x, const cv::Vec4f& line)
	{
		return line[1] / line[0] * (_x - line[2]) + line[3];
	}


	/* @brief Calculate the slope of a line perpendicular to the given line .
	* @param line: the given line with (v_x, v_y, x0, y0).
	* @param perpendLine: the output perpendicular line with (v_x, v_y, x0, y0).
	* @param pt: the point that the perpendicular pass through.
	* @return return a line perpendicular to the input line and passsing through the input point. */
	extern
	void perpendLine(
		const cv::Vec4f&   line,
		cv::Vec4f&         perpendLine,
		const cv::Point2i& pt
	);


	/* @brief Calculate the intersection point of 2 given lines.
	* @param lhs: line1, it should be like (v_x, v_y, x0, y0).
	* @param rhs: line2, it should be like (v_x, v_y, x0, y0).
	* @return intersection point. */
	extern
	cv::Point2f intersecPoint(
		const cv::Vec4f& lhs,
		const cv::Vec4f& rhs
	);


	/* @brief Offset the given line toward postive y-axis or negative y-axis direction.
	* @param line: it should be like (v_x, v_y, x0, y0).
	* @param len: offset length.
	* @param towardPostiveY: true if toward y-axis positive direction, else false. */
	void shiftLine(
		cv::Vec4f&  line,
		double		len,
		bool		towardPositveY
	);


	/* @brief Count the number of non-zero pixels along the given line.
	* @param line: the line for searching.
	* @param start: starting row or column.
	* @param end: ending row or column.
	* @param direction: search column by column if HORIZONTAL, else row by row.
	* @return cnt: the number of non-zero pixels that found along the line. */
	extern
	int countLineNonZero(
		const cv::Mat&      src,
		const cv::Vec4f&    line,
		int					        start,
		int					        end,
		DirectionType       direction = DirectionType::HORIZONTAL
	);


	/* @brief The sum of grayscale value from start to end along the given line.
	* @param src: 8-bit single channel image.
	* @param line: input line, it should be (v_x, v_y, x0, y0).
	* @param start: starting x or y, it depends on param: [direction].
	* @param end: ending x or y, it depends on param: [direction].
	* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
	* @return sum of grayscale along the given line. */
	extern
	float lineTotalGrayVal(
		const cv::Mat&      src,
		const cv::Vec4f&    line,
		int					        start,
		int					        end,
		DirectionType       direction = DirectionType::HORIZONTAL
	);


	/* @brief The sum of grayscale value from start to end along the given line.
	* @param src: 8-bit single channel image.
	* @param line: input line, it should be (v_x, v_y, x0, y0).
	* @param grayVals: output array, each element is a std::pair p. Use p.first to get position cv::Point2f, p.second to get grayscale value.
	* @param start: starting x or y, it depends on param: [direction].
	* @param end: ending x or y, it depends on param: [direction].
	* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
	* @return sum of grayscale along the given line. */
	double lineGrayVal(
		const cv::Mat&      src,
		const cv::Vec4f&    line,
		std::vector<std::pair<cv::Point2f, double>>& 
		                    grayVals,
		int					        start,
		int					        end,
		DirectionType       direction = DirectionType::HORIZONTAL
	);


	/* @brief Get bounding rectangle of given line.
	* @param src: source image.
	* @param line: input line.
	* @param rect: output bounding rectangle.
	* @param start: starting x or y, it depends on param: [direction].
	* @param end: ending x or y, it depends on param: [direction].
	* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
	* @return flag if the bounding rectangle is out of range. */
	extern
	bool lineBoundingRect(
		const cv::Mat&      src,
		const cv::Vec4f&    line,
		cv::Rect&           rect,
		int					        start,
		int					        end,
		DirectionType		    direction = DirectionType::HORIZONTAL
	);


	/* @brief Mean distance from base line to target line.
	* @param baseLine: base line.
	* @param targetLine: target line.
	* @param start: starting x or y, it depends on param: [direction].
	* @param end: ending x or y, it depends on param: [direction].
	* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
	* @return mean distance value. */
	extern 
	double meanDistBetweenLines(
		const cv::Vec4f& baseLine,
		const cv::Vec4f& targetLine,
		int              start,
		int              end,
		DirectionType    direction
	);


	/* @brief Distance between 2 given points. */
	template <typename T = int>
	double distPoint2Point(
		const cv::Point_<T>& lhs,
		const cv::Point_<T>& rhs
	)
	{
		return std::sqrt(
			(rhs.x - lhs.x) * (rhs.x - lhs.x) +
			(rhs.y - lhs.y) * (rhs.y - lhs.y)
		);
	}


	/* @brief Calculate the output value.
	* @param _x: input x value.
	* @param coeffs: coeffcients of polyfitting curve.
	* @return output y value. */
	template <typename T = double>
	double retPolyfitY(double _x, const cv::Mat& coeffs)
	{
		// check coeffs
		assert(coeffs.rows > 0 && coeffs.cols == 1);

		int degrees = coeffs.rows - 1;	// polyfit curve degree
		double _y = 0.0;

		for (int k = 0; k <= degrees; ++k)
		{
			double coeff = coeffs.at<T>(k, 0);
			_y += coeff * std::pow(_x, (double)k);
		}

		return _y;
	}


	/* @brief Polynominal Fitting. T1 is the type of input points, T2 is the type of OutputArray.
	* @param points: input points for fitting.
	* @param coeffs: output coefficents array.
	* @param degres: the degree. */
	template <typename T1 = float, typename T2 = double>
	void polynominalFitting(
		std::vector<cv::Point_<T1>>& points,
		cv::Mat&                     coeffs,
		int                          degree
	)
	{
		size_t numPoints = points.size();

		// Constrcut X Matrix， its shape is (degree + 1, degree + 1).
		cv::Mat X = cv::Mat_<T2>::zeros(degree + 1, degree + 1);

		for (int i = 0; i <= degree; ++i)
		{
			for (int j = 0; j <= degree; ++j)
			{
				for (int n = 0; n < numPoints; ++n)
				{
					X.at<T2>(i, j) +=
						std::pow(points[n].x, (T2)(i + j));
				}
			}
		}

		// Constrcut Y Matrix， its shape is (degree + 1, 1).
		cv::Mat Y = cv::Mat_<T2>::zeros(degree + 1, 1);

		for (int i = 0; i <= degree; ++i)
		{
			for (int n = 0; n < numPoints; ++n)
			{
				Y.at<double>(i, 0) += (
					std::pow(points[n].x, (T2)i) * points[n].y);
			}
		}

		coeffs = cv::Mat_<T2>::zeros(degree + 1, 1);

		cv::solve(X, Y, coeffs, cv::DECOMP_LU);

		return;
	}


	/* @brief Lagrange interpolate.
	* @param inputx: input array.
	* @param inputy: input array.
	* @param x: input value for interpolation.
	* @return  output value after interpolating. */
	template <typename T = double>
	double lagrangeInterpolate(
		const std::vector<T>& inputx,
		const std::vector<T>& inputy,
		T                     x
	)
	{
		if (inputx.size() != inputy.size() || inputx.empty() || inputy.empty())
		{
			std::cerr << "Error inputs.\n" << std::endl;
			return -1.0;
		}

		double result = 0.0;
		size_t size = inputx.size();

		for (size_t k = 0; k != size; ++k)
		{
			double temp = 1.0;

			for (size_t i = 0; i != size; ++i)
			{
				if (i != k)
				{
					temp *= ((1.0 * x - inputx[i]) / (1.0 * inputx[k] - inputx[i]));
				}
			}

			result += (temp * inputy[k]);
		}

		return result;
	}

}	// namespace
