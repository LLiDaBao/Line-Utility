#include "lineutils.hpp"


/* @brief Calculate the slope of a line perpendicular to the given line .
* @param line: the given line with (v_x, v_y, x0, y0).
* @param perpendLine: the output perpendicular line with (v_x, v_y, x0, y0).
* @param pt: the point that the perpendicular pass through.
* @return return a line perpendicular to the input line and passsing through the input point.
*/
void lineutils::perpendLine(
	const cv::Vec4f&   line,
	cv::Vec4f&         perpendLine,
	const cv::Point2i& pt
)
{
	// (vy_0 / vx_0) * (vy_1 / vx_1) = -1

	float v_x = line[0], v_y = line[1];
	perpendLine[0] = -v_y;
	perpendLine[1] = v_x;
	perpendLine[2] = pt.x;
	perpendLine[3] = pt.y;
}


/* @brief Calculate the intersection point of 2 given lines.
* @param lhs: line1, it should be like (v_x, v_y, x0, y0).
* @param rhs: line2, it should be like (v_x, v_y, x0, y0).
* @return intersection point.
*/
cv::Point2f lineutils::intersecPoint(
	const cv::Vec4f& lhs,
	const cv::Vec4f& rhs
)
{
	if (lhs[0] == 0.0f || rhs[0] == 0.0f)	// slope equals inf
	{
		if(lhs[0] == 0.0f && rhs[0] == 0.0f)
		{
			std::cerr << "Both line are perpendicular to x-axis." << std::endl;
			return cv::Point2f();
		}
		
		cv::Vec4f notVerticalLine = (lhs[0] == 0.0f) ? rhs : lhs;
		cv::Vec4f verticalLine = (lhs[0] == 0.0f) ? lhs : rhs;

		float x = verticalLine[2];
		float y = retY(x, notVerticalLine);

		return cv::Point2f(x, y);
	}

	// y = line[1] / line[0] * (x - line[2]) + line[3]
	float A1 = -lhs[1] / (lhs[0]);
	float B1 = 1.0f;
	float C1 = -lhs[1] / (lhs[0]) * lhs[2] + lhs[3];

	float A2 = -rhs[1] / (rhs[0]);
	float B2 = 1.0f;
	float C2 = -rhs[1] / (rhs[0]) * rhs[2] + rhs[3];

	if (A1 * B1 == A2 * B2)
	{
		std::cerr << "Parallel." << std::endl;
		return cv::Point2f();
	}
	else if (A1 == A2 && B1 == B2 && C1 == C2)
	{
		std::cerr << "Same." << std::endl;
		return cv::Point2f();
	}

	float x = (B2 * C1 - B1 * C2) / (A1 * B2 - A2 * B1);
	float y = (A1 * C2 - A2 * C1) / (A1 * B2 - A2 * B1);

	return cv::Point2f(x, y);
}


/* @brief Offset the given line toward postive y-axis or negative y-axis direction.
* @param line: it should be like (v_x, v_y, x0, y0).
* @param len: offset length.
* @param towardPostiveY: true if toward y-axis positive direction, else false.
*/
void lineutils::shiftLine(
	cv::Vec4f&  line,
	double		len,
	bool		towardPositveY
)
{
	if (line[0] == 0.0f && line[1] == 0.0f)
	{
		std::cerr << "Error Line." << std::endl;
		return;
	}

	else if (line[0] == 0.0)
	{
		line[2] += len;
		return;
	}

	else if (line[1] == 0.0)
	{
		line[3] += len;
		return;
	}

	else
	{
		double phi = atan(line[1] / line[0]);	// return rad

		double offsetX = 0.0, offsetY = 0.0;
		if (towardPositveY)
		{
			offsetX = -sin(phi) * len;
			offsetY = cos(phi) * len;
		}
		else
		{
			offsetX = sin(phi) * len;
			offsetY = -cos(phi) * len;
		}

		line[2] += offsetX;
		line[3] += offsetY;
	}

	return;
}


/* @brief Count the number of non-zero pixels along the given line.
* @param line: the line for searching.
* @param start: starting row or column.
* @param end: ending row or column.
* @param direction: search column by column if HORIZONTAL, else row by row.
* @return cnt: the number of non-zero pixels that found along the line.
*/
int lineutils::countLineNonZero(
	const cv::Mat&      src,
	const cv::Vec4f&    line,
	int					start,
	int					end,
	DirectionType       direction
)
{
	if (start >= end)
	{
		std::cerr << "\nInput Parameters Got Error." << std::endl;
		std::cerr << "Check 'start' and 'end'." << std::endl;
		return -1;
	}

	int cnt = 0;	// the return value.

	/* Searching Column by Column, the Line Should be Near Horizontal. */
	if (direction == DirectionType::HORIZONTAL)
	{
		if (start < 0 || end > src.cols)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.cols ." << std::endl;
			return -1;
		}

		for (int col = start; col != end; ++col)
		{
			float row = retY(col, line);

			if (inMatrix(src, row, col) && src.at<uchar>(row, col) != 0)
				++cnt;
		}
	}

	/* Searching Row by Row, the Line Should be Near Vertical. */
	else
	{
		if (start < 0 || end > src.rows)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.rows ." << std::endl;
			return -1;
		}

		for (int row = start; row != end; ++row)
		{
			float col = retX(row, line);
			if (inMatrix(src, row, col) && src.at<uchar>(row, col) != 0)
				++cnt;
		}
	}

	return cnt;
}


/* @brief The sum of grayscale value from start to end along the given line.
* @param src: 8-bit single channel image.
* @param line: input line, it should be (v_x, v_y, x0, y0).
* @param start: starting x or y, it depends on param: [direction].
* @param end: ending x or y, it depends on param: [direction].
* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
* @return sum of grayscale along the given line.
*/
float lineutils::lineTotalGrayVal(
	const cv::Mat&      src,
	const cv::Vec4f&    line,
	int					start,
	int					end,
	DirectionType       direction
)
{
	if (start >= end)
	{
		std::cerr << "\nInput Parameters Got Error." << std::endl;
		std::cerr << "Check 'start' and 'end'." << std::endl;
		return -1;
	}

	float sum = 0.0f;	// the return value.

	/* Searching Column by Column, the Line Should be Near Horizontal. */
	if (direction == DirectionType::HORIZONTAL)
	{
		if (start < 0 || end > src.cols)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.cols ." << std::endl;
			return -1;
		}

		for (int col = start; col != end; ++col)
		{
			float row = retY(col, line);

			if (inMatrix(src, row, col))
				sum += 1.0f * src.at<uchar>(row, col);
		}
	}

	/* Searching Row by Row, the Line Should be Near Vertical. */
	else
	{
		if (start < 0 || end > src.rows)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.rows ." << std::endl;
			return -1;
		}

		for (int row = start; row != end; ++row)
		{
			float col = retX(row, line);
			if (inMatrix(src, row, col))
				sum += 1.0f * src.at<uchar>(row, col);
		}
	}

	return sum;
}


/* @brief The sum of grayscale value from start to end along the given line.
* @param src: 8-bit single channel image.
* @param line: input line, it should be (v_x, v_y, x0, y0).
* @param grayVals: output array, each element is a std::pair p. Use p.first to get position cv::Point2f, p.second to get grayscale value.
* @param start: starting x or y, it depends on param: [direction].
* @param end: ending x or y, it depends on param: [direction].
* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
* @return sum of grayscale along the given line.
*/
double lineutils::lineGrayVal(
	const cv::Mat&      src,
	const cv::Vec4f&    line,
	std::vector<std::pair<cv::Point2f, double>>&
	                    grayVals,
	int					start,
	int					end,
	DirectionType       direction
)
{
	// pre-process vector
	grayVals.clear();

	if (start >= end)
	{
		std::cerr << "\nInput Parameters Got Error." << std::endl;
		std::cerr << "Check 'start' and 'end'." << std::endl;
		return 0.0;
	}

	/* Searching Column by Column, the Line Should be Near Horizontal. */
	if (direction == DirectionType::HORIZONTAL)
	{
		if (start < 0 || end > src.cols)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.cols ." << std::endl;
			return 0.0;
		}

		for (int col = start; col != end; ++col)
		{
			float row = retY(col, line);
			cv::Point2f temp(col, row);

			if (inMatrix(src, row, col))
			{
				double grayval = (double)src.at<uchar>(row, col);
				grayVals.emplace_back(std::make_pair(temp, grayval));
			}
		}
	}

	/* Searching Row by Row, the Line Should be Near Vertical. */
	else
	{
		if (start < 0 || end > src.rows)
		{
			std::cerr << "\nInput Parameters Got Error." << std::endl;
			std::cerr << "Check 'start' and 'end' if is out of 0 or src.rows ." << std::endl;
			return 0.0;
		}

		for (int row = start; row != end; ++row)
		{
			float col = retX(row, line);
			cv::Point2f temp(col, row);

			if (inMatrix(src, row, col))
			{
				double grayval = (double)src.at<uchar>(row, col);
				grayVals.emplace_back(std::make_pair(temp, grayval));
			}
		}
	}

	double sum = 0.0;

	for (const auto& pair : grayVals)
		sum += pair.second;
	
	return sum;
}


/* @brief Get bounding rectangle of given line.
* @param src: source image.
* @param line: input line.
* @param rect: output bounding rectangle.
* @param start: starting x or y, it depends on param: [direction].
* @param end: ending x or y, it depends on param: [direction].
* @param direction: if HORIZONTAL, start and end should be x-axis values, else y-axis values.
* @return flag if the bounding rectangle is out of range.
*/
bool lineutils::lineBoundingRect(
	const cv::Mat&      src,
	const cv::Vec4f&    line,
	cv::Rect&           rect,
	int					start,
	int					end,
	DirectionType		direction
)
{
	if (start >= end)
	{
		std::cerr << "lineBoundingRect: error "
			"input parameters[start, end]." << std::endl;
		return false;
	}

	int leftmost = -1,	// the leftmost value of RoI.
		topmost = -1,	// the top-most value of RoI.
		rightmost = -1,	// the right-most value of RoI.
		botmost = -1;	// the bottom-most value of RoI.

	if (direction == DirectionType::HORIZONTAL)
	{
		leftmost = start;
		rightmost = end;	// the leftest and rightest values of RoI.

		int _y1 = cvCeil(retY(leftmost, line));
		int _y2 = cvFloor(retY(rightmost, line));

		topmost = std::min(_y1, _y2);
		botmost = std::max(_y1, _y2);
	}

	else
	{
		topmost = start;
		botmost = end;
		int _x1 = cvCeil(retX(topmost, line));
		int _x2 = cvFloor(retX(botmost, line));	// the leftest and rightest values of RoI.

		leftmost = std::min(_x1, _x2);
		rightmost = std::max(_x1, _x2);
	}

	if (inMatrix(src, topmost, leftmost) && inMatrix(src, topmost, rightmost)
		&& inMatrix(src, botmost, leftmost) && inMatrix(src, botmost, rightmost))
	{
		rect.x = leftmost;
		rect.y = topmost;
		rect.width = rightmost - leftmost;
		rect.height = botmost - topmost;
	}

	else
	{
		std::cerr << "RoI is out of range." << std::endl;
		return false;
	}

	return true;
}


/* @brief Distance between 2 given line
* @param baseLine:
* @param targetLine:
* @param start:
* @param end:
* @param direction:
* @return
*/
double lineutils::meanDistBetweenLines(
	const cv::Vec4f& baseLine,
	const cv::Vec4f& targetLine,
	int              start,
	int              end,
	DirectionType    direction
)
{
	// **** Attention **** 此处暂时使用点到直线的距离(即最短距离),还可以采用计算垂线和交点，然后进一步计算两点距离

	// Calculate the distance of a point to a line
	auto distPoint2Line = [](const cv::Point2f& pt, const cv::Vec4f& line) -> double
	{
		// if y = k·x+b, point is (x0, y0)
		// then, dist = std::abs(k * x0 - y0 + b) / std::sqrt(1 + k^2)

		double eps = 1e-6;	// 数值稳定
		double k = line[1] / (line[0] + eps);
		double b = line[3] - k * line[2];

		return std::abs(k * pt.x - pt.y + b) / std::sqrt(k * k + 1);
	};

	double mean = 0.0;
	int numSamples = end - start;

	assert(numSamples > 0);	// end should be greater than start.

	for (int i = start; i != end; ++i)
	{
		cv::Point2f temp;
		switch (direction)
		{
		case lineutils::DirectionType::VERTICAL:
			temp.x = retX(i, baseLine);
			temp.y = i;
			break;
		case lineutils::DirectionType::HORIZONTAL:
			temp.x = i;
			temp.y = retY(i, baseLine);
			break;
		default:
			break;
		}
		
		mean += distPoint2Line(temp, targetLine);
	}

	return mean / numSamples;
}
