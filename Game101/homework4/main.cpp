#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#define STEP 0.0005
#define WIDTH 700
#define HEIGHT 700

#define AA

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window) {
	auto& p_0 = points[0];
	auto& p_1 = points[1];
	auto& p_2 = points[2];
	auto& p_3 = points[3];

	for (double t = 0.0; t <= 1.0; t += STEP) {
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
			3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, double t) {
	// TODO: Implement de Casteljau's algorithm

	int count = control_points.size();
	if (count == 1) {
		return control_points[0];
	}
	std::deque<cv::Point2f> q;
	std::for_each(control_points.begin(), control_points.end(), [&q](cv::Point2f point) { q.push_back(point); });

	while (true) {
		if (count == 1) {
			return q.front();
		}
		cv::Point2f p1 = q.front();
		q.pop_front();
		cv::Point2f p2 = q.front();
		// 每次多pop一个点
		for (int i = count-- - 1; i > 0; i--) {
			cv::Point2f p = p1 + (1 - t) * (p2 - p1);
			q.push_back(p);
			q.pop_front();
			p1 = p2, p2 = q.front();
		}
	}
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window) {
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.

	const float dism = 3;
	const int aa_range = 3;
	for (double t = 0.0; t <= 1.0; t += STEP) {
		cv::Point2f point = recursive_bezier(control_points, t);
#ifdef AA
		int x = point.x, y = point.y;
		float xf = point.x, yf = point.y;
		for (int i = -aa_range; i <= aa_range; i++)
			for (int j = -aa_range; j <= aa_range; j++) {
				float px = x + i + 0.5, py = y + j + 0.5;
				if (px < 0 || px >= 700 || py < 0 || py >= 700)
					continue;
				float dis = sqrt((px - xf) * (px - xf) + (py - yf) * (py - yf));
				dis = dis > dism ? dism : dis;
				window.at<cv::Vec3b>(point.y + j, point.x + i)[1] = std::max(
					(uchar)((1 - dis / dism) * 255),
					window.at<cv::Vec3b>(point.y + j, point.x + i)[1]);
			}
#else
		window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
#endif // !AA

	}
}

int main() {
	cv::Mat window = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while (key != 27) {
		for (auto& point : control_points) {
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == 4) {
			// naive_bezier(control_points, window);
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(0);

			return 0;
		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
