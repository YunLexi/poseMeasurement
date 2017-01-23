// poseMeasurement.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "measure.h"
#include <fstream>
#include <string>
#include <opencv.hpp>
int _tmain(int argc, _TCHAR* argv[])
{
	std::ifstream img_file;
	std::string img_list = "init-30.txt";//outer_eclip
	img_file.open(img_list.c_str());
	std::string line;
	
	while (img_file >> line)
	{
		printf("Image name is:%s\n", line.c_str());
		Measure measure;
		//img_file >> line;
		//img_file >> line;
		//img_file >> line;
		cv::Mat img;
		IplImage* img_color = cvLoadImage(line.c_str(), 1);
		img = cv::imread(line,0);
		//img_color = cv::imread(line, 0);
		cv::imshow(line, img);
		cv::waitKey();
		measure.get_eclip_points(img, PRED_OUTER);
		measure.get_p_q();
		measure.debug_p_q(img_color);
		measure.cal_w_v_u();
		if (measure.get_b_c() == false)
		{
			printf("press any key to continue!\n");
			std::getchar();
			continue;
		}
		measure.debug_b_c(img_color);
		measure.get_target_pose();
		measure.convert_pose_cor();
		cv::waitKey();
		cv::destroyAllWindows();

	}
	return 0;
}

