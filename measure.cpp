#include "stdafx.h"
#include "measure.h"
#include <fstream>
#include <iostream>
Measure::Measure()
{
}

Measure::~Measure()
{
}
bool Measure::get_eclip_points(cv::Mat &img, EclipType eclip_type)
{
	cv::imshow("show", img);
	cv::waitKey();
	const int channels = img.channels();
	int nRows = img.rows;
	int nCols = img.cols;
	cv::Point point;
	for (int i = 0; i < nRows; ++i)
	{
		const uchar* imgData = img.ptr<uchar>(i);
		for (int j = 0; j < nCols; ++j)
		{
			if (imgData[j] > 50)
			{
				point.x = j;
				point.y = i;
				switch (eclip_type)
				{
				case PRED_INNER:
				{
					m_inner_points.push_back(point);
					break;
				}
				case PRED_OUTER:
				{
					m_outer_points.push_back(point);
					break;
				}
				case GT_INNER:
				{
					printf("Not implement yet!\n");
					break;
				}
				case GT_OUTER:
				{
					printf("Not implement yet!\n");
					break;;
				}
				default:
					break;
				}
			}
		}
	}
	switch (eclip_type)
	{
	case PRED_INNER:
	{
		for (int i = 0; i < m_inner_points.size(); ++i)
		{
			m_inner_points[i].x -= 163;//via intrinsic param of camera:u0
			m_inner_points[i].y -= 118;//via intrinsic param of camera:v0
			m_inner_points[i].x *= 4.0;
			m_inner_points[i].y *= 4.0;
		}
		break;
	}
	case PRED_OUTER:
	{
		//for (int i = 0; i < m_outer_points.size(); ++i)
			//printf("%d, %d\n", m_outer_points[i].x, m_outer_points[i].y);
		printf("col: %d, row: %d\n", img.cols, img.rows);
		for (int i = 0; i < m_outer_points.size(); ++i)
		{
			m_outer_points[i].x -= 163;
			m_outer_points[i].y -= 118;
			m_outer_points[i].x *= 4.0;
			m_outer_points[i].y *= 4.0;
		}
		break;
	}
	default:
		break;
	}
	return true;
}

float Measure::cal_angle(const float d1, const float d2, const float d3)
{
	float cos_val = (d1*d1 + d2*d2 - d3*d3) / (2 * d1*d2);
	return acos(cos_val);
}

float Measure::cal_dis(const cv::Point3d p1, const cv::Point3d p2)
{
	return std::sqrt((p1.x - p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}
bool Measure::get_p_q()
{
	cv::Point3d p_temp, q_temp;
	cv::Point3d G(0, 0, 0);
	float max_angle = 0;
	for (int i = 0; i < m_outer_points.size()-1; ++i)
	{
		p_temp.x = m_outer_points[i].x;
		p_temp.y = m_outer_points[i].y;
		p_temp.z = m_camera_focus_pix;
		for (int j = i + 1; j < m_outer_points.size(); ++j)
		{
			q_temp.x = m_outer_points[j].x;
			q_temp.y = m_outer_points[j].y;
			q_temp.z = m_camera_focus_pix;
			float d1 = cal_dis(p_temp, G);
			float d2 = cal_dis(q_temp, G);
			float d3 = cal_dis(p_temp, q_temp);
			float angle = cal_angle(d1, d2, d3);
			if (max_angle < angle)
			{
				max_angle = angle;
				m_p.x = p_temp.x;
				m_p.y = p_temp.y;
				m_q.x = q_temp.x;
				m_q.y = q_temp.y;
			}
		}
	}
	m_angle_pGq = max_angle;
	return true;
}
bool Measure::debug_p_q(IplImage* img)
{
	cv::Point p_org, q_org;
	p_org.x = m_p.x / 4.0 + 163;
	p_org.y = m_p.y / 4.0 + 118;
	q_org.x = m_q.x / 4.0 + 163;
	q_org.y = m_q.y / 4.0 + 118;
	cvLine(img, p_org, q_org, cv::Scalar(255,255,255), 2, 16, 0);
	cvShowImage("debug_p_q", img);
	cv::waitKey();
	return true;
}
float Measure::cal_norm(const Eigen::Vector3d v)
{
	return sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
}
bool Measure::cal_w_v_u()
{
	Eigen::Vector3d u_Gp(m_p.x, m_p.y, m_camera_focus_pix);
	Eigen::Vector3d u_Gq(m_q.x, m_q.y, m_camera_focus_pix);
	float l2_norm_Gp = cal_norm(u_Gp);
	float l2_norm_Gq = cal_norm(u_Gq);
	u_Gp(0) /= l2_norm_Gp; u_Gp(1) /= l2_norm_Gp; u_Gp(2) /= l2_norm_Gp;
	u_Gq(0) /= l2_norm_Gq; u_Gq(1) /= l2_norm_Gq; u_Gq(2) /= l2_norm_Gq;
	auto GM = u_Gp + u_Gq;
	float l2_norm_GM = cal_norm(GM);
	m_w << GM(0) / l2_norm_GM, GM(1) / l2_norm_GM, GM(2) / l2_norm_GM;
	m_v = u_Gp.cross(u_Gq);//cross 的顺序很关键！
	float l2_norm_v = cal_norm(m_v);
	m_v(0) /= l2_norm_v; m_v(1) /= l2_norm_v; m_v(2) /= l2_norm_v;
	m_u = m_w.cross(m_v);//cross的顺序很关键！
	return true;
}
bool Measure::get_b_c()
{
	float eps = 2;
	const float min_dis = 10;
	std::vector<float> detas;
	for (int i = 0; i < m_outer_points.size(); ++i)
	{
		float deta = abs(m_outer_points[i].x * m_u(0) + m_outer_points[i].y * m_u(1) + m_camera_focus_pix * m_u(2));
		if ( deta < eps)
		{
			b_c_temp.push_back(m_outer_points[i]);
			detas.push_back(deta);
		}
	}
	if (b_c_temp.size() == 0)
	{
		printf("Size of b_c is 0\n");
		return false;
	}
	float deta_b = detas[0];
	int b_index = 0;
	int index_temp =0;
	float deta_c = 0;
	int c_index = 0;

	cv::Point3d p1, p2;
	for (int i = 1; i < b_c_temp.size(); ++i)//this loop to get m_b
	{
		p1.x = b_c_temp[i - 1].x;
		p1.y = b_c_temp[i - 1].y;
		p1.z = m_camera_focus_pix;
		p2.x = b_c_temp[i].x;
		p2.y = b_c_temp[i].y;
		p2.z = m_camera_focus_pix;
		if (cal_dis(p1, p2) > min_dis)
		{
			index_temp = i;
			break;
		}
		if (detas[i] < deta_b)
		{
			b_index = i;
			deta_b = detas[i];
		}
	}
	c_index = index_temp;
	deta_c = detas[c_index];
	for (int i = index_temp + 1; i < b_c_temp.size(); ++i)//to get m_c
	{
		if (detas[c_index] < deta_c)
		{
			c_index = i;
			deta_c = detas[c_index];
		}
	}
	m_b = b_c_temp[b_index];
	m_c = b_c_temp[c_index];
	p1.x = m_b.x; p1.y = m_b.y; p1.z = m_camera_focus_pix;
	p2.x = m_c.x; p2.y = m_c.y; p2.z = m_camera_focus_pix;
	cv::Point3d G(0, 0, 0);
	float d1 = cal_dis(p1, G);
	float d2 = cal_dis(p2, G);
	float d3 = cal_dis(p1, p2);
	m_angle_bGc = cal_angle(d1, d2, d3);
	return true;
}
bool Measure::debug_b_c(IplImage* img)
{
	cv::Point temp;
	printf("size of b_c is:%d\n", b_c_temp.size());
	//for (int i = 0; i < b_c_temp.size(); ++i)
	//{
		temp = m_b;
		temp.x /= 4.0;
		temp.y /= 4.0;
		temp.x += 163;
		temp.y += 118;
		cvCircle(img, temp, 2, CV_RGB(255, 0, 0), -1, 8);

		temp = m_c;
		temp.x /= 4.0;
		temp.y /= 4.0;
		temp.x += 163;
		temp.y += 118;
		cvCircle(img, temp, 2, CV_RGB(255, 0, 0), -1, 8);
	//}
	cvShowImage("debug_p_q", img);
	cv::waitKey();
	return true;
}

bool Measure::get_target_pose()
{
	float v_b = m_camera_focus_mm * std::tan(m_angle_bGc / 2);
	float u_p = m_camera_focus_mm * std::tan(m_angle_pGq / 2);
	m_O1.x = 0;
	m_O1.y = ((m_target_radius_outer * v_b) / u_p)*sqrt((u_p * u_p - v_b * v_b)/(v_b * v_b + m_camera_focus_mm * m_camera_focus_mm));
	m_O1.z = ((m_target_radius_outer * m_camera_focus_mm) / u_p)*sqrt((u_p*u_p + m_camera_focus_mm*m_camera_focus_mm) / (v_b*v_b + m_camera_focus_mm*m_camera_focus_mm));
	m_O2.x = 0;
	m_O2.y = -m_O1.y;
	m_O2.z = m_O1.z;

	float deno = sqrt((u_p*u_p-v_b*v_b)*(m_camera_focus_mm*m_camera_focus_mm) + (u_p*u_p+m_camera_focus_mm*m_camera_focus_mm)*(v_b*v_b));
	m_n1.x = 0;
	m_n1.y = sqrt(u_p*u_p - v_b*v_b)*m_camera_focus_mm;
	m_n1.z = -sqrt(u_p*u_p + m_camera_focus_mm*m_camera_focus_mm)*v_b;
	m_n1.y /= deno;
	m_n1.z /= deno;

	m_n2.x = m_n1.x;
	m_n2.y = -m_n1.y;
	m_n2.z = m_n1.z;

	printf("cordinates of center is: (%.4f, %.4f, %.4f)\n", m_O1.x, m_O1.y, m_O1.z);
	printf("cordinates of vector n is: (%.4f, %.4f, %.4f)\n", m_n1.x, m_n1.y, m_n1.z);
	printf("press any key to continue!\n");
	//std::getchar();
	return true;
}

bool Measure::convert_pose_cor()
{
	std::fstream save_center_position("center.txt", std::ios::out | std::fstream::app);
	R<<m_u(0), m_v(0), m_w(0),
		m_u(1), m_v(1), m_w(1),
		m_u(2), m_v(2), m_w(2);
	Eigen::Vector3d O1_temp, O2_temp;
	Eigen::Vector3d n1_temp, n2_temp;
	Eigen::Vector3d O1_cam, O2_cam;
	Eigen::Vector3d n1_cam, n2_cam;
	std::cout << "convert matrix is: \n" << R << std::endl;
	O1_temp << m_O1.x, m_O1.y, m_O1.z;
	O2_temp << m_O2.x, m_O2.y, m_O2.z;
	O1_cam = R * O1_temp;
	O2_cam = R * O2_temp;
	std::cout << "cordinate of O1 in camera cordinate is: \n" << O1_cam<<std::endl;
	m_O1_cam.x = O1_cam(0); m_O1_cam.y = O1_cam(1); m_O1_cam.z = O1_cam(2);
	m_O2_cam.x = O2_cam(0); m_O2_cam.y = O2_cam(1); m_O2_cam.z = O2_cam(2);

	n1_temp << m_n1.x, m_n1.y, m_n1.z;
	n2_temp << m_n2.x, m_n2.y, m_n2.z;
	n1_cam = R * n1_temp;
	n2_cam = R * n2_temp;
	std::cout << "cordinate of n1 in camera cordinate is: \n" << n1_cam<<std::endl;
	m_n1_cam.x = n1_cam(0); m_n1_cam.y = n1_cam(1); m_n1_cam.z = n1_cam(2);
	m_n2_cam.x = n2_cam(0); m_n2_cam.y = n2_cam(1); m_n2_cam.z = n2_cam(2);
	save_center_position << m_O1_cam.x << " " << m_O1_cam.y << " " << m_O1_cam.z << " " << 1 << std::endl;
	return true;
}