#include <Eigen/Dense>
#include <vector>
#include <opencv.hpp>
enum EclipType
{
	PRED_INNER,
	PRED_OUTER,
	GT_INNER,
	GT_OUTER
};
class Measure
{
public:
	Measure();
	~Measure();
	bool get_eclip_points(cv::Mat& img, EclipType eclip_type);
	bool get_p_q();
	bool debug_p_q(IplImage* img);
	bool get_b_c();
	bool debug_b_c(IplImage* img);
	bool cal_w_v_u();
	bool get_target_pose();
	bool convert_pose_cor();
	float cal_angle(const float d1, const float d2, const float d3);
	float cal_dis(const cv::Point3d p1, const cv::Point3d p2);
	float cal_norm(const Eigen::Vector3d v);
private:
	cv::Size m_img_size;
	cv::Point m_p, m_q, m_b, m_c;
	Eigen::Vector3d m_w, m_v, m_u;
	std::vector<cv::Point> m_inner_points;
	std::vector<cv::Point> m_outer_points;
	std::vector<cv::Point> b_c_temp;
	const float m_camera_focus_mm = 5.152;//via intrinsic param of camera
	const float m_camera_focus_pix = 5.152 * 1000 / 3.75;
	const float pix2mm = 3.75 / 1000;
	const float m_target_radius_outer = 215.0;//mm
	const float m_target_radius_inner = 50.0;
	float m_angle_pGq, m_angle_bGc;
	cv::Point3d m_O1, m_O2;
	cv::Point3d m_n1, m_n2;
	cv::Point3d m_O1_cam, m_O2_cam;
	cv::Point3d m_n1_cam, m_n2_cam;
	Eigen::Matrix3d R;
};

