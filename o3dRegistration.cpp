#include "o3dRegistration.h"
using namespace open3d;

o3dRegistration::o3dRegistration()
{

};
o3dRegistration::~o3dRegistration()
{

};
void o3dRegistration::preprocess_point_cloud(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down, std::shared_ptr<open3d::registration::Feature> &pcd_fpfh) // �������pcd�������ݣ�voxel_size���ش�С
{
	pcd_down = pcd.VoxelDownSample(voxel_size);
	double radius_normal = voxel_size * 2; // # kdtree���������ڹ��Ʒ��ߵİ뾶��һ����Ϊ���ش�С��2��
		//print(":: Estimate normal with search radius %.3f." % radius_normal)

	pcd_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, 30)); // # ���Ʒ��ߵ�1��������ʹ�û���͵�kdtree���뾶��ȡ���30���ھ�

	double radius_feature = voxel_size * 5; // # kdtree���������ڹ��������İ뾶����Ϊ���ش�С��5��
	//print(":: Compute FPFH feature with search radius %.3f." % radius_feature)

	pcd_fpfh = open3d::registration::ComputeFPFHFeature(*pcd_down, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
	
	//return pcd_down, pcd_fpfh  //# ���ؽ������ĵ��ơ�fpfh����
};
void o3dRegistration::prepare_dataset(std::string source_path,std::string target_path ,double voxel_size)
{
	    //print(":: Load two point clouds and disturb initial pose.")  # ����2���Ʋ����ҳ�ʼ��λ
		open3d::geometry::PointCloud cloud1;
		open3d::geometry::PointCloud cloud2;
	    bool ok1 = open3d::io::ReadPointCloudFromPCD(source_path, cloud1);
		bool ok2 = open3d::io::ReadPointCloudFromPCD(target_path, cloud2);
		//save_source->points_ = cloud1.points_;
		//save_target->points_ = cloud2.points_;
		Eigen::Matrix4d trans_init;
		
		/*trans_init << 0.0, 0.0, 1.0, 0.0,
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0;
		cloud1.Transform(trans_init);
		*/
		//draw_registration_result(source, target, np.identity(4))  # ���ӻ��ó�ʼ������׼���
	
		preprocess_point_cloud(cloud1, voxel_size, save_source_down, save_source_fpfh); // # �������ͼ���������
		preprocess_point_cloud(cloud2, voxel_size, save_target_down, save_target_fpfh); // # �������ͼ���������
		//return source, target, source_down, target_down, source_fpfh, target_fpfh // # �����������ơ������������ĵ��ơ��������Ƶ�������


		//voxel_size = 0.05 # means 5cm for this dataset�����ش�С�趨Ϊ0.05m
		//source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)  # ���ú�����ȡ��Щֵ
}
void o3dRegistration::prepare_dataset(open3d::geometry::PointCloud cloud1,open3d::geometry::PointCloud cloud2, double voxel_size)
{
	//print(":: Load two point clouds and disturb initial pose.")  # ����2���Ʋ����ҳ�ʼ��λ
	Eigen::Matrix4d trans_init;

	//draw_registration_result(source, target, np.identity(4))  # ���ӻ��ó�ʼ������׼���
	preprocess_point_cloud(cloud1, voxel_size, save_source_down, save_source_fpfh); // # �������ͼ���������
	preprocess_point_cloud(cloud2, voxel_size, save_target_down, save_target_fpfh); // # �������ͼ���������
	//return source, target, source_down, target_down, source_fpfh, target_fpfh // # �����������ơ������������ĵ��ơ��������Ƶ�������


	//voxel_size = 0.05 # means 5cm for this dataset�����ش�С�趨Ϊ0.05m
	//source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)  # ���ú�����ȡ��Щֵ
}
Eigen::Matrix4d o3dRegistration::execute_global_registration(open3d::geometry::PointCloud source_down, open3d::geometry::PointCloud target_down,
	open3d::registration::Feature source_fpfh,
	open3d::registration::Feature target_fpfh, double voxel_size)
{
	// # ����2�ĵ��ƵĽ��������������Ƶ����������ش�С
	double distance_threshold = voxel_size * 1.5;//  # �趨������ֵΪ���ص�1.5��
		//print(":: RANSAC registration on downsampled point clouds.")
		//print("   Since the downsampling voxel size is %.3f," % voxel_size)
		//print("   we use a liberal distance threshold %.3f." % distance_threshold)
		//# 2���������ĵ��ƣ��������Ƶ�������������ֵ��һ��������4��һ��list[0.9��������Ӧ����߶γ�����ֵ��������ľ�����ֵ]��һ�������趨�����������������֤����
	std::vector<std::reference_wrapper<const open3d::registration::CorrespondenceChecker>>checkers;
	auto EdgeLength = open3d::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
	checkers.push_back(EdgeLength);
	auto Distance = open3d::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
	checkers.push_back(Distance);
	auto result =open3d::registration::RegistrationRANSACBasedOnFeatureMatching(source_down, target_down, source_fpfh, target_fpfh,
		distance_threshold,open3d::registration::TransformationEstimationPointToPoint(false),4, checkers,
		open3d::registration::RANSACConvergenceCriteria(4000000,500));
	Eigen::Matrix4d mat = result.transformation_;
	auto set = result.correspondence_set_;
	return mat;
		//return result
	/*# ִ����׼
				result_ransac = execute_global_registration(source_down, target_down,
					source_fpfh, target_fpfh,
					voxel_size)
				print(result_ransac)
				draw_registration_result(source_down, target_down, result_ransac.transformation) # Դ������תƽ�Ƶ�Ŀ�����
				*/
}
//# ���������2�����ƣ�2�����Ƶ����������ش�С
Eigen::Matrix4d o3dRegistration::refine_registration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, open3d::registration::Feature source_fpfh, open3d::registration::Feature target_fpfh,double voxel_size)
{
	double distance_threshold = voxel_size * 0.4;//  # ������ֵ�����ش�С��0.4��
	//	print(":: Point-to-plane ICP registration is applied on original point")
	//	print("   clouds to refine the alignment. This time we use a strict")
	//	print("   distance threshold %.3f." % distance_threshold)
	Eigen::Matrix4d init = execute_global_registration(source, target, source_fpfh, target_fpfh, voxel_size);
	auto result = open3d::registration::RegistrationICP(source, target, distance_threshold, init, open3d::registration::TransformationEstimationPointToPlane());
		
	return result.transformation_;
}

Eigen::Matrix4d o3dRegistration::refine_registration(double voxel_size)
{
	double distance_threshold = voxel_size * 0.4;//  # ������ֵ�����ش�С��0.4��
	//	print(":: Point-to-plane ICP registration is applied on original point")
	//	print("   clouds to refine the alignment. This time we use a strict")
	//	print("   distance threshold %.3f." % distance_threshold)
	Eigen::Matrix4d init = execute_global_registration(*save_source_down, *save_target_down, *save_source_fpfh, *save_target_fpfh, voxel_size);
	auto result = open3d::registration::RegistrationICP(*save_source_down, *save_target_down, distance_threshold, init, open3d::registration::TransformationEstimationPointToPlane());

	return result.transformation_;



}
void o3dRegistration::execute_fast_global_registration(open3d::geometry::PointCloud source_down, open3d::geometry::PointCloud target_down, open3d::registration::Feature source_fpfh,
	open3d::registration::Feature target_fpfh, double voxel_size)
{
	/*
	double distance_threshold = voxel_size * 0.5;
	//	print(":: Apply fast global registration with distance threshold %.3f" \
		//	% distance_threshold)
    auto result=open3d::registration::re
		result = o3d.registration.registration_fast_based_on_feature_matching(
			source_down, target_down, source_fpfh, target_fpfh,
			o3d.registration.FastGlobalRegistrationOption(
				maximum_correspondence_distance = distance_threshold))
		return result
	*/
}
	


				
