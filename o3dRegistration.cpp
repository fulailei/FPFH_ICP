#include "o3dRegistration.h"
using namespace open3d;

o3dRegistration::o3dRegistration()
{

};
o3dRegistration::~o3dRegistration()
{

};
void o3dRegistration::preprocess_point_cloud(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down, std::shared_ptr<open3d::registration::Feature> &pcd_fpfh) // 传入参数pcd点云数据，voxel_size体素大小
{
	pcd_down = pcd.VoxelDownSample(voxel_size);
	double radius_normal = voxel_size * 2; // # kdtree参数，用于估计法线的半径，一般设为体素大小的2倍
		//print(":: Estimate normal with search radius %.3f." % radius_normal)

	pcd_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, 30)); // # 估计法线的1个参数，使用混合型的kdtree，半径内取最多30个邻居

	double radius_feature = voxel_size * 5; // # kdtree参数，用于估计特征的半径，设为体素大小的5倍
	//print(":: Compute FPFH feature with search radius %.3f." % radius_feature)

	pcd_fpfh = open3d::registration::ComputeFPFHFeature(*pcd_down, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
	
	//return pcd_down, pcd_fpfh  //# 返回降采样的点云、fpfh特征
};
void o3dRegistration::prepare_dataset(std::string source_path,std::string target_path ,double voxel_size)
{
	    //print(":: Load two point clouds and disturb initial pose.")  # 加载2点云并打乱初始的位
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
		//draw_registration_result(source, target, np.identity(4))  # 可视化用初始矩阵配准结果
	
		preprocess_point_cloud(cloud1, voxel_size, save_source_down, save_source_fpfh); // # 降采样和计算特征点
		preprocess_point_cloud(cloud2, voxel_size, save_target_down, save_target_fpfh); // # 降采样和计算特征点
		//return source, target, source_down, target_down, source_fpfh, target_fpfh // # 返回两个点云、两个降采样的点云、两个点云的特征点


		//voxel_size = 0.05 # means 5cm for this dataset，体素大小设定为0.05m
		//source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)  # 调用函数获取这些值
}
void o3dRegistration::prepare_dataset(open3d::geometry::PointCloud cloud1,open3d::geometry::PointCloud cloud2, double voxel_size)
{
	//print(":: Load two point clouds and disturb initial pose.")  # 加载2点云并打乱初始的位
	Eigen::Matrix4d trans_init;

	//draw_registration_result(source, target, np.identity(4))  # 可视化用初始矩阵配准结果
	preprocess_point_cloud(cloud1, voxel_size, save_source_down, save_source_fpfh); // # 降采样和计算特征点
	preprocess_point_cloud(cloud2, voxel_size, save_target_down, save_target_fpfh); // # 降采样和计算特征点
	//return source, target, source_down, target_down, source_fpfh, target_fpfh // # 返回两个点云、两个降采样的点云、两个点云的特征点


	//voxel_size = 0.05 # means 5cm for this dataset，体素大小设定为0.05m
	//source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)  # 调用函数获取这些值
}
Eigen::Matrix4d o3dRegistration::execute_global_registration(open3d::geometry::PointCloud source_down, open3d::geometry::PointCloud target_down,
	open3d::registration::Feature source_fpfh,
	open3d::registration::Feature target_fpfh, double voxel_size)
{
	// # 传入2的点云的降采样，两个点云的特征、体素大小
	double distance_threshold = voxel_size * 1.5;//  # 设定距离阈值为体素的1.5倍
		//print(":: RANSAC registration on downsampled point clouds.")
		//print("   Since the downsampling voxel size is %.3f," % voxel_size)
		//print("   we use a liberal distance threshold %.3f." % distance_threshold)
		//# 2个降采样的点云，两个点云的特征，距离阈值，一个函数，4，一个list[0.9的两个对应点的线段长度阈值，两个点的距离阈值]，一个函数设定最大迭代次数和最大验证次数
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
	/*# 执行配准
				result_ransac = execute_global_registration(source_down, target_down,
					source_fpfh, target_fpfh,
					voxel_size)
				print(result_ransac)
				draw_registration_result(source_down, target_down, result_ransac.transformation) # 源点云旋转平移到目标点云
				*/
}
//# 传入参数：2个点云，2个点云的特征，体素大小
Eigen::Matrix4d o3dRegistration::refine_registration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, open3d::registration::Feature source_fpfh, open3d::registration::Feature target_fpfh,double voxel_size)
{
	double distance_threshold = voxel_size * 0.4;//  # 距离阈值是体素大小的0.4倍
	//	print(":: Point-to-plane ICP registration is applied on original point")
	//	print("   clouds to refine the alignment. This time we use a strict")
	//	print("   distance threshold %.3f." % distance_threshold)
	Eigen::Matrix4d init = execute_global_registration(source, target, source_fpfh, target_fpfh, voxel_size);
	auto result = open3d::registration::RegistrationICP(source, target, distance_threshold, init, open3d::registration::TransformationEstimationPointToPlane());
		
	return result.transformation_;
}

Eigen::Matrix4d o3dRegistration::refine_registration(double voxel_size)
{
	double distance_threshold = voxel_size * 0.4;//  # 距离阈值是体素大小的0.4倍
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
	


				
