#pragma once
#include "open3d/Open3D.h"
#include "Open3D/Geometry/VoxelGrid.h"
#include <Eigen/Core>
class o3dRegistration
{
public:
	 o3dRegistration();
	~o3dRegistration();

	void preprocess_point_cloud(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud>& pcd_down, std::shared_ptr<open3d::registration::Feature>& pcd_fpfh);

	void prepare_dataset(std::string source_path, std::string target_path, double voxel_size);

	void prepare_dataset(open3d::geometry::PointCloud cloud1, open3d::geometry::PointCloud cloud2, double voxel_size);

	Eigen::Matrix4d execute_global_registration(open3d::geometry::PointCloud source_down, open3d::geometry::PointCloud target_down, open3d::registration::Feature source_fpfh, open3d::registration::Feature target_fpfh, double voxel_size);

	Eigen::Matrix4d refine_registration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, open3d::registration::Feature source_fpfh, open3d::registration::Feature target_fpfh, double voxel_size);

	Eigen::Matrix4d refine_registration(double voxel_size);

	void execute_fast_global_registration(open3d::geometry::PointCloud source_down, open3d::geometry::PointCloud target_down, open3d::registration::Feature source_fpfh, open3d::registration::Feature target_fpfh, double voxel_size);

	std::shared_ptr<open3d::geometry::PointCloud> save_source= std::shared_ptr<open3d::geometry::PointCloud>();
	std::shared_ptr<open3d::geometry::PointCloud> save_target= std::shared_ptr<open3d::geometry::PointCloud>();
	std::shared_ptr<open3d::geometry::PointCloud> save_source_down= std::shared_ptr<open3d::geometry::PointCloud>();
	std::shared_ptr<open3d::registration::Feature> save_source_fpfh= std::shared_ptr<open3d::registration::Feature>();
	std::shared_ptr<open3d::geometry::PointCloud> save_target_down= std::shared_ptr<open3d::geometry::PointCloud>();
	std::shared_ptr<open3d::registration::Feature> save_target_fpfh = std::shared_ptr<open3d::registration::Feature>();;

private:




};

