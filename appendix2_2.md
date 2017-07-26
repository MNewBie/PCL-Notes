# 常用函数(2)

以下代码为平时测试代码，实现并不严谨，仅供参考！

* **计算模型分辨率**

resolution.h

```
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}
```

* **计算旋转平移矩阵**

getTransformation.h

```
#ifndef _GET_TRANSFORMATION_
#define _GET_TRANSFORMATION_

#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <boost/random.hpp>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/search/pcl_search.h>

int getTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& tar,
	double resolution,
	pcl::Correspondences &correspondences, Eigen::Matrix4f & transform)
{
	int cor_size = correspondences.size();
	if (cor_size < 3)
	{
		pcl::console::print_error("matching less 3");
		return (-1);
	}

	float fitness_score = FLT_MAX;
	boost::mt19937 gen;
	boost::uniform_int<> dist(0, cor_size - 1);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<>> die(gen, dist);
	pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr transformationEstimation(new pcl::registration::TransformationEstimation3Point <pcl::PointXYZ, pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(tar);
	//	float max_inlier_dist_sqr_ = 0.0064;
	for (int i = 0; i < 10000; ++i)
	{
		int cor1, cor2, cor3;
		cor1 = die();
		while (true)
		{
			cor2 = die();
			if (cor2 != cor1)
				break;
		}
		while (true)
		{
			cor3 = die();
			if (cor3 != cor1 && cor3 != cor2)
				break;
		}

		pcl::Correspondences correspondences_temp;
		correspondences_temp.push_back(correspondences[cor1]);
		correspondences_temp.push_back(correspondences[cor2]);
		correspondences_temp.push_back(correspondences[cor3]);
		Eigen::Matrix4f transform_temp;
		transformationEstimation->estimateRigidTransformation(*src, *tar, correspondences_temp, transform_temp);

		// 先看三个对应点变换后是否匹配
		pcl::PointCloud<pcl::PointXYZ> src_temp;
		pcl::PointCloud<pcl::PointXYZ> tar_temp;
		src_temp.push_back(src->points[correspondences[cor1].index_query]);
		tar_temp.push_back(tar->points[correspondences[cor1].index_match]);
		src_temp.push_back(src->points[correspondences[cor2].index_query]);
		tar_temp.push_back(tar->points[correspondences[cor2].index_match]);
		src_temp.push_back(src->points[correspondences[cor3].index_query]);
		tar_temp.push_back(tar->points[correspondences[cor3].index_match]);

		pcl::PointCloud<pcl::PointXYZ> src_transformed;
		pcl::transformPointCloud(src_temp, src_transformed, transform_temp);
		float mse = 0.f;
		for (int k = 0; k < 3; ++k)
		{
			mse += pcl::squaredEuclideanDistance(src_transformed.points[k], tar_temp.points[k]);
		}
		mse /= 3;
		if (mse > 2* resolution) 
			continue;

		// 整体变换、得出评价
		pcl::PointCloud<pcl::PointXYZ> match_transformed;
		pcl::transformPointCloud(*src, match_transformed, transform_temp);

		std::vector <int> ids;
		std::vector <float> dists_sqr;
		float score = 0.f;
		for (int k = 0; k < match_transformed.size(); ++k)
		{
			tree.nearestKSearch(src->points[k], 1, ids, dists_sqr);
			score += dists_sqr[0];
			//			score += (dists_sqr[0] < max_inlier_dist_sqr_ ? dists_sqr[0] : max_inlier_dist_sqr_);
		}
		score /= match_transformed.size();
		//		score /= (max_inlier_dist_sqr_ * match_transformed.size());

		if (score < fitness_score)
		{
			fitness_score = score;
			transform = transform_temp;
		}

	}
	return 0;
}

#endif
```