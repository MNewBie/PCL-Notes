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

* **旋转平移矩阵评估**

参考论文：
Mian A, Bennamoun M, Owens R. A novel representation and feature matching algorithm for automatic pairwise registration of range images[J]. International Journal of Computer Vision, 66(1), 19–40.

evaluation.h

```
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;

//compute ideal ratation and translation matrix
Eigen::Matrix4f createIdealTransformationMatrix(float tx, float ty, float tz, float rx, float ry, float rz)
{
	////rotating and translating point cloud
	////m and m1 are both the unit matrixes
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
	//tx,ty,tz are translation values of x ,y and z axis
	m1(0, 3) = tx;
	m1(1, 3) = ty;
	m1(2, 3) = tz;
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the translation
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the x axis
	m1 = Eigen::Matrix4f::Identity();
	rx = rx / 180.0 * M_PI;
	m1(1, 1) = cos(rx);
	m1(1, 2) = -sin(rx);
	m1(2, 1) = sin(rx);
	m1(2, 2) = cos(rx);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along x axis
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the y axis
	m1 = Eigen::Matrix4f::Identity();
	ry = ry / 180.0 * M_PI;
	m1(0, 0) = cos(ry);
	m1(0, 2) = sin(ry);
	m1(2, 0) = -sin(ry);
	m1(2, 2) = cos(ry);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along y axis
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the z axis
	m1 = Eigen::Matrix4f::Identity();
	rz = rz / 180.0 * M_PI;
	m1(0, 0) = cos(rz);
	m1(0, 1) = -sin(rz);
	m1(1, 0) = sin(rz);
	m1(1, 1) = cos(rz);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along y axis
	//cout << "m" << endl << m << endl;

	return m;
}

//compute the error of rotation matrix and translation matrix
void computeMatrixError(float rotation, float translation, float & rotationError, float &translationError, Eigen::Matrix4f &realRtMatrix)
{
	Eigen::Matrix4f idealMatrix = Eigen::Matrix4f::Identity();
	idealMatrix = createIdealTransformationMatrix(translation, translation, translation, rotation, rotation, rotation);
	//cout << "idealMatrix:"<<endl << idealMatrix << endl;


	Eigen::Matrix3f idealRotationMatrix = Eigen::Matrix3f::Identity();
	Eigen::Vector3f iealTranslationVector = Eigen::Vector3f(0, 0, 0);

	//getting ideal rotation matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			idealRotationMatrix(i, j) = idealMatrix(i, j);
		}
	}

	//getting ideal translation matrix
	for (int i = 0; i < 3; i++)
	{
		iealTranslationVector(i) = idealMatrix(i, 3);
	}
	cout << "ideal Rotation matrix:" << endl << idealRotationMatrix << endl;
	cout << "ideal Translatin vector:" << endl << iealTranslationVector << endl;

	cout << "real Matrix:" << endl << realRtMatrix << endl;

	Eigen::Matrix3f realRotationMatrix = Eigen::Matrix3f::Identity();
	Eigen::Vector3f realTranslationVector = Eigen::Vector3f(0, 0, 0);

	//getting ideal rotation matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			realRotationMatrix(i, j) = realRtMatrix(i, j);
		}
	}
	//getting real translation matrix
	for (int i = 0; i < 3; i++)
	{
		realTranslationVector(i) = realRtMatrix(i, 3);
	}

	cout << "real Rotation matrix:" << endl << realRotationMatrix << endl;
	cout << "real Translatin vector:" << endl << realTranslationVector << endl;

	//compute rotation error
	Eigen::Matrix3f m = idealRotationMatrix * realRotationMatrix.inverse();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
	es.compute(m);
	float tr = es.eigenvalues()(0) + es.eigenvalues()(1) + es.eigenvalues()(2);
	if (tr > 3) tr = 3;
	if (tr < -3) tr = -3;
	rotationError = acos((tr - 1) / 2) * 180.0 / M_PI;

	//compute translation error
	translationError = (realTranslationVector - iealTranslationVector).norm();
}
```

* **将点旋转平移**

tranform.h

```
#pragma once
#include <pcl/common/transforms.h>

void MyTransformationPoint(pcl::PointXYZ &pt_in, pcl::PointXYZ &pt_out, float tx, float ty, float tz, float rx, float ry, float rz)
{
	////rotating and translating point cloud
	////m and m1 are both the unit matrixes
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
	//tx,ty,tz are translation values of x ,y and z axis
	m1(0, 3) = tx;
	m1(1, 3) = ty;
	m1(2, 3) = tz;
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the translation
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the x axis
	m1 = Eigen::Matrix4f::Identity();
	rx = rx / 180.0 * M_PI;
	m1(1, 1) = cos(rx);
	m1(1, 2) = -sin(rx);
	m1(2, 1) = sin(rx);
	m1(2, 2) = cos(rx);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along x axis
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the y axis
	m1 = Eigen::Matrix4f::Identity();
	ry = ry / 180.0 * M_PI;
	m1(0, 0) = cos(ry);
	m1(0, 2) = sin(ry);
	m1(2, 0) = -sin(ry);
	m1(2, 2) = cos(ry);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along y axis
	//cout << "m" << endl << m << endl;
	//rotating the point cloud along the z axis
	m1 = Eigen::Matrix4f::Identity();
	rz = rz / 180.0 * M_PI;
	m1(0, 0) = cos(rz);
	m1(0, 1) = -sin(rz);
	m1(1, 0) = sin(rz);
	m1(1, 1) = cos(rz);
	//cout << "m1" << endl << m1 << endl;
	m = m * m1;   //apply the rotation along y axis
	//cout << "m" << endl << m << endl;

	//pcl::transformPointCloud(src, dst, m);
	Eigen::Matrix<float, 3, 1> pt(pt_in.x, pt_in.y, pt_in.z);
	pt_out.x = static_cast<float> (m(0, 0) * pt.coeffRef(0) + m(0, 1) * pt.coeffRef(1) + m(0, 2) * pt.coeffRef(2) + m(0, 3));
	pt_out.y = static_cast<float> (m(1, 0) * pt.coeffRef(0) + m(1, 1) * pt.coeffRef(1) + m(1, 2) * pt.coeffRef(2) + m(1, 3));
	pt_out.z = static_cast<float> (m(2, 0) * pt.coeffRef(0) + m(2, 1) * pt.coeffRef(1) + m(2, 2) * pt.coeffRef(2) + m(2, 3));
}
```