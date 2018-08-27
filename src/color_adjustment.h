#pragma once

#include "common.h"

#include <opencv2\opencv.hpp>
#include <Eigen\Eigen>
#include <vector>
#include <set>


typedef std::vector<std::vector<int>> Adj_list;

struct Model {
	std::vector<Eigen::Vector3f> vertices;
	std::vector<Eigen::Vector3f> normals;
	std::vector<Eigen::Vector2f> texcoords;
	std::vector<int> faces;

	void clear() {
		vertices.clear();
		normals.clear();
		texcoords.clear();
		faces.clear();
	}
};

struct View {
	cv::Mat img;		// RGB image taken from different direction
	Eigen::Matrix3f R;	// Extrinsic: Rotation Matrix
	Eigen::Vector3f t;	// Extrinsic: Translation Vector
	Eigen::Matrix3f K;	// Intrinsic Matrix
};

typedef struct Connected_area {
	int label;
	std::vector<int> faces;
}CArea;

typedef struct Vertex_info {
	int label;
	float x, y;
	//int patch;
	cv::Vec3f color;
	std::set<int> adj_vertices;

	Vertex_info() :label(-1)/*, patch(-1)*/{};
	Vertex_info(int _label/*, int _patch*/) :label(_label)/*, patch(_patch)*/ {};
}VInfo;

struct BBox {
	int patch_index;
	int bb[4];
};

typedef struct Texture_map {
	int texture_num;
	int x, y;
}TMap;

//typedef struct Vertex_color {
//	int label;
//	
//
//	Vertex_color() :label(-1) {};
//
//	Vertex_color(int _label) :label(_label) {};
//}VColor;
