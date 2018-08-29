#pragma once

#include "common.h"

#include <opencv2\opencv.hpp>
#include <Eigen\Eigen>
#include <vector>
#include <set>
#include <unordered_map>
#include <map>

typedef std::vector<std::vector<int>> Adj_face_list;

struct Model { //3D model
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

typedef struct Connected_area { // a connected area of the same label. correspond to patches(img).
	int label;
	std::vector<int> faces;
}CArea;

typedef struct SUB_Vertex_info { // sub-vertex color, index, uv coordinates, adjacent vertices
	int v_index;
	int label;
	float x, y;
	//int patch;
	cv::Vec3f color;
	std::set<int> adj_vertices;

	SUB_Vertex_info() :label(-1), v_index(-1) {};
	SUB_Vertex_info(int _label, int _v_index) :label(_label), v_index(_v_index) {};
}VInfo;

struct BBox { // patch bounding box
	int patch_index;
	int bb[4];
};

typedef struct Texture_map { // patch table
	int texture_page;
	int x, y;
}TMap;

class Graph
{
public:
	int faces_num;
	int views_num;
	std::vector<std::vector<float>> data_costs;
	std::vector<bool> b_good_face;
	std::vector<std::vector<int>> valid_mask;

	Graph::Graph(int _faces_num, int _views_num);
	int calculate_data_cost(const Model& model, const std::vector<View>& views);
};


#define VEC_VERTICE_SIZE std::vector
#define VEC_SUB_VERTICE_SIZE std::vector
#define VEC_PATCHES_SIZE std::vector

typedef class Texture_generator{
private:
	int texture_width;
	int texture_height;
	VEC_PATCHES_SIZE<CArea> connected_areas; // connected areas(faces) of the same label. correspond to patches(img).
	VEC_PATCHES_SIZE<BBox> patch_bboxes; // image patches' bounding boxes
	VEC_PATCHES_SIZE<TMap> texture_map; // patches location map to texture img.
	
	VEC_SUB_VERTICE_SIZE<VInfo> vertex_infos; // sub-vertex color, index, uv coordinates, adjacent vertices
	VEC_SUB_VERTICE_SIZE<cv::Vec3f> delta_color; // color adjustments for 3 channels.

	VEC_VERTICE_SIZE<std::map<int, int>> sub_vertex_table; //vector::index: vertex, map::key: label, map::value: sub_vertex
	
public:
	int patches_num;
	int sub_vertices_num;

	Texture_generator() :texture_height(1000), texture_width(1000) {}
	Texture_generator(int _texture_height, int _texture_width):texture_height(_texture_height), texture_width(_texture_width){}

	int find_connected_area(const Adj_face_list& adj_face_list, const std::vector<int>& label);

	// compute the topology_info, color_info and patch info for each vertex (for color adjustment)
	void vertices_prepare(const std::vector<View>& view, const Model& model, const std::vector<int>& label);

	// calculate bounding boxes for image patches
	void calculate_bboxes(const Model& model);

	// arrange patch location on texture image.
	int patch_arrange();


	void color_adjustment_for_vertices(const Model& model);

	// generate uv texcoord for model and renderer to show texture
	void generate_texcoord(const Model& src_model, Model& dst_model);


	void generate_texture(const Model& model, const std::vector<View> &views, int texture_num,
		std::vector<cv::Mat>& texture_imgs);
}TG;

int labelling(const Model &model, const Adj_face_list &adj_face_list,
	const std::vector<View>& views, std::vector<int>& labels);

int seamless_texturing(const Model& src_model, const std::vector<View> &views, const Adj_face_list& adj_face_list,
	const std::vector<int>& labels, Model& rst_model, std::vector<cv::Mat>& texture_imgs);