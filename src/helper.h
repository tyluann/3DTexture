#pragma once

#include "common.h"
#include "color_adjustment.h"
#include <unordered_map>
#include <map>

int read_obj_model(std::string filename, Model& model);

int read_label_file(std::string filename, std::vector<int>& label);

// build face adjecent list
int build_adj_list(const Model& model, Adj_list& adj_list);

// find connected faces which are of the same label
int find_connected_area(const Adj_list& adj_list, const std::vector<int>& label,
	std::vector<CArea>& area_list/*, std::vector<int>& patch*/);

// load images and their poses from file 
int load_view(std::string path, std::vector<View>& view_list);

// change the data structure, fastly search which patch a face belong
void generate_patch_list(const std::vector<CArea>& area_list, int face_num, std::vector<int>& patch);

// compute the topology_info, color_info and patch info for each vertex (for color adjustment)
void vertices_prepare(const std::vector<View>& view, const Model& model, const std::vector<int>& label,
	/*const std::vector<int>& patch, */const std::vector<CArea>& area_list,
	std::unordered_multimap<int, VInfo> & vertex_info_list);


void calculate_bboxes(const std::vector<CArea>& connected_area_list, const Model& model,
	const std::unordered_multimap<int, VInfo> & vertex_info_list, std::vector<BBox>& patch_bboxes);


int patch_arrange(const std::vector<BBox>& patch_bboxes, int texture_width, int texture_height,
	std::vector<TMap>& texture_map, std::vector<int>& patch_page);

void vertex_info_map_to_vec(const std::unordered_multimap<int, VInfo>& vertex_info_list,
	std::vector<VInfo> &vertex_infos);


void color_adjustment_for_vertices(const std::unordered_multimap<int, VInfo> & vertex_info_list,
	const Model& model, std::vector<std::map<int, int>>& index_table,
	std::vector<cv::Vec3f> &delta_color);


void generate_texcoord(const Model& src_model, const std::vector<VInfo> &vertex_infos,
	const std::vector<std::map<int, int>>& index_table, int texture_width, int texture_height,
	const std::vector<TMap>& texture_map, const std::vector<int>& patch_page,
	const std::vector<CArea>& area_list, Model& dst_model);



void generate_texture(const std::vector<CArea>& connected_area_list,
	const std::unordered_multimap<int, VInfo>& vertex_info_list,
	const Model& model, const std::vector<std::map<int, int>>& index_table,
	const std::vector<cv::Vec3f> &delta_color, const std::vector<TMap>& texture_map,
	const std::vector<View> &views, int texture_width, int texture_height, int texture_num,
	std::vector<cv::Mat>& texture_imgs);

int write_obj(std::string filename, const Model& model, int page_num);

void write_texture(std::string path, const std::vector<cv::Mat>& texture_imgs);