
#include <iostream>

#include "helper.h"

int main(int argc, char** argv)
{

	if (argv[1][0] == '?')argv[1] = argv[1] + 1;

	std::vector<View> view_list;
	load_view(argv[3], view_list);

	Model model;
	read_obj_model(argv[1], model);
	
	std::vector<int> label;
	read_label_file(argv[2], label);

	Adj_list adj_list;
	build_adj_list(model, adj_list);

	std::vector<CArea> connected_area_list;
	find_connected_area(adj_list, label, connected_area_list);

	std::unordered_multimap<int, VInfo> vertex_info_list;
	vertices_prepare(view_list, model, label/*, patch*/, connected_area_list, vertex_info_list);

	//vector index: vertex, map key: label
	std::vector<std::map<int, int>> index_table;
	std::vector<cv::Vec3f> delta_color;
	color_adjustment_for_vertices(vertex_info_list, model, index_table, delta_color);

	std::vector<BBox> patch_bboxes;
	calculate_bboxes(connected_area_list, model, vertex_info_list, patch_bboxes);

	//mapping: origin image to texture;
	std::vector<TMap> texture_map; std::vector<int> patch_page;
	int texture_width = 1000, texture_height = 1000;
	int texture_num = patch_arrange(patch_bboxes, texture_width, texture_height, texture_map, patch_page);

	std::vector<VInfo> vertex_infos;
	vertex_info_map_to_vec(vertex_info_list, vertex_infos);

	generate_texcoord(model, vertex_infos, index_table, texture_width, texture_height,
		texture_map, patch_page, connected_area_list, model);

	//color_adjusetment & generate_texture_image
	std::vector<cv::Mat> texture_imgs;
	generate_texture(connected_area_list, vertex_info_list, model, index_table, delta_color,
		 texture_map, view_list, texture_width, texture_height, texture_num, texture_imgs);

	std::string out_dir = argv[4];
	write_obj(out_dir + "/result.obj", model, texture_num);

	write_texture(out_dir, texture_imgs);

	system("pause");
	return 0;
}