#include "texture.h"

int seamless_texturing(const Model& src_model, const std::vector<View> &views, const Adj_face_list& adj_list,
	const std::vector<int>& labels, Model& rst_model, std::vector<cv::Mat>& texture_imgs)
{
	TG tg; // texture_generator
	tg.find_connected_area(adj_list, labels);

	tg.vertices_prepare(views, src_model, labels);

	

	tg.color_adjustment_for_vertices(src_model);


	tg.calculate_bboxes(src_model);

	//mapping: origin image to texture;
	int texture_num = tg.patch_arrange();


	//tg.vertex_info_map_to_vec();

	tg.generate_texcoord(src_model, rst_model);

	//color adjusetment & generate texture image
	tg.generate_texture(rst_model, views, texture_num, texture_imgs);
	return texture_num;
}