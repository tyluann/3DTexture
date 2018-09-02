#include "texture.h"
#include <iostream>
#include "helper.h"
#include <direct.h>
#include <time.h>


int main(int argc, char** argv)
{
	if (argv[1][0] == '?')argv[1] = argv[1] + 1;

	if (argc < 4) {
		LOGE("not enough in put arguments!");
		return -1;
	}

	std::string obj_file = argv[1];
	std::string view_dir = argv[2];
	std::string out_dir = argv[3];
	std::string label_file;
	if (argc > 4)
		label_file = argv[4];
	_mkdir(out_dir.c_str());
	clock_t t0, t1;

	std::vector<View> views;
	load_view(view_dir, views);

	Model model;
	read_obj_model(obj_file, model);
	
	Adj_face_list adj_face_list;
	build_adj_list(model, adj_face_list);

	std::vector<int> labels;
#ifdef TEX_DEBUG
	read_label_file(label_file, labels);
#else
	t0 = clock();
	labelling(model, adj_face_list, views, labels);
	t1 = clock();
	LOGI("Labelling time: %fs", static_cast<float>(t1 - t0) / CLOCKS_PER_SEC);
	write_pile(out_dir + "/label.obj", out_dir + "/label.txt", model, labels);
#endif

	std::vector<cv::Mat> texture_imgs;
	t0 = clock();
	int texture_num = seamless_texturing(model, views, adj_face_list, labels, model, texture_imgs);
	t1 = clock();
	LOGI("Seamless texturing time: %fs", static_cast<float>(t1 - t0) / CLOCKS_PER_SEC);
	
	write_obj(out_dir + "/result.obj", model, texture_num);

	write_mtl(out_dir + "/result.mtl", texture_num);

	write_texture(out_dir, texture_imgs);
	return 0;
}