#pragma once

#include "common.h"
#include "texture.h"


// load images and their poses from file 
int load_view(std::string path, std::vector<View>& view_list);

int read_obj_model(std::string filename, Model& model);

// build face adjecent list
int build_adj_list(const Model& model, Adj_face_list& adj_list);


// output
void write_mtl(std::string filename, int texture_num);

int write_obj(std::string filename, const Model& model, int page_num);

void write_texture(std::string path, const std::vector<cv::Mat>& texture_imgs);


// piling and debug
int read_label_file(std::string filename, std::vector<int>& label);

void write_pile(std::string filename, std::string filename_labeling, Model mesh, std::vector<int>& labeling);