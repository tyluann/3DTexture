#include "texture.h"

#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <io.h>
#include <queue>
#include <iomanip>


int read_obj_model(std::string filename, Model& model) {
	std::ifstream fin(filename, std::ios::in);
	if (!fin.good()) {
		LOGE("obj file %s does not exist!\n",filename.c_str() );
		return -1;
	}
	std::string line;
	bool has_vn = false;
	while (std::getline(fin, line)) {
		std::istringstream sin(line);
		std::string format;
		sin >> format;
		if (format == "v") {
			float x, y, z;
			sin >> x >> y >> z;
			model.vertices.emplace_back(x, y, z);
		}
		if (format == "vn") {
			has_vn = true;
			float x, y, z;
			sin >> x >> y >> z;
			model.normals.emplace_back(x, y, z);
		}
		if (format == "f") {
			int f[10] = { 0 };
			size_t space = line.find_first_of(' ');
			line = line.substr(space + 1, -1);
			

			tex_loop(i, 3){ // 3d points index
				space = line.find_first_of(' ');
				std::string p = line.substr(0, space);
				size_t slash = 0;
				int j = 0; // 0:vertex, 3:normal, 6:texcoord
				do {
					slash = p.find_first_of('/');
					if (slash == 0) {
						p = p.substr(1, -1);
						slash = p.find_first_of('/');
					}
					std::string num = p.substr(0, slash);
					f[i + j] = std::stoi(num) - 1;
					p = p.substr(slash + 1, -1);
					j += 3;
				} while (slash != std::string::npos);
				line = line.substr(space + 1, -1);
			}
			
			std::vector<int> one_face({ f[0], f[1], f[2], f[3], f[4], f[5], 0, 0, 0, 0 });// dont need texcoord
			if (f[0] != f[1] && f[1] != f[2] && f[2] != f[0])
				model.faces.insert(model.faces.end(), one_face.begin(), one_face.end());
		}
	}

	fin.close();
	return 0;
}

int read_label_file(std::string filename, std::vector<int>& label) {
	std::ifstream fin(filename, std::ios::in);
	if (!fin.good()) {
		LOGE("label file %s does not exist!\n", filename.c_str());
		return -1;
	}
	int cur_label = 0;
	while (fin >> cur_label) {
		label.push_back(cur_label);
	}
	fin.close();
}

// insert a certain edge to edge_map. Output the correspondant face pair if found one. 
int insert(size_t edge_key, int current_face,
	std::unordered_map<size_t, int>& edge_map, Adj_face_list& adj_list)
{
	std::unordered_map<size_t, int>::iterator map_it = edge_map.find(edge_key);
	if (map_it == edge_map.end()) edge_map[edge_key] = current_face; //the 1st face
	else { //the 2nd face
		int& face0 = map_it->second;
		int& face1 = current_face;
		if (face0 == -1) { // the 3rd face. Maniford mesh has at most 2 faces by each edge.
			LOGE("non maniford mesh! current_face: %d", face1);
			return -1;
		}
		adj_list[face0].push_back(face1);
		adj_list[face1].push_back(face0);
		map_it->second = -1;
	}
	return 0;
}


int build_adj_list(const Model& model, Adj_face_list& adj_list) {
	adj_list.clear();
	adj_list.resize(model.faces.size()/10, std::vector<int>());
	std::unordered_map<size_t, int> edge_map; // key: undirected edge, value: face_index
		//edge_map(model.vertices.size(), std::vector<int>()); // faces list of each vertex composite.
	for (int i = 0, ii = 0; ii < model.faces.size(); ++i, ii += 10) { // i:face_index, ii: 3 * face_index

		// sort vertex_index for undirected edge
		int v0 = model.faces[ii];
		int v1 = model.faces[ii + 1];
		int v2 = model.faces[ii + 2];
		if (v0 > v1)std::swap(v0, v1);
		if (v1 > v2)std::swap(v1, v2);
		if (v0 > v1)std::swap(v0, v1);

		size_t edge_key0 = v0 * model.vertices.size() + v1;
		size_t edge_key1 = v0 * model.vertices.size() + v2;
		size_t edge_key2 = v1 * model.vertices.size() + v2;

		if (insert(edge_key0, i, edge_map, adj_list) == -1) return -1;
		if (insert(edge_key1, i, edge_map, adj_list) == -1) return -1;
		if (insert(edge_key2, i, edge_map, adj_list) == -1) return -1;
	}
	
#if 0
	int count_greater3 = 0, count_less1 = 0;
	tex_loop(i, adj_list.size()) {
		if (adj_list[i].size() > 3) {
			count_greater3++;
		}
		if (adj_list[i].size() < 1) {
			count_less1++;
		}
	}
	LOGI("Greater than 3 face count: %d.\n  Less than 3 face count : %d.", count_greater3, count_less1);
#endif
	return 0;
}

int load_view(std::string path, std::vector<View>& view_list) {
	intptr_t handle;
	_finddata_t findData;
	handle = _findfirst((path + "/*.jpg").c_str(), &findData);
	if (handle == -1){
		LOGE("Image dir is empty!");
		return -1;
	}
	do {
		if (!(findData.attrib & _A_SUBDIR && strcmp(findData.name, ".") == 0
			&& strcmp(findData.name, "..") == 0))
		{
			std::string img_name = findData.name;
			size_t pos = img_name.find_last_of(".");
			std::string appendix = img_name.substr(pos, -1);
			std::string name = img_name.substr(0, pos);
			if (appendix == ".jpg") {
				std::string pose_name = path + "/" + name + ".cam";
				std::ifstream fin(pose_name, std::ios::in);
				if (!fin.good()) {
					LOGW("image %s has no pose file.", img_name);
				}
				else {
					view_list.push_back(View());
					View& view = view_list[view_list.size() - 1];
					view.img = cv::imread(path + "/" + img_name);

					Eigen::Matrix3f& R = view.R;
					Eigen::Vector3f& t = view.t;
					Eigen::Matrix3f& K = view.K;

					//extrinsic
					fin >> t(0) >> t(1) >> t(2);
					fin >> R(0, 0) >> R(0, 1) >> R(0, 2)
						>> R(1, 0) >> R(1, 1) >> R(1, 2)
						>> R(2, 0) >> R(2, 1) >> R(2, 2);

					float f, d0, d1, ppect, cx, cy; //intrinsic
					fin >> f >> d0 >> d1 >> ppect >> cx >> cy;

					// to pixel coodinates
					f *= std::max(view.img.cols, view.img.rows);
					cx *= view.img.cols;
					cy *= view.img.rows;
					
					// d0, d1, ppect are not used.
					K(0, 0) = f; K(0, 1) = 0; K(0, 2) = cx;
					K(1, 0) = 0; K(1, 1) = f; K(1, 2) = cy;
					K(2, 0) = 0; K(2, 1) = 0; K(2, 2) = 1;

					fin.close();
				}

			}
		}
	} while (_findnext(handle, &findData) == 0);

	_findclose(handle);
	return 0;
}


int write_obj(std::string filename, const Model& model, int page_num) {
	std::ofstream out(filename.c_str(), std::ios::out);
	if (!out.good()) {
		LOGE("fail to open %s", filename);
		return -1;
	}
	const std::vector<Eigen::Vector3f>& vertices = model.vertices;
	const std::vector<Eigen::Vector3f>& normals = model.normals;
	const std::vector<Eigen::Vector2f>& texcoords = model.texcoords;
	const std::vector<int>& faces = model.faces;

	size_t num_faces = faces.size() / 10;

	out << "mtllib result.mtl" << std::endl;
	for (int i = 0; i < vertices.size(); ++i) {
		out << "v ";
		out << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << std::endl;
	}
	for (int i = 0; i < texcoords.size(); ++i) {
		out << "vt ";
		out << texcoords[i][0] << " " << texcoords[i][1] << std::endl;
	}
	for (int i = 0; i < normals.size(); ++i) {
		out << "vn ";
		out << normals[i][0] << " " << normals[i][1] << " " << normals[i][2] << std::endl;
	}
	std::vector<std::vector<int>> pages(page_num);
	for (int ii = 0; ii < faces.size(); ii += 10) {
		const int &page = faces[ii + 9];
		pages[page].push_back(ii);
	}
	LOGI("Total texture pages: %d", pages.size());

	tex_loop(p, pages.size()) {
		
		out << "usemtl m";
		out << std::setfill('0') << std::setw(4) << p << std::endl;
		for (auto it = pages[p].begin(); it != pages[p].end(); ++it){
			int &ii = *it;
			out << "f ";
			out << faces[ii + 0] + 1 << "/" << faces[ii + 6] + 1 << "/" << faces[ii + 3] + 1 << " ";
			out << faces[ii + 1] + 1 << "/" << faces[ii + 7] + 1 << "/" << faces[ii + 4] + 1 << " ";
			out << faces[ii + 2] + 1 << "/" << faces[ii + 8] + 1 << "/" << faces[ii + 5] + 1;
			out << std::endl;
		}
	}

	out.close();
	return 0;
}

void write_mtl(std::string filename, int texture_num) {
	std::ofstream fout(filename);
	tex_loop(i, texture_num) {
		fout << "newmtl m";
		fout << std::setfill('0') << std::setw(4) << i << std::endl;
		fout << "Ka 1.000000 1.000000 1.000000" << std::endl;
		fout << "Kd 1.000000 1.000000 1.000000" << std::endl;
		fout << "Ks 0.000000 0.000000 0.000000" << std::endl;
		fout << "Tr 0.000000" << std::endl;
		fout << "illum 1" << std::endl;
		fout << "Ns 1.000000" << std::endl;
		fout << "map_Kd ";
		fout << std::setfill('0') << std::setw(4) << i;
		fout << ".png" << std::endl;
	}
}

void write_texture(std::string path, const std::vector<cv::Mat>& texture_imgs) {
	tex_loop(i, texture_imgs.size()) {
		
		std::ostringstream ostr;
		ostr << std::setfill('0') << std::setw(4) << i;
		std::string filename = path + "/" + ostr.str() + ".png";
		cv::imwrite(filename, texture_imgs[i]);
	}
}

void write_pile(std::string filename, std::string filename_labeling, Model mesh, std::vector<int>& labeling) {

	std::ofstream out(filename.c_str(), std::ios::binary);
	if (!out.good()) {
		LOGE("piling failed");
		return;
	}

	std::ofstream out_label(filename_labeling.c_str(), std::ios::binary);
	if (!out_label.good()) {
		LOGE("piling failed");
		return;
	}

	std::vector<Eigen::Vector3f> const & mesh_vertices = mesh.vertices;
	std::vector<Eigen::Vector3f> const & mesh_normals = mesh.normals;
	std::vector<int> const & mesh_faces = mesh.faces;
	std::vector<int> mesh_vertices_color;
	mesh_vertices_color.resize(mesh_vertices.size());

	size_t num_faces = mesh_faces.size() / 10;

	Eigen::Vector3f palette[100] = {
		Eigen::Vector3f(0 ,0 ,0) / 255.0f, //black
		Eigen::Vector3f(255 ,0 ,0) / 255.0f, //red
		Eigen::Vector3f(0 ,255 ,0) / 255.0f, //green
		Eigen::Vector3f(0 ,0 ,255) / 255.0f, //blue
		Eigen::Vector3f(255 ,255 ,0) / 255.0f, //yellow
		Eigen::Vector3f(255 ,0 ,255) / 255.0f, //pink
		Eigen::Vector3f(0 ,255 ,255) / 255.0f, //cyan
		Eigen::Vector3f(255 ,255 ,255) / 255.0f, //white
	};
	for (int i = 0, ii = 0; i < num_faces; ++i, ii += 10) {
		for (int j = 0; j < 3; ++j) {
			mesh_vertices_color[mesh_faces[ii + j]] = labeling[i];
		}
	}
	for (int i = 0; i < mesh_vertices.size(); ++i) {
		out << "v ";
		out << mesh_vertices[i][0] << " " << mesh_vertices[i][1] << " " << mesh_vertices[i][2];
		out << " " << palette[mesh_vertices_color[i]][0] << " " << palette[mesh_vertices_color[i]][1] << " " << palette[mesh_vertices_color[i]][2];
		out << std::endl;
	}
	for (int i = 0; i < mesh_normals.size(); ++i) {
		out << "vn ";
		out << mesh_normals[i][0] << " " << mesh_normals[i][1] << " " << mesh_normals[i][2] << std::endl;
	}
	for (int i = 0, ii = 0; i < num_faces; ++i, ii += 10) {
		out << "f ";
		out << mesh_faces[ii + 0] + 1 << "/" << mesh_faces[ii + 0] + 1 << " ";
		out << mesh_faces[ii + 1] + 1 << "/" << mesh_faces[ii + 1] + 1 << " ";
		out << mesh_faces[ii + 2] + 1 << "/" << mesh_faces[ii + 2] + 1;
		out << std::endl;
		out_label << labeling[i] << std::endl;

	}
	out.close();
	out_label.close();
}