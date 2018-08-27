#include "helper.h"

#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <io.h>
#include <queue>


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
			

			loop(i, 3){ // 3d points index
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
			model.faces.insert(model.faces.end(), one_face.begin(), one_face.end());
			//if (space != std::string::npos && !line.empty()) { //label(optional)
			//	int label = std::stoi(line);
			//	model.faces_label.push_back(label);
			//}
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
	std::unordered_map<size_t, int>& edge_map, Adj_list& adj_list)
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

// build face adjecent list
int build_adj_list(const Model& model, Adj_list& adj_list) {
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
	
#if 1
	int count_greater3 = 0, count_less1 = 0;
	loop(i, adj_list.size()) {
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

// find connected faces which are of the same label
int find_connected_area(const Adj_list& adj_list, const std::vector<int>& label,
	std::vector<CArea>& area_list/*, std::vector<int>& patch*/) {

	area_list.clear();
	if (adj_list.size() == 0) {
		LOGE("adj_list is empty!");
		return -1;
	}
	if (adj_list.size() != label.size()) {
		LOGE("Incorrect size. adj_list.size(): %d,  label.size(): %d", adj_list.size(), label.size());
		return -1;
	}
	std::vector<int> flag(adj_list.size(), 0);
	std::queue<int> bfs_queue;
	loop(i, adj_list.size()) {
		if (flag[i] == 1) continue;
		// a new area
		CArea current_area;

		// bfs search
		bfs_queue.push(i);
		flag[i] = 1;
		const int& current_label = label[i];
		current_area.faces.push_back(i);
		current_area.label = current_label;
		

		while (!bfs_queue.empty()) {
			int current_face = bfs_queue.front();
			bfs_queue.pop();
			loop(j, adj_list[current_face].size()) {
				const int &adj_face_index = adj_list[current_face][j];
				if (flag[adj_face_index] == 0 && label[adj_face_index] == current_label) {
					bfs_queue.push(adj_face_index);
					flag[adj_face_index] = 1;
					current_area.faces.push_back(adj_face_index);
				}
			}
		}
		area_list.push_back(current_area);
	}
	//patch.clear();
	//patch.resize(label.size());
	//loop(i, area_list.size()) {
	//	loop(j, area_list[i].faces.size()) {
	//		const int& face = area_list[i].faces[j];
	//		patch[face] = i;
	//	}
	//}
	//std::sort(area_list.begin(), area_list.end(),
	//	[](CArea& v1, CArea& v2) {v1.faces.size() > v2.faces.size(); });
	return 0;
}

// change the data structure, fastly search which patch a face belong
void generate_patch_list(const std::vector<CArea>& area_list, int face_num, std::vector<int>& patch) {
	patch.clear();
	patch.resize(face_num);
	loop(i, area_list.size()) {
		loop(j, area_list[i].faces.size()) {
			patch[area_list[i].faces[j]] = i;
		}
	}
}


void vertex_info_map_to_vec(const std::unordered_multimap<int, VInfo>& vertex_info_list,
	std::vector<VInfo> &vertex_infos) {
	vertex_infos.clear();
	vertex_infos.resize(vertex_info_list.size());
	int index = 0;
	for (auto it = vertex_info_list.begin(); it != vertex_info_list.end(); ++it) {
		vertex_infos[index] = it->second;
		++index;
	}
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
	//for (auto it = pages.begin(); it != pages.end(); ++it) {

	//}
	loop(p, pages.size()) {
		std::cout << "page = " << p << std::endl;
		out << "newmtl m" << p << std::endl;
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

void write_texture(std::string path, const std::vector<cv::Mat>& texture_imgs) {
	loop(i, texture_imgs.size()) {
		std::string filename = path + "/" + std::to_string(i) + ".png";
		cv::imwrite(filename, texture_imgs[i]);
	}
}