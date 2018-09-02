
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <unordered_map>

#include "texture.h"

// find connected faces which are of the same label
int Texture_generator::find_connected_area(const Adj_face_list& adj_face_list, const std::vector<int>& label) {

	connected_areas.clear();
	if (adj_face_list.size() == 0) {
		LOGE("adj_face_list is empty!");
		return -1;
	}
	if (adj_face_list.size() != label.size()) {
		LOGE("Incorrect size. adj_face_list.size(): %d,  label.size(): %d", adj_face_list.size(), label.size());
		return -1;
	}
	std::vector<int> flag(adj_face_list.size(), 0);
	std::queue<int> bfs_queue;
	tex_loop(i, adj_face_list.size()) {
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
			tex_loop(j, adj_face_list[current_face].size()) {
				const int &adj_face_index = adj_face_list[current_face][j];
				if (flag[adj_face_index] == 0 && label[adj_face_index] == current_label) {
					bfs_queue.push(adj_face_index);
					flag[adj_face_index] = 1;
					current_area.faces.push_back(adj_face_index);
				}
			}
		}
		connected_areas.push_back(current_area);
	}
	patches_num = connected_areas.size();
	return 0;
}


// for 3 channel color img (CV_8U3C)
cv::Vec3f get_color(const cv::Mat& img, float x, float y) {
	int x0 = floor(x), x1 = x0 + 1;
	int y0 = floor(y), y1 = y0 + 1;
	if (x0 < 0 || x0 > img.cols || y0 < 0 || y0 > img.rows) {
		LOGW("uv coordinate out of image range. x = %f, y = %f", x, y);
		return cv::Vec3f(0, 0, 0);
	}
	cv::Vec3f color;
	if  (x1 <= img.cols - 1 && y1 <= img.rows - 1) {
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y0, x0)) * (y1 - y) * (x1 - x);
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y0, x1)) * (y1 - y) * (x - x0);
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y1, x0)) * (y - y0) * (x1 - x);
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y1, x1)) * (y - y0) * (x - x0);
	}
	else if (x1 > img.cols - 1 && y1 <= img.rows - 1) {
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y0, img.cols - 1)) * (y1 - y);
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(y1, img.cols - 1)) * (y - y0);
	}
	else if (x1 <= img.cols - 1 && y1 > img.rows - 1) {
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(img.rows - 1, x0)) * (x1 - x);
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(img.rows - 1, x1)) * (x - x0);
	}
	else {
		color += static_cast<cv::Vec3f>(img.at<cv::Vec3b>(img.rows - 1, img.cols - 1));
	}

	return color;
}


void Texture_generator::vertices_prepare(const std::vector<View>& views, const Model& model, const std::vector<int>& label)
{
	// unorder adjacent list of vertex in the every label seperately
	std::unordered_multimap<int, VInfo> info_map;
	for (int i = 0, ii = 0; ii < model.faces.size(); ++i, ii += 10) {

		////sort
		int v0 = model.faces[ii];
		int v1 = model.faces[ii + 1];
		int v2 = model.faces[ii + 2];

		int v[3] = { v0, v1, v2 };

		tex_loop(j, 3) {
			bool current_label_exist = false;
			auto vertex_it = info_map.equal_range(v[j]);
			const Eigen::Vector3f& p = model.vertices[v[j]];
			for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
				if (it->second.label == label[i]) {
					current_label_exist = true;
				}
			}
			if (!current_label_exist) {
				info_map.emplace(v[j], VInfo(label[i], v[j]));
				auto vertex_it = info_map.equal_range(v[j]);
				for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
					if (it->second.label == label[i]) {

						//insert vertex color
						if (label[i] > 0) {
							const Eigen::Matrix3f& R = views[label[i] - 1].R;
							const Eigen::Vector3f& t = views[label[i] - 1].t;
							const Eigen::Matrix3f& K = views[label[i] - 1].K;
							const cv::Mat& img = views[label[i] - 1].img;
							Eigen::Vector3f p_cam = R * p + t;
							Eigen::Vector3f p_img = K * p_cam;
							p_img /= p_cam(2);
							it->second.x = p_img(0);
							it->second.y = p_img(1);
							it->second.color = get_color(img, p_img(0), p_img(1));
						}
						else {
							it->second.x = 0;
							it->second.y = 0;
							it->second.color = cv::Vec3f(0, 0, 0);
						}
						break;
					}
				}
			}
			for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
				if (it->second.label == label[i]) {

					// insert vertex adj list
					tex_loop(k, 3) {
						if(v[k] != v[j]) it->second.adj_vertices.insert(v[k]);
					}
					break;
				}
			}
		}
	}

	// initialize sub_vertex_table
	sub_vertex_table.resize(model.vertices.size());
	int index = 0;

	for (auto it = info_map.begin(); it != info_map.end(); ++it) {
		const int& vertex = it->first;
		const int& label = it->second.label;
		std::map<int, int> &label_index_map = sub_vertex_table[vertex];
		label_index_map[label] = index;
		++index;
	}
	sub_vertices_num = index;

	// initialize vertex_infos
	vertex_infos.resize(info_map.size());
	index = 0;
	for (auto it = info_map.begin(); it != info_map.end(); ++it) {
		vertex_infos[index] = it->second;
		++index;
	}
	
#ifdef NEW_SEAMLESS
	// use weighted average color as vertex color
	tex_loop(i, vertex_infos.size()) {
		const int &v_index = vertex_infos[i].v_index;
		const std::map<int, int>& map_vertex = sub_vertex_table[v_index];
		if (map_vertex.size() < 2) {
			vertex_infos[i].color_ave = vertex_infos[i].color;
			continue;
		}
		const int &vertex_label = vertex_infos[i].label;
		cv::Vec3f color_ave = vertex_infos[i].color;
		const std::set<int>& adj_vertices = vertex_infos[i].adj_vertices;
		float edge_sample_weight = 1;
		for (auto it = adj_vertices.begin(); it != adj_vertices.end(); ++it) {
			if (this->sub_vertex_table[*it].size() > 1 ) { // adj vertices which are on patch edges.
				const int& sub_vertex = sub_vertex_table[*it].find(vertex_label)->second;
				const VInfo &vinfo = this->vertex_infos[sub_vertex];
				const float &x0 = vertex_infos[i].x;
				const float &x1 = vinfo.x;
				const float &y0 = vertex_infos[i].y;
				const float &y1 = vinfo.y;

				float d0 = sqrt((x0 - x1) * (x0 - x1) +
					(vertex_infos[i].y - vinfo.y) * (vertex_infos[i].y - vinfo.y));
				float dx = 0.5 * (x1 - x0) / d0;
				float dy = 0.5 * (y1 - y0) / d0;
				float dweight = 0.5 / d0;

				for (float x = x0 + dx, y = y0 + dy, weight = 1.0f - dweight;
					weight > 0 && (x - x1) * dx < 0 && (y - y1) * dy < 0;
					x += dx, y += dy, weight -= dweight)
				{
					color_ave += get_color(views[vertex_label - 1].img, x, y) * weight;
					edge_sample_weight += weight;
				}
			}
		}
		color_ave /= edge_sample_weight;
		vertex_infos[i].color_ave = color_ave;
	}
#endif

}

void Texture_generator::calculate_bboxes(const Model& model) {
	patch_bboxes.clear();
	tex_loop(i, connected_areas.size()) {
		const int &label = connected_areas[i].label;
		float maxx = -1, minx = 1e9, maxy = -1, miny = 1e9;
		tex_loop(j, connected_areas[i].faces.size()) {
			const int& face = connected_areas[i].faces[j];
			tex_loop(k, 3) {
				const int &vertex = model.faces[face * 10 + k];
				const int &sub_vertex = sub_vertex_table[vertex].find(label)->second;
				const VInfo& vinfo = vertex_infos[sub_vertex];
				if (vinfo.x > maxx) maxx = vinfo.x;
				if (vinfo.x < minx) minx = vinfo.x;
				if (vinfo.y > maxy) maxy = vinfo.y;
				if (vinfo.y < miny) miny = vinfo.y;
			}
		}
		
		BBox bbox;
		bbox.patch_index = i;
		// dilate 1 to avoid rendering problems
		bbox.bb[0] = std::floor(minx) - 1;
		bbox.bb[1] = std::floor(miny) - 1;
		bbox.bb[2] = std::ceil(maxx) - bbox.bb[0] + 1;
		bbox.bb[3] = std::ceil(maxy) - bbox.bb[1] + 1;

		patch_bboxes.push_back(bbox);
	}
}


void Texture_generator::color_adjustment_for_vertices(const Model& model, const std::vector<View>& views) {

	delta_color.clear();

	//prepare g_rows indices
	float lambda = 0.1f;

	int g_rows = sub_vertices_num;
	std::vector<Eigen::Triplet<double>> T_Gamma;
	int Gamma_rows = 0;

	std::vector<Eigen::Triplet<double>> T_A;
	int A_rows = 0;
	std::vector<std::vector<float>> V_f(3); // 3 channels

	// load matrices
	tex_loop(i, this->vertex_infos.size()) {
		const int& label = vertex_infos[i].label;
		if (label > 0) {

			//Load matrix Gamma;
			const std::set<int>& adj_set = vertex_infos[i].adj_vertices;
			for (auto adj_it = adj_set.begin(); adj_it != adj_set.end(); ++adj_it) {
				int j = sub_vertex_table[*adj_it].find(label)->second;
				if (i < j) {
					T_Gamma.emplace_back(Gamma_rows, i, lambda);
					T_Gamma.emplace_back(Gamma_rows, j, -lambda);
					++Gamma_rows;
				}
			}

			// Load matrix A;
			const int &v_index = vertex_infos[i].v_index;
			const std::map<int, int>& map_vertex = sub_vertex_table[v_index];
			for (auto it = map_vertex.begin(); it != map_vertex.end(); ++it) {
				const int& label2 = it->first;
				if (label != label2 && label2 != 0) {
					int j = it->second;
					T_A.emplace_back(A_rows, i, 1.0f);
					T_A.emplace_back(A_rows, j, -1.0f);

					// Load Vector f;
#ifdef NEW_SEAMLESS
					const cv::Vec3f &color_label = vertex_infos[i].color_ave;
					const cv::Vec3f &color_label2 = vertex_infos[j].color_ave;
#else
					const cv::Vec3f &color_label = vertex_infos[i].color;
					const cv::Vec3f &color_label2 = vertex_infos[j].color;
#endif
					tex_loop(k, 3) {
						V_f[k].push_back(color_label2(k) - color_label(k));
					}
					++A_rows;
				}
			}

		}
	}

	Eigen::SparseMatrix<float> S_Gamma(Gamma_rows, g_rows);
	S_Gamma.setFromTriplets(T_Gamma.begin(), T_Gamma.end());

	Eigen::SparseMatrix<float> S_A(A_rows, g_rows);
	S_A.setFromTriplets(T_A.begin(), T_A.end());

	std::vector<Eigen::VectorXf> f(3); // 3 channels
	tex_loop(k, 3) {
		f[k].resize(A_rows);
		tex_loop(i, A_rows) {
			f[k][i] = V_f[k][i];
		}
	}

	// A.transpose() * g = b;
	Eigen::SparseMatrix<float> Gamma = S_Gamma.transpose() * S_Gamma + S_A.transpose() * S_A;
	std::vector<Eigen::VectorXf> b(3); // 3 channels
	tex_loop(k, 3) {
		b[k] = S_A.transpose() * f[k];
	}

	// Solve
	Eigen::ConjugateGradient<Eigen::SparseMatrix<float>> solver;
	solver.setMaxIterations(1000);
	solver.setTolerance(0.0001);
	solver.compute(Gamma);

	std::vector<Eigen::VectorXf> g(3); // 3 channels
	tex_loop(k, 3) {
		g[k].resize(g_rows);
		g[k] = solver.solve(b[k]);
	}

	//output
	delta_color.resize(g_rows);
	tex_loop(i, g_rows) {
		delta_color[i] = cv::Vec3f(g[0][i], g[1][i], g[2][i]);
	}
}


int Texture_generator::patch_arrange() {
	texture_map.clear();
	texture_map.resize(patch_bboxes.size());
	std::vector<BBox> bboxes(patch_bboxes);
	std::sort(bboxes.begin(), bboxes.end(),
		[](BBox& b1, BBox& b2) { return b1.bb[2] * b1.bb[3] > b2.bb[2] * b2.bb[3]; });
	int current_page = 0, current_x = 0, current_y = 0, row_maxy = 0;
	tex_loop(i, bboxes.size()) {
		const int& patch_index = bboxes[i].patch_index;
		const int& bbx = bboxes[i].bb[0];
		const int& bby = bboxes[i].bb[1];
		const int& bbw = bboxes[i].bb[2];
		const int& bbh = bboxes[i].bb[3];
		if (bbw > texture_width || bbh > texture_height) {
			LOGE("texture size too small. patch size: %dx%d  texture size: %dx%d",
				bbh, bbw, texture_height, texture_width);
			return -1;
		}
		if (bbh > texture_height - current_y) {
			current_page++;
			current_x = 0;
			current_y = 0;
			row_maxy = 0;
		}
		if (bbw > texture_width - current_x) {
			current_x = 0;
			current_y += row_maxy;
			row_maxy = 0;
		}
		if (bbh > texture_height - current_y) {
			current_page++;
			current_x = 0;
			current_y = 0;
			row_maxy = 0;
		}
		TMap tmap;
		tmap.texture_page = current_page;
		tmap.x = current_x - bbx;
		tmap.y = current_y - bby;
		texture_map[patch_index] = tmap;

		current_x += bbw;
		if (bbh > row_maxy) row_maxy = bbh;
	}
	return current_page + 1;
}

// generate model.texcoord
void Texture_generator::generate_texcoord(const Model& src_model, Model& dst_model) {
	dst_model.vertices = src_model.vertices;
	dst_model.normals = src_model.normals;
	dst_model.faces = src_model.faces;
	dst_model.texcoords.clear();
	std::vector<Eigen::Vector2f> &texcoords = dst_model.texcoords;
	std::vector<int> &faces = dst_model.faces;

	int texcoord_index = 0;
	tex_loop(i, connected_areas.size()) {
		const int &label = connected_areas[i].label;
		tex_loop(j, connected_areas[i].faces.size()) {
			int face = connected_areas[i].faces[j] * 10;
			tex_loop(k, 3) {
				const int &vertex = faces[face + k];
				const int &vertex_index = sub_vertex_table[vertex].find(label)->second;
				float u = (vertex_infos[vertex_index].x + texture_map[i].x) / (texture_width - 0);
				float v = (vertex_infos[vertex_index].y + texture_map[i].y) / (texture_height - 0);
				texcoords.emplace_back(u, 1-v);
				faces[face + k + 6] = texcoord_index;
				++texcoord_index;
			}
			faces[face + 9] = texture_map[i].texture_page;
		}
	}
}


//color_adjusetment & generate_texture_image
void Texture_generator::generate_texture(const Model& model, const std::vector<View> &views, int texture_num,
	std::vector<cv::Mat>& texture_imgs) {

	texture_imgs.clear();
	tex_loop(i, texture_num) {
		texture_imgs.emplace_back(texture_height, texture_width, CV_8UC3);
	}

	tex_loop(i, connected_areas.size()) {
		const int& label = connected_areas[i].label;
		if (label == 0) continue;
		//imwrite("test.png", texture_imgs[0]);
		tex_loop(j, connected_areas[i].faces.size()) {
			const int& face = connected_areas[i].faces[j];
			
			// search vertex info & delta_color
			int v[3];
			VInfo vinfo[3];
			cv::Vec3f d_color[3];
			tex_loop(k, 3) {
				v[k] = model.faces[face * 10 + k];
				const int &index = sub_vertex_table[v[k]].find(label)->second;
				vinfo[k] = vertex_infos[index];
				d_color[k] = delta_color[index];
			}
			
			//1. find face bbox
			float maxx = -1, maxy = -1, minx = 1e9, miny = 1e9;
			tex_loop(k, 3) {
				if (maxx < vinfo[k].x) maxx = vinfo[k].x;
				if (minx > vinfo[k].x) minx = vinfo[k].x;
				if (maxy < vinfo[k].y) maxy = vinfo[k].y;
				if (miny > vinfo[k].y) miny = vinfo[k].y;
			}
			if (maxx >= views[label - 1].img.cols) continue;// maxx = views[label - 1].img.cols - 1;
			if (maxx < 0) continue;// maxx = 0;
			if (maxy >= views[label - 1].img.rows) continue;// maxy = views[label - 1].img.rows - 1;
			if (maxy < 0) continue;// maxy = 0;
			if (minx >= views[label - 1].img.cols) continue;// minx = views[label - 1].img.cols - 1;
			if (minx < 0) continue;// minx = 0;
			if (miny >= views[label - 1].img.rows) continue;// miny = views[label - 1].img.rows - 1;
			if (miny < 0) continue;// miny = 0;

			// dilate 1 to avoid rendering problems
			int face_bbox[4] = { std::floor(minx) - 1, std::floor(miny) - 1,
				std::ceil(maxx) - std::floor(minx) + 2, std::ceil(maxy) - std::floor(miny) + 2 };
			if (face_bbox[2] == 0 || face_bbox[3] == 0)
				continue;

			//2. bbox "in triangle" mask: 
			// use begin&end pixel of each row to represent mask. 
			std::vector<std::pair<int, int>> begin_end_vec(face_bbox[3]);
			{
				std::vector<std::pair<int, int>> begin_end_vec_raw(face_bbox[3] + 2);

				VInfo v0 = vinfo[0];
				VInfo v1 = vinfo[1];
				VInfo v2 = vinfo[2];

				if (v0.y > v1.y) std::swap(v0, v1);
				if (v1.y > v2.y) std::swap(v1, v2);
				if (v0.y > v1.y) std::swap(v0, v1);

				//general form equation of an edge v0, v2
				float A1 = v0.y - v2.y;
				float B1 = v2.x - v0.x;
				float C1 = v0.x * v2.y - v2.x * v0.y;

				if (A1 == 0.0f) continue;

				Eigen::Vector3f L1(A1, -B1, -C1);
				L1 /= A1;
				float x0 = L1[1] * v1.y + L1[2];

				Eigen::Vector3f L0(v1.y - v2.y, -(v2.x - v1.x), -(v1.x * v2.y - v2.x * v1.y));
				L0 /= L0[0];

				Eigen::Vector3f L2(v1.y - v0.y, -(v0.x - v1.x), -(v1.x * v0.y - v0.x * v1.y));
				L2 /= L2[0];

				for (int y = face_bbox[1]; y < v1.y; ++y) {
					float begin = L1[1] * y + L1[2];
					float end = L2[1] * y + L2[2];
					if (x0 > v1.x) std::swap(begin, end);
					begin = std::max(begin, static_cast<float>(face_bbox[0]));
					end = std::min(end, static_cast<float>(face_bbox[0] + face_bbox[2]));
					begin_end_vec_raw[y - face_bbox[1] + 1]
						= std::pair<int, int>(std::floor(begin) - 1, std::ceil(end) + 1); // x dilation
				}
				for (int y = std::ceil(v1.y); y < face_bbox[1] + face_bbox[3]; ++y) {
					float begin = L1[1] * y + L1[2];
					float end = L0[1] * y + L0[2];
					if (x0 > v1.x) std::swap(begin, end);
					begin = std::max(begin, static_cast<float>(face_bbox[0]));
					end = std::min(end, static_cast<float>(face_bbox[0] + face_bbox[2]));
					begin_end_vec_raw[y - face_bbox[1] + 1]
						= std::pair<int, int>(std::floor(begin) - 1, std::ceil(end) + 1); // x dilation
				}
				// y dilation. to avoid black edge between patches
				begin_end_vec_raw[0] = std::pair<int, int>(INT_MAX, 0);
				begin_end_vec_raw[begin_end_vec_raw.size() - 1] = std::pair<int, int>(INT_MAX, 0);
				for (int s1 = 1; s1 < begin_end_vec_raw.size() - 1; ++s1) {
					int begin = INT_MAX;
					int end = 0;
					for (int s2 = -1; s2 <= 1; ++s2) {
						if (begin_end_vec_raw[s1 + s2].first < begin) begin = begin_end_vec_raw[s1 + s2].first;
						if (begin_end_vec_raw[s1 + s2].second > end) end = begin_end_vec_raw[s1 + s2].second;
					}
					begin_end_vec[s1 - 1] = std::pair<int, int>(begin, end);
				}

				tex_loop(k, begin_end_vec.size()) {
					begin_end_vec[k].first = std::max(begin_end_vec[k].first, face_bbox[0]);
					begin_end_vec[k].first = std::min(begin_end_vec[k].first, face_bbox[0] + face_bbox[2] - 1);
					begin_end_vec[k].second = std::max(begin_end_vec[k].second, face_bbox[0]);
					begin_end_vec[k].second = std::min(begin_end_vec[k].second, face_bbox[0] + face_bbox[2] - 1);
				}
			}

			//3. find delta_color plane for 3 channels 
			std::vector<Eigen::Vector4f> S(3);
			{
				const VInfo &v1 = vinfo[0];
				const VInfo &v2 = vinfo[1];
				const VInfo &v3 = vinfo[2];

				tex_loop(k, 3) {
					const float &x1 = v1.x, &y1 = v1.y, &z1 = d_color[0](k);
					const float &x2 = v2.x, &y2 = v2.y, &z2 = d_color[1](k);
					const float &x3 = v3.x, &y3 = v3.y, &z3 = d_color[2](k);

					//color plane equation
					Eigen::Matrix2f A_m; 
					A_m << y2 - y1, z2 - z1, y3 - y1, z3 - z1;
					float A = A_m.determinant();

					Eigen::Matrix2f B_m;
					B_m << z2 - z1, x2 - x1, z3 - z1, x3 - x1;
					float B = B_m.determinant();

					Eigen::Matrix2f C_m;
					C_m << x2 - x1, y2 - y1, x3 - x1, y3 - y1;
					float C = C_m.determinant();
					if (C == 0.0f) {
						LOGW("duplicate points& zero area face\n!");
						continue;
					}

					Eigen::Matrix3f D_m;
					D_m << -x1, -y1, -z1, 
						x2 - x1, y2 - y1, z2 - z1, 
						x3 - x1, y3 - y1, z3 - z1;
					float D = D_m.determinant();

					S[k] = Eigen::Vector4f(A, B, C, D);
					S[k] /= -C;
				}
			}

			//4. image copy(in triangle with color changed)
			{
				const cv::Mat &view_img = views[label - 1].img;
				const int &x0 = face_bbox[0], &y0 = face_bbox[1];
				const int &w = face_bbox[2], &h = face_bbox[3];
				int x1 = x0 + w, y1 = y0 + h;

				const TMap &tmap = texture_map[i];
				cv::Mat &texture_img = texture_imgs[tmap.texture_page];

				for (int y = y0; y < y1; ++y) {
					const int &begin = begin_end_vec[y - y0].first;
					const int &end = begin_end_vec[y - y0].second;
					const cv::Vec3b *p0 = view_img.ptr<cv::Vec3b>(y);
					cv::Vec3b* p1 = texture_img.ptr<cv::Vec3b>(tmap.y + y);
					for (int x = begin; x <= end; ++x) {
						const cv::Vec3b& pixel0 = p0[x];
						cv::Vec3b& pixel1 = p1[tmap.x + x];
						tex_loop(k, 3) {
							float c = static_cast<float>(pixel0[k]);
							c += S[k][0] * x + S[k][1] * y + S[k][3];
							c = std::min(std::max(c, 0.0f), 255.0f);
							pixel1[k] = c;
						}
					}
				}
			}
		}
	}
}