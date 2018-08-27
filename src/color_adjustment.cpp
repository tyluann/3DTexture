
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <unordered_map>

#include "color_adjustment.h"
#include "helper.h"


//struct Edge {
//	int v0;
//	int v1;
//	Edge(int _v0, int _v1) { v0 = _v0; v1 = _v1; }
//};

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


void vertices_prepare(const std::vector<View>& view, const Model& model, const std::vector<int>& label,
	/*const std::vector<int>& patch,*/ const std::vector<CArea>& area_list,
	std::unordered_multimap<int, VInfo> & vertex_info_list)
	/*std::unordered_multimap<int, VColor>& vertice_color*/
{

	// unorder adjacent list of vertex in the every label seperately
	std::unordered_multimap<int, VInfo>& info_map = vertex_info_list;
	for (int i = 0, ii = 0; ii < model.faces.size(); ++i, ii += 10) {

		////sort
		int v0 = model.faces[ii];
		int v1 = model.faces[ii + 1];
		int v2 = model.faces[ii + 2];
		//if (v0 > v1)std::swap(v0, v1);
		//if (v1 > v2)std::swap(v1, v2);
		//if (v0 > v1)std::swap(v0, v1);

		int v[3] = { v0, v1, v2 };
		
		
		loop(j, 3) {
			if (v[j] == 3096) {
				std::cout << std::endl;
			}
			bool current_label_exist = false;
			auto vertex_it = info_map.equal_range(v[j]);
			const Eigen::Vector3f& p = model.vertices[v[j]];
			for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
				if (it->second.label == label[i]) {
					current_label_exist = true;
				}
			}
			if (!current_label_exist) {
				info_map.emplace(v[j], VInfo(label[i]/*, patch[i]*/));
				auto vertex_it = info_map.equal_range(v[j]);
				for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
					if (it->second.label == label[i]) {

						//insert vertex color
						if (label[i] > 0) {
							const Eigen::Matrix3f& R = view[label[i] - 1].R;
							const Eigen::Vector3f& t = view[label[i] - 1].t;
							const Eigen::Matrix3f& K = view[label[i] - 1].K;
							const cv::Mat& img = view[label[i] - 1].img;
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
					loop(k, 3) {
						if(v[k] > v[j]) it->second.adj_vertices.insert(v[k]);
					}
					break;
				}
			}
			//if (!current_label_exist) {
			//	info_map.emplace(v[j], VInfo(label[i], patch[i]));
			//	auto vertex_it = info_map.equal_range(v[j]);
			//	loop(k, 3) {
			//		if (v[k] > v[j]) vertex_it.first->second.adj_vertices.insert(v[k]);
			//	}

			//	if (label[i] > 0) {
			//		const Eigen::Matrix3f& R = view[label[i] - 1].R;
			//		const Eigen::Vector3f& t = view[label[i] - 1].t;
			//		const Eigen::Matrix3f& K = view[label[i] - 1].K;
			//		const cv::Mat& img = view[label[i] - 1].img;
			//		Eigen::Vector3f p_cam = R * p + t;
			//		Eigen::Vector3f p_img = K * p_cam;
			//		p_img /= p_cam(2);
			//		vertex_it.first->second.color = get_color(img, p_img(0), p_img(1));
			//	}
			//	else {
			//		vertex_it.first->second.color = cv::Vec3f(0, 0, 0);
			//	}
			//}
		}
	}
}

void calculate_bboxes(const std::vector<CArea>& connected_area_list, const Model& model,
	const std::unordered_multimap<int, VInfo> & vertex_info_list, std::vector<BBox>& patch_bboxes) {
	patch_bboxes.clear();
	loop(i, connected_area_list.size()) {
		const int &label = connected_area_list[i].label;
		float maxx = -1, minx = 1e9, maxy = -1, miny = 1e9;
		loop(j, connected_area_list[i].faces.size()) {
			const int& face = connected_area_list[i].faces[j];
			loop(k, 3) {
				const int &vertex = model.faces[face * 10 + k];
				auto vertex_it = vertex_info_list.equal_range(vertex);
				for (auto it = vertex_it.first; it != vertex_it.second; ++it) {
					if (it->second.label == label) {
						if (it->second.x > maxx) maxx = it->second.x;
						if (it->second.x < minx) minx = it->second.x;
						if (it->second.y > maxy) maxy = it->second.y;
						if (it->second.y < miny) miny = it->second.y;
					}
				}
			}
		}
		
		BBox bbox;
		bbox.patch_index = i;
		bbox.bb[0] = std::floor(minx);
		bbox.bb[1] = std::floor(miny);
		bbox.bb[2] = std::ceil(maxx) - bbox.bb[0] + 1;
		bbox.bb[3] = std::ceil(maxy) - bbox.bb[1] + 1;

		patch_bboxes.push_back(bbox);

#if 1
		if (maxx < 0) {
			LOGE("something wrong.");
		}

#endif
	}
}


void color_adjustment_for_vertices(const std::unordered_multimap<int, VInfo> & vertex_info_list,
	const Model& model, std::vector<std::map<int, int>>& index_table,
	std::vector<cv::Vec3f> &delta_color) {

	delta_color.clear();
	index_table.clear();

	//prepare g_rows indices

	index_table.resize(model.vertices.size());
	int index = 0;
	

	for (auto it = vertex_info_list.begin(); it != vertex_info_list.end(); ++it) {
		const int& vertex = it->first;
		const int& label = it->second.label;
		std::map<int, int> &label_index_map = index_table[vertex];
		label_index_map[label] = index;
		++index;
	}

	float lambda = 0.1f;

	int g_rows = vertex_info_list.size();
	std::vector<Eigen::Triplet<double>> T_Gamma;
	int Gamma_rows = 0;

	std::vector<Eigen::Triplet<double>> T_A;
	int A_rows = 0;
	std::vector<std::vector<float>> V_f(3); // 3 channels

	// load matrices
	loop(i, model.vertices.size()) {
		auto vertex = vertex_info_list.equal_range(i);
		for (auto it = vertex.first; it != vertex.second; ++it) {
			const int& label = it->second.label;
			int &index_i = index_table[i].find(label)->second;
			if (label > 0) {

				//Load matrix Gamma;
				const std::set<int>& adj_set = it->second.adj_vertices;
				for (auto adj_it = adj_set.begin(); adj_it != adj_set.end(); ++adj_it) {
					int index_j = index_table[*adj_it].find(label)->second;
					T_Gamma.emplace_back(Gamma_rows, index_i, lambda);
					T_Gamma.emplace_back(Gamma_rows, index_j, -lambda);
					++Gamma_rows;
				}

				// Load matrix A;
				for (auto it2 = it; it2 != vertex.second; ++it2) {
					const int& label2 = it2->second.label;
					if (label != label2 && label2 != 0) {
						int index_j = index_table[i].find(label2)->second;
						T_A.emplace_back(A_rows, index_i, 1.0f);
						T_A.emplace_back(A_rows, index_j, -1.0f);

						// Load Vector f;
						const cv::Vec3f &color_label = it->second.color;
						const cv::Vec3f &color_label2 = it2->second.color;
						loop(k, 3) {
							V_f[k].push_back(color_label2(k) - color_label(k));
						}
						++A_rows;
					}
				}

			}
		}
	}

	Eigen::SparseMatrix<float> S_Gamma(Gamma_rows, g_rows);
	S_Gamma.setFromTriplets(T_Gamma.begin(), T_Gamma.end());

	Eigen::SparseMatrix<float> S_A(A_rows, g_rows);
	S_A.setFromTriplets(T_A.begin(), T_A.end());

	std::vector<Eigen::VectorXf> f(3); // 3 channels
	loop(k, 3) {
		f[k].resize(A_rows);
		loop(i, A_rows) {
			f[k][i] = V_f[k][i];
		}
	}

	// A.transpose() * g = b;
	Eigen::SparseMatrix<float> Gamma = S_Gamma.transpose() * S_Gamma + S_A.transpose() * S_A;
	std::vector<Eigen::VectorXf> b(3); // 3 channels
	loop(k, 3) {
		b[k] = S_A.transpose() * f[k];
	}

	// Solve
	Eigen::ConjugateGradient<Eigen::SparseMatrix<float>> solver;
	solver.setMaxIterations(1000);
	solver.setTolerance(0.0001);
	solver.compute(Gamma);

	std::vector<Eigen::VectorXf> g(3); // 3 channels
	loop(k, 3) {
		g[k].resize(g_rows);
		g[k] = solver.solve(b[k]);
	}

	//output
	delta_color.resize(g_rows);
	loop(i, g_rows) {
		delta_color[i] = cv::Vec3f(g[0][i], g[1][i], g[2][i]);
	}
}

// change data structure of vertex_info_list
void re_construct(const std::unordered_multimap<int, VInfo>& vertex_info_list,
	const Model& model, std::vector<VInfo>& new_vertex_info) {
	new_vertex_info.clear();
	new_vertex_info.resize(vertex_info_list.size());
	int index = 0;
	for (auto it = vertex_info_list.begin(); it != vertex_info_list.end(); ++it) {
		new_vertex_info[index] = it->second;
		++index;
	}
}

// arrrange the location of patches on the final texture image(s).
int patch_arrange(const std::vector<BBox>& patch_bboxes, int texture_width, int texture_height,
	std::vector<TMap>& texture_map, std::vector<int>& patch_page) {
	texture_map.clear();
	texture_map.resize(patch_bboxes.size());
	patch_page.clear();
	patch_page.resize(patch_bboxes.size());
	std::vector<BBox> bboxes(patch_bboxes);
	std::sort(bboxes.begin(), bboxes.end(),
		[](BBox& b1, BBox& b2) { return b1.bb[2] * b1.bb[3] > b2.bb[2] * b2.bb[3]; });
	int current_page = 0, current_x = 0, current_y = 0, row_maxy = 0;
	loop(i, bboxes.size()) {
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
		tmap.texture_num = current_page;
		tmap.x = current_x - bbx;
		tmap.y = current_y - bby;
		texture_map[patch_index] = tmap;
		patch_page[i] = current_page;

		current_x += bbw;
		if (bbh > row_maxy) row_maxy = bbh;
	}
	return current_page + 1;
}

// generate model.texcoord
void generate_texcoord(const Model& src_model, const std::vector<VInfo> &vertex_infos,
	const std::vector<std::map<int, int>>& index_table, int texture_width, int texture_height,
	const std::vector<TMap>& texture_map, const std::vector<int>& patch_page, 
	const std::vector<CArea>& area_list, Model& dst_model) {
	dst_model.vertices = src_model.vertices;
	dst_model.normals = src_model.normals;
	dst_model.faces = src_model.faces;
	dst_model.texcoords.clear();
	std::vector<Eigen::Vector2f> &texcoords = dst_model.texcoords;
	std::vector<int> &faces = dst_model.faces;

	//loop(i, vertex_infos.size()) {
	//	float u = (vertex_infos[i].x + texture_map[i].x) / texture_width;
	//	float v = (vertex_infos[i].y + texture_map[i].y) / texture_height;
	//	texcoords.emplace_back(u, v);
	//}
	int texcoord_index = 0;
	loop(i, area_list.size()) {
		const int &label = area_list[i].label;
		loop(j, area_list[i].faces.size()) {
			int face = area_list[i].faces[j] * 10;
			loop(k, 3) {
				const int &vertex = faces[face + k];
				const int &vertex_index = index_table[vertex].find(label)->second;
				float u = (vertex_infos[vertex_index].x + texture_map[i].x) / (texture_width - 1);
				float v = (vertex_infos[vertex_index].y + texture_map[i].y) / (texture_height - 1);
				texcoords.emplace_back(u, 1-v);
				faces[face + k + 6] = texcoord_index;
				++texcoord_index;
			}
			faces[face + 9] = patch_page[i];
		}
	}
}


//color_adjusetment & generate_texture_image
void generate_texture(const std::vector<CArea>& connected_area_list, 
	const std::unordered_multimap<int, VInfo>& vertex_info_list, 
	const Model& model, const std::vector<std::map<int, int>>& index_table,
	const std::vector<cv::Vec3f> &delta_color, const std::vector<TMap>& texture_map,
	const std::vector<View> &views, int texture_width, int texture_height, int texture_num,
	std::vector<cv::Mat>& texture_imgs) {

	texture_imgs.clear();
	loop(i, texture_num) {
		texture_imgs.emplace_back(texture_height, texture_width, CV_8UC3);
	}

	// change data structure
	std::vector<VInfo> vertex_infos;
	vertex_infos.resize(vertex_info_list.size());
	int index = 0;
	for (auto it = vertex_info_list.begin(); it != vertex_info_list.end(); ++it) {
		vertex_infos[index] = it->second;
		++index;
	}

	loop(i, connected_area_list.size()) {
		const int& label = connected_area_list[i].label;
		if (label == 0) continue;
		//imwrite("test.png", texture_imgs[0]);
		loop(j, connected_area_list[i].faces.size()) {
			const int& face = connected_area_list[i].faces[j];
			
			// search vertex info & delta_color
			int v[3];
			VInfo vinfo[3];
			cv::Vec3f d_color[3];
			loop(k, 3) {
				v[k] = model.faces[face * 10 + k];
				const int &index = index_table[v[k]].find(label)->second;
				vinfo[k] = vertex_infos[index];
				d_color[k] = delta_color[index];
			}
			
			//1. find face bbox
			float maxx = -1, maxy = -1, minx = 1e9, miny = 1e9;
			loop(k, 3) {
				if (maxx < vinfo[k].x) maxx = vinfo[k].x;
				if (minx > vinfo[k].x) minx = vinfo[k].x;
				if (maxy < vinfo[k].y) maxy = vinfo[k].y;
				if (miny > vinfo[k].y) miny = vinfo[k].y;
			}
			int face_bbox[4] = { std::floor(minx), std::floor(miny),
				std::ceil(maxx) - std::floor(minx), std::ceil(maxy) - std::floor(miny) };
			if (face_bbox[2] == 0.0f || face_bbox[3] == 0.0f)
				continue;

			//2. bbox "in triangle" mask: 
			// use begin&end pixel of each row to represent mask. 
			std::vector<std::pair<int, int>> begin_end_vec(face_bbox[3]);

			{
				std::vector<std::pair<int, int>> begin_end_vec_raw(face_bbox[3]);

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
					begin_end_vec_raw[y - face_bbox[1]]
						= std::pair<int, int>(std::ceil(begin) - 1, std::floor(end) + 1); // x dilation
				}
				for (int y = std::ceil(v1.y); y < face_bbox[1] + face_bbox[3]; ++y) {
					float begin = L1[1] * y + L1[2];
					float end = L0[1] * y + L0[2];
					if (x0 > v1.x) std::swap(begin, end);
					begin = std::max(begin, static_cast<float>(face_bbox[0]));
					end = std::min(end, static_cast<float>(face_bbox[0] + face_bbox[2]));
					begin_end_vec_raw[y - face_bbox[1]] 
						= std::pair<int, int>(std::ceil(begin) - 1, std::floor(end) + 1); // x dilation
				}
				// y dilation. to avoid black edge between patches
				if (begin_end_vec_raw.size() <= 1) {
					begin_end_vec = begin_end_vec_raw;
				}
				else {
					for (int s = 1; s < begin_end_vec_raw.size() - 1; ++s) {
						begin_end_vec[s] = std::pair<int, int>(
							std::min(std::min(
								begin_end_vec_raw[s].first,
								begin_end_vec_raw[s - 1].first),
								begin_end_vec_raw[s + 1].first)
							,
							std::max(std::max(
								begin_end_vec_raw[s].second,
								begin_end_vec_raw[s - 1].second),
								begin_end_vec_raw[s + 1].second)
							);
					}

					begin_end_vec[0] = std::pair<int, int>(
						std::min(begin_end_vec_raw[0].first, begin_end_vec_raw[1].first),
						std::max(begin_end_vec_raw[0].second, begin_end_vec_raw[1].second));
					begin_end_vec[face_bbox[3] - 1] = std::pair<int, int>(
						std::min(begin_end_vec_raw[face_bbox[3] - 1].first,
							begin_end_vec_raw[face_bbox[3] - 2].first),
						std::max(begin_end_vec_raw[face_bbox[3] - 1].second,
							begin_end_vec_raw[face_bbox[3] - 2].second));
				}
			}

			//3. find delta_color plane for 3 channels 
			std::vector<Eigen::Vector4f> S(3);

			{
				// find the vertex of a triangle face which has the longest opposite side(name it v0)
				//auto square_distance = [](const VInfo &v1, const VInfo &v2)->float{
				//	return (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y);
				//};
				//float d0 = square_distance(vinfo[2], vinfo[1]);
				//float d1 = square_distance(vinfo[0], vinfo[2]);
				//float d2 = square_distance(vinfo[0], vinfo[1]);
				//
				//if (d0 < d1) {
				//	std::swap(vinfo[0], vinfo[1]);
				//	std::swap(d0, d1);
				//}
				//if (d0 < d2) {
				//	std::swap(vinfo[0], vinfo[2]);
				//	std::swap(d0, d2);
				//}
				const VInfo &v1 = vinfo[0];
				const VInfo &v2 = vinfo[1];
				const VInfo &v3 = vinfo[2];

				loop(k, 3) {
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
				cv::Mat &texture_img = texture_imgs[tmap.texture_num];

				for (int y = y0; y < y1; ++y) {
					const int &begin = begin_end_vec[y - y0].first;
					const int &end = begin_end_vec[y - y0].second;
					const cv::Vec3b *p0 = view_img.ptr<cv::Vec3b>(y);
					cv::Vec3b* p1 = texture_img.ptr<cv::Vec3b>(tmap.y + y);
					for (int x = begin; x <= end; ++x) {
						const cv::Vec3b& pixel0 = p0[x];
						cv::Vec3b& pixel1 = p1[tmap.x + x];
						loop(k, 3) {
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

