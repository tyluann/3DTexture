
#include "texture.h"
#include <mapmap/full.h>
#include <acc/bvh_tree.h>
#include <fstream>
#include <algorithm>

int labelling(const Model &model, const Adj_face_list &adj_face_list, 
	const std::vector<View>& views, std::vector<int>& labels)
{
	Graph graph(model.faces.size() / 10, views.size());
	graph.calculate_data_cost(model, views);

	// mapMAP template parameters
	const unsigned int simd_w = mapmap::sys_max_simd_width<float>();

	// cost types
	using unary_t = mapmap::UnaryTable<float, simd_w>;
	using pairwise_t = mapmap::PairwisePotts<float, simd_w>;

	//data structures
	mapmap::Graph<float> map_graph(graph.faces_num);
	mapmap::LabelSet<float, simd_w> map_labels(graph.faces_num, 0);

	std::vector<unary_t> unaries;
	unaries.reserve(graph.faces_num);
	pairwise_t pairwise(1.0f);

	// termination criterion and control flow
	mapmap::StopWhenReturnsDiminish<float, simd_w> terminate(5, 0.01);
	mapmap::mapMAP_control ctr;

	mapmap::mapMAP<float, simd_w> map_solver;

	//if both faces for one edge are good face, add the edge to map_graph
	tex_loop(i, graph.faces_num){
		//if the face has no good view, ignore it
		if (!graph.b_good_face[i]) continue;
		std::vector<int> adj_faces_t = adj_face_list[i];
		auto it = std::unique(adj_faces_t.begin(), adj_faces_t.end());
		std::vector<int> adj_faces(adj_faces_t.begin(), it);
		tex_loop(j, adj_faces.size())
		{
			int adj_face_idx = adj_faces[j];
			if (!graph.b_good_face[adj_face_idx]) continue;
			if (i < adj_face_idx) map_graph.add_edge(i, adj_face_idx, 1.0f);
		}
	}
	map_graph.update_components();

	//set labels and costs
	tex_loop(i, graph.faces_num)
	{
		std::vector<mapmap::_iv_st<float, simd_w>> map_labels_tmp;
		std::vector<mapmap::_s_t<float, simd_w>> map_costs_tmp;
		if (!graph.b_good_face[i])
		{
			map_labels_tmp.push_back(0);
			map_costs_tmp.push_back(1.0f);
		}
		tex_loop(j, graph.views_num)
		{
			if (graph.valid_mask[i][j])
			{
				map_labels_tmp.push_back(j + 1);
				map_costs_tmp.push_back(graph.data_costs[i][j + 1]);
			}
		}
		map_labels.set_label_set_for_node(i, map_labels_tmp);
		unaries.emplace_back(i, &map_labels);
		unaries.back().set_costs(map_costs_tmp);
	}

	// create (optional) control flow settings
	ctr.use_multilevel = false;
	ctr.use_spanning_tree = true;
	ctr.use_acyclic = true;
	ctr.spanning_tree_multilevel_after_n_iterations = 5;
	ctr.force_acyclic = true;
	ctr.min_acyclic_iterations = 5;
	ctr.relax_acyclic_maximal = true;
	ctr.tree_algorithm = mapmap::LOCK_FREE_TREE_SAMPLER;

	// set to true and select a seed for (serial) deterministic sampling
	ctr.sample_deterministic = false;
	ctr.initial_seed = 548923723;

	// construct optimizer
	map_solver.set_graph(&map_graph);
	map_solver.set_label_set(&map_labels);
	tex_loop(i, graph.faces_num) {
		map_solver.set_unary(i, &unaries[i]);
	}
	map_solver.set_pairwise(&pairwise);
	map_solver.set_termination_criterion(&terminate);

	// use standard multilevel and termination criterion and start
	std::vector<mapmap::_iv_st<float, simd_w>> solution;

	auto display = [](const mapmap::luint_t time_ms,
		const mapmap::_iv_st<float, simd_w> objective) {
		//LOGI( "time_ms: %d    Energy: %d", time_ms, objective);
	};

	map_solver.set_logging_callback(display);
	map_solver.optimize(solution, ctr);

	//output label result to model;
	labels.clear();
	labels.resize(graph.faces_num, 0);
	tex_loop(i, graph.faces_num) {
		int label = map_labels.label_from_offset(i, solution[i]);
		if (label < 0 || label >= graph.views_num + 1) {
			LOGE("Incorrect label value");
			return -1;
		}
		labels[i] = label;
	}
	return 0;
}

