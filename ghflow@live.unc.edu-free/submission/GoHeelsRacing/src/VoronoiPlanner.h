// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>

typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef boost::polygon::segment_data<coordinate_type> segment_type;

#include <boost/graph/adjacency_list.hpp>

namespace boost
{
	enum vertex_coordinates_t
	{
		vertex_coordinates = 111
	}; // a unique id for the type tag
	BOOST_INSTALL_PROPERTY(vertex, coordinates);
}

/**
 * 
 */
class VoronoiPlanner
{
public:
	const std::vector<point_type> &GetPlan(const point_type &car_location, point_type milestone, const std::vector<segment_type> &Walls, double allowed_obs_dist);
	void GetRoadmapPoints(std::list<point_type> &points);
	void GetRoadmapSegments(std::vector<segment_type> &segments);

	std::vector<point_type> Plan;
	double allowed_obs_dist = 1.0; // in meters
	const double max_discretization_error = 0.1;

private:
	void MakeRoadmap(const point_type &car_location, const std::vector<segment_type> &Walls, double allowed_obs_dist);

	typedef boost::polygon::voronoi_diagram<coordinate_type> VD;
	typedef VD::cell_type cell_type;
	typedef VD::cell_type::source_index_type source_index_type;
	typedef VD::cell_type::source_category_type source_category_type;
	typedef VD::edge_type edge_type;
	typedef VD::cell_container_type cell_container_type;
	typedef VD::cell_container_type vertex_container_type;
	typedef VD::edge_container_type edge_container_type;
	typedef VD::const_cell_iterator const_cell_iterator;
	typedef VD::const_vertex_iterator const_vertex_iterator;
	typedef VD::const_edge_iterator const_edge_iterator;
	typedef VD::vertex_type vertex_type;

	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
																boost::property<boost::vertex_coordinates_t, point_type>,
																boost::property<boost::edge_weight_t, double>>
			Roadmap_t;
	typedef typename boost::graph_traits<VoronoiPlanner::Roadmap_t>::vertex_descriptor vertex_descriptor;
	typedef typename boost::graph_traits<VoronoiPlanner::Roadmap_t>::edge_descriptor edge_descriptor;
	typedef boost::property_map<Roadmap_t, boost::vertex_coordinates_t>::type coordinates_map_t; // The type of the mapping from a vertex descriptor to its coordiantes property
	typedef boost::property_map<Roadmap_t, boost::edge_weight_t>::type weight_map_t;

	Roadmap_t Roadmap;
	vertex_descriptor start_vertex;
	void add_start_vertex(const point_type &car_location);
	void add_finish_vertex();

	// Voronoi diagram processing
	point_type retrieve_endpoint(const cell_type &cell, const std::vector<segment_type> &Walls);
	segment_type retrieve_segment(const cell_type &cell, const std::vector<segment_type> &Walls);
	void sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge, const std::vector<segment_type> &Walls);
	void color_close_vertices(const VD &vd, const std::vector<segment_type> &Walls);
	void add_linear_edge(const edge_type &edge, std::unordered_map<const vertex_type *, vertex_descriptor> &voronoi_to_roadmap);
	void add_curved_edge(const edge_type &edge, std::unordered_map<const vertex_type *, vertex_descriptor> &voronoi_to_roadmap, const std::vector<segment_type> &Walls);
	vertex_descriptor add_roadmap_vertex(point_type point);
	edge_descriptor add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1);
	edge_descriptor add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1, double weight);
	vertex_descriptor get_closest_vertex(point_type point);
	VoronoiPlanner::edge_descriptor get_closest_edge(point_type point);
};
