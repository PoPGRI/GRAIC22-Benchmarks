// Fill out your copyright notice in the Description page of Project Settings.

#include "VoronoiPlanner.h"

#include "voronoi_visual_utils.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

#include <unordered_map>

const std::vector<point_type> &VoronoiPlanner::GetPlan(const point_type &car_location, point_type milestone, const std::vector<segment_type> &Walls, double allowed_obs_dist)
{
	MakeRoadmap(car_location, Walls, allowed_obs_dist);
	std::cout << "Roadmap. " << std::flush;

	Plan.clear();
	// shortest paths from source
	std::vector<vertex_descriptor> pred(num_vertices(Roadmap));
	std::vector<double> distances(num_vertices(Roadmap));
	dijkstra_shortest_paths_no_color_map(Roadmap, start_vertex,
																			 predecessor_map(boost::make_iterator_property_map(pred.begin(), get(boost::vertex_index, Roadmap))).distance_map(boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, Roadmap))));

	vertex_descriptor vertex = get_closest_vertex(milestone), child;
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	do
	{
		point_type point = get(coordinates_map, vertex);
		Plan.push_back(point);
		child = vertex;
		vertex = pred[child];
	} while (vertex != child);
	std::reverse(std::begin(Plan), std::end(Plan));

	return Plan;
}

void VoronoiPlanner::MakeRoadmap(const point_type &car_location, const std::vector<segment_type> &Walls, double allowed_obs_dist)
{
	this->allowed_obs_dist = allowed_obs_dist;

	// Map each wall to its connected component
	std::vector<int> component(Walls.size(), 0);
	for (size_t i = 1; i < Walls.size(); ++i)
	{
		if (euclidean_distance(Walls[i - 1].high(), Walls[i].low()) < 0.001)
		{
			component[i] = component[i - 1];
		}
		else
		{
			component[i] = component[i - 1] + 1;
		}
	}

	// Clear the previous roadmap
	Roadmap.clear();

	// Compute the Voronoid diagram for the walls
	std::vector<segment_type> Walls_mm(Walls.size());
	for (size_t i = 0; i < Walls.size(); ++i)
	{
		segment_type seg = Walls[i];
		Walls_mm[i] = boost::polygon::scale_up(seg, 1000);
	}
	VD VDiagram;
	construct_voronoi(Walls_mm.begin(), Walls_mm.end(), &VDiagram);
	std::cout << "Voronoi. " << std::flush;

	// Maintain a mapping from Voronoi vertices to roadmap vertices
	std::unordered_map<const vertex_type *, vertex_descriptor> voronoi_to_roadmap;

	// Color too-close (to the walls) Voronoi vertices.
	color_close_vertices(VDiagram, Walls_mm);
	std::cout << "Color. " << std::flush;

	// Add the colorless vertices to the roadmap.
	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it)
	{
		// If vertex is too close to the walls, skip.
		if (it->color() == 1)
			continue;
		vertex_descriptor newvertex = add_roadmap_vertex(point_type(it->x() / 1000.f, it->y() / 1000.f)); // The coordinates are in millimeters in VDiagram, but in meters in the roadmap.
		voronoi_to_roadmap.insert({&(*it), newvertex});
	}
	std::cout << "Vertices. " << std::flush;

	for (const auto &edge : VDiagram.edges())
	{
		if (!edge.is_primary())
			continue;
		if (component[edge.cell()->source_index()] == component[edge.twin()->cell()->source_index()])
		{
			continue;
		}

		if (edge.is_finite() && edge.vertex0()->color() == 0 && edge.vertex1()->color() == 0)
		{
			if (edge.is_linear())
			{
				try
				{
					add_linear_edge(edge, voronoi_to_roadmap);
				}
				catch (int e)
				{
					std::cout << "Failed adding linear edge." << std::endl
										<< std::flush;
				}
			}
			else
			{
				try
				{
					add_curved_edge(edge, voronoi_to_roadmap, Walls_mm);
				}
				catch (int e)
				{
					std::cout << "Failed adding curved edge." << std::endl
										<< std::flush;
				}
			}
		}
	}
	std::cout << "Edges. " << std::flush;

	add_start_vertex(car_location);
	add_finish_vertex();
}

// Assumes that the Voronoi diagram has only input segments, i.e. no input points.
point_type VoronoiPlanner::retrieve_endpoint(const cell_type &cell, const std::vector<segment_type> &Walls)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT)
	{
		return low(Walls[index]);
	}
	else
	{
		return high(Walls[index]);
	}
}

// Assumes that the Voronoi diagram has only input segments, i.e. no input points.
segment_type VoronoiPlanner::retrieve_segment(const cell_type &cell, const std::vector<segment_type> &Walls)
{
	source_index_type index = cell.source_index();
	return Walls[index];
}

void VoronoiPlanner::sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge, const std::vector<segment_type> &Walls)
{
	point_type point = edge.cell()->contains_point() ? retrieve_endpoint(*edge.cell(), Walls) : retrieve_endpoint(*edge.twin()->cell(), Walls);
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell(), Walls) : retrieve_segment(*edge.cell(), Walls);
	boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_discretization_error * 1000.f, sampled_edge);
}

void VoronoiPlanner::color_close_vertices(const VD &vd, const std::vector<segment_type> &Walls)
{
	for (const auto &vertex : vd.vertices())
		vertex.color(0);

	for (const auto &vertex : vd.vertices())
	{
		point_type voronoi_point(vertex.x(), vertex.y());
		const cell_type *cell = vertex.incident_edge()->cell();
		if (cell->contains_point())
		{
			point_type endpoint = retrieve_endpoint(*cell, Walls);
			if (euclidean_distance(endpoint, voronoi_point) < allowed_obs_dist * 1000.f) // *1000.f to convert to millimeters
				vertex.color(1);
		}
		else
		{ // i.e. cell contains a segment
			segment_type segment = retrieve_segment(*cell, Walls);
			if (euclidean_distance(segment, voronoi_point) < allowed_obs_dist * 1000.f)
				vertex.color(1);
		}
	}
}

VoronoiPlanner::vertex_descriptor VoronoiPlanner::add_roadmap_vertex(point_type point)
{
	vertex_descriptor newvertex = add_vertex(Roadmap);
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	coordinates_map[newvertex] = point;
	return newvertex;
}

VoronoiPlanner::edge_descriptor VoronoiPlanner::add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1)
{
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	double weight = euclidean_distance(coordinates_map[vertex0], coordinates_map[vertex1]);
	return add_roadmap_edge(vertex0, vertex1, weight);
}

VoronoiPlanner::edge_descriptor VoronoiPlanner::add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1, double weight)
{
	edge_descriptor roadmap_edge;
	bool inserted;
	tie(roadmap_edge, inserted) = add_edge(vertex0, vertex1, Roadmap);
	weight_map_t weight_map = get(boost::edge_weight, Roadmap);
	weight_map[roadmap_edge] = weight;
	return roadmap_edge;
}

/// Assumes the roadmap vertices have been added before.
void VoronoiPlanner::add_linear_edge(const edge_type &edge, std::unordered_map<const vertex_type *, vertex_descriptor> &voronoi_to_roadmap)
{
	vertex_descriptor roadmap_vertex0;
	vertex_descriptor roadmap_vertex1;
	try
	{
		roadmap_vertex0 = voronoi_to_roadmap.at(edge.vertex0());
		roadmap_vertex1 = voronoi_to_roadmap.at(edge.vertex1());
	}
	catch (int e)
	{
		std::cout << "Trying to add edge for nonexistent vertices!" << std::endl
							<< std::flush;
		return;
	}

	point_type p0(edge.vertex0()->x(), edge.vertex0()->y());
	point_type p1(edge.vertex1()->x(), edge.vertex1()->y());
	add_roadmap_edge(roadmap_vertex0, roadmap_vertex1, euclidean_distance(p0, p1) / 1000.f);
}

void VoronoiPlanner::add_curved_edge(const edge_type &edge, std::unordered_map<const vertex_type *, vertex_descriptor> &voronoi_to_roadmap, const std::vector<segment_type> &Walls_mm)
{
	point_type vertex0(edge.vertex0()->x(), edge.vertex0()->y());
	point_type vertex1(edge.vertex1()->x(), edge.vertex1()->y());
	std::vector<point_type> samples;
	samples.push_back(vertex0);
	samples.push_back(vertex1);
	sample_curved_edge(edge, &samples, Walls_mm);

	// Check that all segments of the discretization are far enough from the focus of the parabola
	point_type focus = edge.cell()->contains_point() ? retrieve_endpoint(*edge.cell(), Walls_mm) : retrieve_endpoint(*edge.twin()->cell(), Walls_mm);
	for (std::size_t i = 0; i < samples.size() - 1; ++i)
	{
		segment_type segment(samples[i], samples[i + 1]);
		if (euclidean_distance(segment, focus) < allowed_obs_dist * 1000.f)
			return;
	}

	std::vector<vertex_descriptor> vertices(samples.size());
	try
	{
		vertices[0] = voronoi_to_roadmap.at(edge.vertex0());
		vertices[samples.size() - 1] = voronoi_to_roadmap.at(edge.vertex1());
	}
	catch (int e)
	{
		std::cout << "Trying to add edge for nonexistent vertices!" << std::endl
							<< std::flush;
		return;
	}

	// Add all the new points from discretization to the roadmap
	for (std::size_t i = 1; i < samples.size() - 1; ++i)
		vertices[i] = add_roadmap_vertex(boost::polygon::scale_down(samples[i], 1000));
	// Add all the edges of the discretization
	for (std::size_t i = 0; i < samples.size() - 1; ++i)
	{
		auto si = boost::polygon::scale_down(samples[i], 1000);
		auto sii = boost::polygon::scale_down(samples[i + 1], 1000);
		double weight = euclidean_distance(si, sii);
		add_roadmap_edge(vertices[i], vertices[i + 1], weight);
	}
}

void VoronoiPlanner::add_start_vertex(const point_type &car_location)
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	edge_descriptor e_closest = get_closest_edge(car_location);
	vertex_descriptor vertex0 = source(e_closest, Roadmap);
	vertex_descriptor vertex1 = target(e_closest, Roadmap);
	point_type point0 = get(coordinates_map, vertex0);
	point_type point1 = get(coordinates_map, vertex1);
	using namespace boost::numeric;
	ublas::vector<double> v0(2), v1(2), v_closest(2);
	v0[0] = point0.x() - car_location.x();
	v0[1] = point0.y() - car_location.y();
	v1[0] = point1.x() - car_location.x();
	v1[1] = point1.y() - car_location.y();
	v_closest = -(ublas::inner_prod(v0, v1 - v0) / ublas::norm_2_square(v1 - v0)) * (v1 - v0);
	point_type p_closest(point0.x() + v_closest[0], point0.y() + v_closest[1]);
	start_vertex = add_roadmap_vertex(p_closest);
	add_roadmap_edge(start_vertex, vertex0);
	add_roadmap_edge(start_vertex, vertex1);
}

void VoronoiPlanner::add_finish_vertex()
{
}

void VoronoiPlanner::GetRoadmapPoints(std::list<point_type> &points) // TODO: Change to vector and reserve space
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::vertex_iterator vi, vi_end;
	point_type coords;
	for (tie(vi, vi_end) = vertices(Roadmap); vi != vi_end; ++vi)
	{
		coords = get(coordinates_map, *vi);
		points.push_back(coords);
	}
}

void VoronoiPlanner::GetRoadmapSegments(std::vector<segment_type> &segments) // TODO: Change to vector and reserve space
{
	using namespace boost;
	weight_map_t weight_map = get(boost::edge_weight, Roadmap);
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = edges(Roadmap); ei != ei_end; ++ei)
	{
		vertex_descriptor vertex0 = source(*ei, Roadmap);
		vertex_descriptor vertex1 = target(*ei, Roadmap);
		point_type point0 = get(coordinates_map, vertex0);
		point_type point1 = get(coordinates_map, vertex1);
		double diff = abs(euclidean_distance(point0, point1) - weight_map[*ei]);
		// if (diff > 0.001)
		// {
		// 	std::cout << "Dist: " << euclidean_distance(point0, point1) << ", Weight: " << weight_map[*ei]
		// 						<< " (" << point0.x() << "," << point0.y() << "), (" << point1.x() << "," << point1.y() << ")" << std::endl
		// 						<< std::flush;
		// }
		segments.push_back(segment_type(point0, point1));
	}
}

VoronoiPlanner::vertex_descriptor VoronoiPlanner::get_closest_vertex(point_type point)
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::vertex_iterator vi, vi_end;
	double closest_distance = std::numeric_limits<double>::max(); // infinity
	vertex_descriptor current_vertex;
	for (tie(vi, vi_end) = vertices(Roadmap); vi != vi_end; ++vi)
	{
		point_type current_point = get(coordinates_map, *vi);
		double current_distance = euclidean_distance(point, current_point);
		if (current_distance < closest_distance)
		{
			current_vertex = *vi;
			closest_distance = current_distance;
		}
	}
	return current_vertex;
}

VoronoiPlanner::edge_descriptor VoronoiPlanner::get_closest_edge(point_type point)
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::edge_iterator ei, ei_end;
	double min_distance = std::numeric_limits<double>::max(); // infinity
	edge_descriptor e_closest;
	for (tie(ei, ei_end) = edges(Roadmap); ei != ei_end; ++ei)
	{
		vertex_descriptor vertex0 = source(*ei, Roadmap);
		vertex_descriptor vertex1 = target(*ei, Roadmap);
		point_type point0 = get(coordinates_map, vertex0);
		point_type point1 = get(coordinates_map, vertex1);
		double current_distance = euclidean_distance(segment_type(point0, point1), point);
		if (current_distance < min_distance)
		{
			e_closest = *ei;
			min_distance = current_distance;
		}
	}
	return e_closest;
}
