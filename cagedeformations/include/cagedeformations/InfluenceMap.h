#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>

Eigen::Vector3d HSVtoRGB(double H, double S, double V);

/**
 * Viridis, the perceptually uniform sequential colormap of matplotlib, evaluated with the
 * usual degree six polynomial fit (max deviation from the tabulated colormap well below
 * one 8 bit color step). Unlike the rainbow ramp of the influence map it has a monotone
 * luminance, so equal steps in the distance read as equal steps in the color.
 *
 * @param t Position in the colormap, clamped to [0, 1].
 */
Eigen::Vector3d viridisToRGB(double t);

/**
 * Settings of the distance field color map. The isolines are drawn on the unnormalized
 * distances, so they keep counting in world units when the colors saturate against a
 * fixed normalization maximum.
 *
 * Both thresholds are plain doubles instead of optionals because this header is also
 * compiled as C++14 by the library and the command line tool.
 */
struct DistanceColorMapParams
{
	/// Normalization maximum of the color gradient. Zero (or less) normalizes against the
	/// largest distance of the exported field, which makes the colors of a single export
	/// easy to read but not comparable across exports. A positive value keeps separate
	/// exports comparable.
	double maxDistance = 0.;

	/// Isoline spacing in world units (the units of the exported vertices). A negative
	/// value spaces about 20 isolines over the normalization range, zero draws none.
	double contourInterval = -1.;

	/// Every n-th isoline is emphasized (drawn twice as thick and in a deeper stroke) so
	/// the bands in between stay countable. Values below two emphasize no isoline.
	int contourEmphasisEvery = 5;

	/// Written into the OBJ header comment to identify the exported field.
	std::string label;
};

/**
 * Writes the mesh with per-vertex colors to an OBJ file, prefixed by the given comment
 * lines. Modified from libigl.
 */
bool writeOBJVertexColors(const std::string& file_name, const Eigen::MatrixXd& V, const Eigen::MatrixXd& V_colors,
	const Eigen::MatrixXi& F, const std::vector<std::string>& header_comments = { });

void write_influence_color_map_OBJ(const std::string& file_name, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& T, const Eigen::MatrixXd& W, const std::vector<int>& control_vertices_idx, int cage_vertices_offset, bool transposeW);

/**
 * Writes a per-vertex distance field as an OBJ with vertex colors, in the same format as
 * write_influence_color_map_OBJ. The distance is mapped onto the viridis gradient and
 * overlaid with isolines every DistanceColorMapParams::contourInterval world units.
 *
 * The isolines are baked into the vertex colors, so their width follows the local mesh
 * resolution: a vertex is part of an isoline when its distance is within three quarters of
 * a vertex spacing (one and a half for the emphasized ones) of a multiple of the interval,
 * which keeps the lines closed on coarse meshes without flooding fine ones. Distance
 * fields have a unit gradient, so that band stays about one vertex wide either way.
 *
 * @param distances One distance per vertex of V, in the units of V.
 */
void write_distance_color_map_OBJ(const std::string& file_name, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& T, const Eigen::VectorXd& distances, const DistanceColorMapParams& params);
