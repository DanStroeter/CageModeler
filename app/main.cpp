#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <fstream>

#include <cagedeformations/globals.h>
#include <cagedeformations/GreenCoordinates.h>
#include <cagedeformations/MaximumLikelihoodCoordinates.h>
#include <cagedeformations/MaximumEntropyCoordinates.h>
#include <cagedeformations/InfluenceMap.h>
#include <cagedeformations/Parametrization.h>
#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/WeightInterpolation.h>
#include <cagedeformations/LoadFBX.h>

#include <boost/program_options.hpp>
//#define VERBOSE
#include <igl/bbw.h>
#include <igl/boundary_conditions.h>
#include <igl/harmonic.h>
#include <igl/lbs_matrix.h>
#include <igl/writeMSH.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/Timer.h>
#include <LBC/LBCSolver.h>
#ifdef WITH_SOMIGLIANA
#include <somigliana/somigliana_3d.h>
#endif

int main(int argc, char** argv)
{
	boost::program_options::options_description desc("This is a tool used for morphing with different coordinate variants.\n");

	std::string inputFile, outMeshFile = "a.msh", cageFile, cageDeformedFile, embeddedMeshFile, parameterFile, variant_string = "bbw", modelInfluenceFile, fbxFile;
	int numSamples = 1, numBBWSteps = 400, lbc_scheme = 2;
	float scaling_factor = 1.;
	int mec_flag = 1;
	std::unique_ptr<igl::Timer> timer;
	#ifdef WITH_SOMIGLIANA
	double somig_nu = 0, somig_bulging = 0, somig_blend_factor = 0;
	int somig_bulging_type = SWEPT_VOLUME;
	std::unique_ptr<green::somig_deformer_3> somig_deformer;
	#endif

	desc.add_options()
		("help,h", "Help screen")
		("verbose,v", boost::program_options::value<unsigned int>(&verbosity), "Handle verbosity (0: quiet, 1: verbose, default: 1)")
		("model,m", boost::program_options::value<std::string>(&inputFile), "Specifies the input *.msh or *.obj file of the to-be-deformed model")
		("cage,c", boost::program_options::value<std::string>(&cageFile), "Specifies the cage to use (Halfface *.hf file for subspaces and obj for others)")
		("cage-deformed,cd", boost::program_options::value<std::string>(&cageDeformedFile), "Specifies a derformed cage file (instead of a parametrization)")
		("fbx", boost::program_options::value<std::string>(&fbxFile), "Specifies the .fbx file to be loaded")
		("embedded,e", boost::program_options::value<std::string>(&embeddedMeshFile), "Specifies the embedded tet mesh file which embeds the cage and the deformation model")
		("parameters,p", boost::program_options::value<std::string>(&parameterFile), "Specifies the *.param parameter file to control mesh deformation")
		("samples,s", boost::program_options::value<int>(&numSamples), "Specifies the number of samples for the parameters")
		("output,o", boost::program_options::value<std::string>(&outMeshFile), "write resulting mesh to file")
		("no-offset", "Do not calculate an offset for model within the embedding (except the cage)")
		("find-offset", "Search points of embedding for offset (requires equality of points)")
		("scale", boost::program_options::value<float>(&scaling_factor), "Scale model, embedding and cage by factor")
		("time,t", "Measure runtime of coordinate calculation")
		("iter", boost::program_options::value<int>(&numBBWSteps), "The number of iterations for calculating BBW or LBC")
		("lbc-scheme", boost::program_options::value<int>(&lbc_scheme), "The weighting scheme for lbc")
#ifdef WITH_SOMIGLIANA
		("somig-nu", boost::program_options::value<double>(&somig_nu), "The material parameter nu for somigliana deformer")
		("somig-bulging", boost::program_options::value<double>(&somig_bulging), "The bulging parameter gamma for somigliana deformer")
		("somig-bulging-type", boost::program_options::value<int>(&somig_bulging_type), "The bulging type for somigliana deformer 0: solid angle, 1: swept volume (default 1)")
		("somig-blend", boost::program_options::value<double>(&somig_blend_factor), "The blending factor for somigliana deformer interpolating between local and global boundary conditions")
#endif
		("influence", "Evaluate the influence of the control vertices involved in the deformation and write the plot (Mesh for OBJ) to file")
		("interpolate-weights", "Interpolate weights of model vertices from the embedding (embedding does not contain vertices of model)")
		("harmonic", "Use harmonic coordinates by Joshi et al.")
#ifdef WITH_SOMIGLIANA
		("MVC", "Use mean value coordinates by Floater et al.")
#endif
		("LBC", "Use local barycentric coordinates by Zhang et al.")
		("green", "Use green coordinates by Lipman et al.")
		("BBW", "Use bounded biharmonic weights by Jacobson et al.")
		("QMVC", "Use triquad mean value coordinates by Thiery et al.")
		("QGC", "Use tri-quad green coordinates by Thiery et al.")
		("MLC", "Use maximum likelihood coordinates by Chang et al.")
		// ("MEC", boost::program_options::value<int>(&mec_flag), "Use maximum entropy coordinates by Hormann et al.")
		("MEC", "Use maximum entropy coordinates by Hormann et al.")   // here we only use MEC-1 prior function
#ifdef WITH_SOMIGLIANA
		("somigliana", "Use somigliana coordinates by Chen et al.")
#endif
		("subspace", "Use Linear subspace design by Wang et al.");
	boost::program_options::positional_options_description p;
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	boost::program_options::notify(vm);

	if (vm.count("help"))
	{
      std::cout << desc << '\n';
	  return 0;
	}

	const bool harmonic = static_cast<bool>(vm.count("harmonic"));
	const bool lbc = !harmonic && static_cast<bool>(vm.count("LBC"));
	const bool green = !lbc && !harmonic && static_cast<bool>(vm.count("green"));
	const bool QGC = !lbc && !harmonic && !green && static_cast<bool>(vm.count("QGC"));
	const bool QMVC = !QGC && !lbc && !harmonic && !green && static_cast<bool>(vm.count("QMVC"));
	const bool MLC = ! QMVC && !QGC && !lbc && !harmonic && !green && static_cast<bool>(vm.count("MLC"));
	const bool MEC = !MLC && !QMVC && !QGC && !lbc && !harmonic && !green && static_cast<bool>(vm.count("MEC"));
	const bool somigliana = 
#ifdef WITH_SOMIGLIANA
	!lbc && !harmonic && !green && !QMVC && !QGC && !MLC && !MEC && static_cast<bool>(vm.count("somigliana"));
	const bool MVC = !lbc && !harmonic && !green && ! QMVC && !QGC && !MLC && !MEC && !somigliana && static_cast<bool>(vm.count("MVC"));
#else
	false;
	const bool MVC = false;
#endif
	const bool load_fbx = static_cast<bool>(vm.count("fbx"));
	const bool find_offset = static_cast<bool>(vm.count("find-offset"));
	const bool scale = static_cast<bool>(vm.count("scale"));
	const bool influence = static_cast<bool>(vm.count("influence"));
	const bool load_deformed_cage = static_cast<bool>(vm.count("cage-deformed"));
	const bool interpolate_weights = static_cast<bool>(vm.count("interpolate-weights")) && !(QMVC || QGC || MEC || green || MVC || MLC || somigliana);
	const bool no_offset = static_cast<bool>(vm.count("no-offset")) || interpolate_weights;
	const bool measure_time = static_cast<bool>(vm.count("time"));

	// Display help page if requested
	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 0;
	}

	Parametrization params;
	if (vm.count("parameters"))
	{
		params = readParams(parameterFile);
	}
	else if (!load_deformed_cage && ! load_fbx)
	{
		std::cerr << "You need to specify either a deformed cage, parametrization of the cage or an fbx file";
		return 1;
	}

	if (!vm.count("model") && !load_fbx)
	{
		std::cerr << "No deformation model specified!\n";
		return 1;
	}
	if (measure_time)
	{
		timer = std::make_unique<igl::Timer>();
	}
	auto start_timer = [&timer, measure_time]() {
		if (measure_time)
		{
			timer->start();
		}
	};
	auto stop_timer = [&timer, measure_time]() {
		if (measure_time)
		{
			timer->stop();
		}
	};
#ifdef WITH_SOMIGLIANA
	if (somigliana || MVC)
	{
		somig_deformer = std::make_unique<green::somig_deformer_3>(somig_nu, static_cast<bool>(!verbosity));
	}
#endif
	Eigen::MatrixXd V, V_model, C;
	Eigen::MatrixXi T, T_model, CF;
	if (verbosity)
	{
		std::cout << "Loading deformation mesh\n";
	}

	if (load_fbx)
	{
		if (!load_fbx_file(fbxFile, V_model, T_model, C, CF))
		{
			std::cerr << "Failed to load fbx file\n";
			return 1;
		}
	}

	if (!load_fbx && !load_mesh(inputFile, V_model, T_model, scaling_factor))
	{
		std::cerr << "Failed to load mesh file\n";
		return 1;
	}

#ifdef WITH_SOMIGLIANA
	if (somigliana || MVC)
	{
		if (inputFile.substr(inputFile.size() - 4, 4).compare(".msh") == 0)
		{
			somig_deformer->V_ = V_model.transpose();
			somig_deformer->F_ = T_model;
		} 
		else if (!somig_deformer->load_mesh(inputFile))
		{
			std::cerr << "Failed to load mesh file\n";
			return 1;
		}
	}
#endif


	if (!MVC && !somigliana && !green && !QMVC && !QGC && !MLC && !MEC && !vm.count("embedded"))
	{
		std::cerr << "You must specify an embedding!\n";
		return 1;
	}

	if (!somigliana && !MVC && !green && !QMVC && !QGC && !MLC && !MEC)
	{
		if (verbosity)
		{
			std::cout << "Loading the embedding\n";
		}
		if (!load_mesh(embeddedMeshFile, V, T, scaling_factor))
		{
			return 1;
		}
	}

	if (!vm.count("cage"))
	{
		std::cerr << "You must specify a cage!\n";
		return 1;
	}

	int model_vertices_offset = 0, cage_vertices_offset = 0;

	// Finding model verts in embedding
	if (!interpolate_weights && find_offset && !MVC && !green && !QGC && !MLC && !MEC && !somigliana)
	{
		auto verices_equal = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
		{
			return abs(a(0) - b(0)) < igl::FLOAT_EPS && abs(a(1) - b(1)) < igl::FLOAT_EPS && abs(a(2) - b(2)) < igl::FLOAT_EPS;
		};

		const Eigen::Vector3d first_model_vert = V_model.row(0);
		bool found = false;
		for (int i = 0; i < V.rows(); ++i)
		{
			const Eigen::Vector3d embedding_vert = V.row(i);
			if (verices_equal(first_model_vert, embedding_vert))
			{
				found = true;
				model_vertices_offset = i;
				break;
			}
		}
		if (found && verbosity)
		{
			std::cout << "Found model verts in embedding with an offset of " << model_vertices_offset << "\n";
		}
		else
		{
			std::cerr << "Could not find model verts in embedding\n";
			return 1;
		}
	}
	else if (!green && !QMVC && !QGC && !MLC && !MEC && !somigliana && !MVC)
	{
		auto const additional_offset = no_offset ? 0 : V.rows() - (V_model.rows() + model_vertices_offset);
		model_vertices_offset += additional_offset;
		if (verbosity)
		{
			std::cout << "Adding an offset of " << additional_offset << "\n";
		}
	}

	if (verbosity)
	{
		std::cout << "Loading cage\n";
	}
	Eigen::MatrixXd C_deformed;
	Eigen::VectorXi P;
	Eigen::MatrixXi BE, CE;

	// Load cage if it has not been loaded already
	if (C.rows() == 0 && !load_cage(cageFile, C, P, CF, scaling_factor, !QGC && !QMVC,
		(!MVC && !green && !QMVC && !QGC && !MLC && !MEC && !somigliana) ? &V : nullptr,find_offset))
	{
		std::cerr << "Failed to load cage!\n";
		return 1;
	}

#ifdef WITH_SOMIGLIANA
	if (somigliana || MVC)
	{
		if (somig_deformer->load_cage(C, CF))
		{
			std::cerr << "Failed to load cage!\n";
			return 1;
		}
		somig_deformer->init();
	}
#endif

	if (!find_offset && !interpolate_weights)
	{
		model_vertices_offset = C.rows();
	}

	if (!green && !MVC && !QMVC && !QGC && !MLC && !MEC && verbosity)
	{
		std::cout << "Using " << model_vertices_offset << " as offset for model vertices in embedding\n";
	}

	if (load_deformed_cage)
	{
		Eigen::VectorXi P_deformed;
		Eigen::MatrixXi CF_deformed;
		if (!load_cage(cageDeformedFile, C_deformed, P_deformed, CF_deformed, scaling_factor, !QGC && !QMVC))
		{
			std::cerr << "Failed to load deformed cage!\n";
			return 1;
		}

		params = fromDeformedCage(C, C_deformed);
	}

	auto const suffix_pos = outMeshFile.find(".");
	const bool write_msh = outMeshFile.substr(suffix_pos + 1, outMeshFile.size()).compare("msh") == 0;

	Eigen::MatrixXd normals;
	std::vector<double> psi_tri;
	std::vector<Eigen::Vector4d> psi_quad;
	Eigen::VectorXi b;
	Eigen::MatrixXd bc;
	if (!MVC && !green && !QMVC && !QGC && !MLC && !MEC && !somigliana)
	{
		if (verbosity)
		{
			std::cout << "Computing Boundary conditions\n";
		}
		if (!igl::boundary_conditions(V, T, C, P, BE, CE, CF, b, bc))
		{
			std::cerr << "Failed to extract boundary conditions for cage!\n";
			return 1;
		}
		if (verbosity)
		{
			std::cout << "Done computing boundary conditions\n";
		}
	}
	else if (green)
	{
		calcNormals(C, CF, normals);
	}
	// compute BBW weights matrix
	Eigen::MatrixXd W, W_interpolated, psi;
	if (verbosity)
	{
		std::cout << "Computing weights\n";
	}

	if (harmonic)
	{
		variant_string = "harmonic";
		start_timer();
		if (!igl::harmonic(V, T, b, bc, 1, W))
		{
			std::cerr << "Failed to compute harmonic weights!\n";
			return 1;
		}
		stop_timer();
	}
	else if (lbc)
	{
		variant_string = std::string("lbc_") + std::to_string(lbc_scheme);
		auto const suffix_pos = embeddedMeshFile.find(".");
		auto const prefix = embeddedMeshFile.substr(0, suffix_pos);
		std::string weightsFile = prefix + std::string("_") + std::to_string(lbc_scheme) + std::string("_lbc_weights.dmat");
		std::ifstream in(weightsFile);
		if (in.good())
		{
			if (verbosity)
			{
				std::cout << "Reading weights from file " << weightsFile << "\n";
			}
			if (!igl::readDMAT(weightsFile, W))
			{
				std::cerr << "Failed to read weights from file!\n";
				return 1;
			}
		}
		else
		{
			LBC::DataSetup::WeightingScheme scheme = static_cast<LBC::DataSetup::WeightingScheme>(lbc_scheme);
			Eigen::MatrixXd sample_points(V.cols(), V.rows());
			for (int i = 0; i < V.rows(); ++i)
			{
				sample_points.col(i) = V.row(i);
			}
			Eigen::MatrixXi cell_vertices(T.cols(), T.rows());
			for (int i = 0; i < T.rows(); ++i)
			{
				cell_vertices.col(i) = T.row(i);
			}
			LBC::IndexVector control_point_idx(P.size());
			for (int i = 0; i < P.size(); ++i)
			{
				control_point_idx(i) = i;
			}
			std::vector< LBC::DataSetup::CageBoundaryFacetInfo > boundary_facet_info;
			for (int i = 0; i < CF.rows(); ++i)
			{
				const Eigen::Vector3i tri_indices = CF.row(i);
				std::vector<int> boundary_points;

				for (int j = P.size(); j < bc.rows(); ++j)
				{
					auto const row = bc.row(j);
					bool contains = true;
					for (int l = 0; l < 3; ++l)
					{
						if (row(tri_indices(l)) == 0)
						{
							contains = false;
							break;
						}
					}
					if (contains)
					{
						boundary_points.push_back(b(j));
					}
				}

				auto boundary_points_vec = LBC::IndexVector(boundary_points.size());
				for (int i = 0; i < boundary_points.size(); ++i)
				{
					boundary_points_vec(i) = boundary_points[i];
				}

				LBC::DataSetup::CageBoundaryFacetInfo info(tri_indices, boundary_points_vec);
				boundary_facet_info.push_back(info);
			}

			LBC::DataSetup ds(sample_points, control_point_idx, cell_vertices, boundary_facet_info, scheme);

			LBC::Param param;
			param.max_iterations = numBBWSteps;
			param.relaxation_alpha = 1.65;
			param.convergence_check_frequency = 10;
			param.output_frequency_ratio = 10;
			param.rel_primal_eps = 0.00000000001;
			param.rel_dual_eps = 0.00000000001;
			param.penalty_weight = 100;
			LBC::LBCSolver solver(param, ds);

			if (verbosity)
			{
				std::cout << "LBC Solver started\n";
			}
			start_timer();
			solver.solve();
			stop_timer();
			if (verbosity)
			{
				std::cout << "Finished computation\n";
			}

			W = ds.get_full_coordinate_values(solver.get_coordinates());
			if (!igl::writeDMAT(weightsFile, W))
			{
				std::cerr << "Failed to write weights to file!\n";
			}
		}
	}
	else if (green)
	{
		variant_string = "green";
		start_timer();
		calculateGreenCoordinatesFromQMVC(C, CF, normals, V_model, W, psi);
		stop_timer();
	}
	else if (QGC)
	{
		variant_string = "QGC";
		start_timer();
		calculateGreenCoordinatesTriQuad(C, CF, V_model, W, psi_tri, psi_quad);
		stop_timer();
	}
	else if (QMVC)
	{
		variant_string = "QMVC";
		start_timer();
		computeMVCTriQuad(C, CF, V_model, W);
		stop_timer();
	}
	else if (MLC)
	{
		variant_string = "MLC";
		start_timer();
		calculateMaximumLikelihoodCoordinates(C.transpose(), CF.transpose(), V_model.transpose(), W);
		stop_timer();
	}
	else if (MEC)
	{
		variant_string = "MEC";
		start_timer();
		calculateMaximumEntropyCoordinates(C.transpose(), CF.transpose(), V_model.transpose(), W, mec_flag);
		stop_timer();
	}
#ifdef WITH_SOMIGLIANA
	else if (somigliana)
	{
		variant_string = "somigliana";
		somig_deformer->precompute_somig_coords(); // somigliana coordinates ship their own timer
	}
	else if (MVC)
	{
		variant_string = "MVC";
		somig_deformer->precompute_mvc_coords();
	}
#endif
	else
	{
		igl::BBWData bbw_data;
		// only a few iterations for sake of demo
		bbw_data.active_set_params.max_iter = numBBWSteps;
		bbw_data.verbosity = 2;
		auto const suffix_pos = embeddedMeshFile.find(".");
		auto const prefix = embeddedMeshFile.substr(0, suffix_pos);
		std::string weightsFile = prefix + std::string("_weights.dmat");
		std::ifstream in(weightsFile);
		if (!in.good())
		{
			start_timer();
			if (!igl::bbw(V, T, b, bc, bbw_data, W))
			{
				std::cerr << "Failed to compute bounded biharmonic weights!\n";
				return 1;
			}
			stop_timer();
			// Write computed weights to file
			if (!igl::writeDMAT(weightsFile, W))
			{
				std::cerr << "Failed to write weights to file!\n";
			}
		}
		else
		{
			if (verbosity)
			{
				std::cout << "Reading weights from file " << weightsFile << "\n";
			}
			if (!igl::readDMAT(weightsFile, W))
			{
				std::cerr << "Failed to read weights from file!\n";
				return 1;
			}
		}
	}
	if (measure_time)
	{
		if (verbosity)
		{
			std::cout << "Calculating weights took " << timer->getElapsedTime() << "seconds\n";
		} 
		else
		{
			std::cout << (MVC || somigliana ? 
#ifdef WITH_SOMIGLIANA
				somig_deformer->runtime 
#else
				timer->getElapsedTime()
#endif
				: timer->getElapsedTime()) << "\n";
		}
	}
	if (verbosity)
	{
		std::cout << "Done computing weights\n";
	}

	if (!lbc && !green && !QMVC && !QGC && !MLC && !MEC)
	{
		W  = (W.array().colwise() / W.array().rowwise().sum()).eval();
	}
	Eigen::MatrixXd M;
	// precompute linear blend skinning matrix
	if (verbosity)
	{
		std::cout << "Calculating M\n";
	}
	if (green || QMVC || QGC)
	{
		M = W;
	}
	else {
		if (interpolate_weights)
		{
			interpolateWeightsInEmbedding(V_model, W, V, T, C.rows(), W_interpolated, M);
		}
		else
		{
			igl::lbs_matrix(V, W, M);
		}
	}
	if (verbosity)
	{
		std::cout << "Done Calculating M\n";
	}
	const int dim = C.cols();
	auto translationFactors = createRegularSampling(numSamples, params.minFactor, params.maxFactor);
	auto const numControlVertices = C.rows();
	std::vector<uint8_t> control_vertices_marked(numControlVertices, 0);
	std::vector<int> control_vertices_idx;
	for (auto it = params.translations_per_vertex.begin(), end = params.translations_per_vertex.end(); it != end; ++it)
	{
		auto const control_vertex_idx = it->first;

		control_vertices_marked[control_vertex_idx] = 1;
		if (influence)
		{
			control_vertices_idx.push_back(control_vertex_idx);
		}
	}

	if (influence)
	{
		auto const base_name = outMeshFile.substr(0, suffix_pos);
		if (!MVC)
		{
			write_influence_color_map_OBJ(base_name + "_influence_" + variant_string + ".obj", V_model, T_model, interpolate_weights ? W_interpolated : W,
				control_vertices_idx, (green || QGC || MLC || MEC || interpolate_weights) ? 0 : model_vertices_offset, green || QGC || MLC || MEC);
		}
#ifdef WITH_SOMIGLIANA	
		else
		{
			write_influence_color_map_OBJ(base_name + "_influence_" + variant_string + ".obj", V_model, T_model, somig_deformer->getPhi(),
				control_vertices_idx, 0, true);
		}
#endif
	}

	auto const numTransformations = numControlVertices;
	Eigen::VectorXi tet_tags(T_model.rows());
	for (int j = 0; j < T_model.rows(); ++j)
	{
		tet_tags(j) = 1;
	}

	Eigen::MatrixXd Transformation(numTransformations * (dim + 1), dim);
	for (int j = 0; j < numTransformations; ++j)
	{
		auto a = Eigen::Affine3d::Identity();
		Transformation.block(j * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}

	for (int i = 0; i < translationFactors.size(); ++i)
	{
		Eigen::MatrixXd Cage_transforms = Transformation;
		C_deformed = C;
		auto const factor = translationFactors[i];

		for (auto it = params.translations_per_vertex.begin(), end = params.translations_per_vertex.end(); it != end; ++it)
		{
			auto const cage_vertex_idx = it->first;
			auto const translation = Eigen::Affine3d(Eigen::Translation3d(factor * it->second));
			Transformation.block(cage_vertex_idx * (dim + 1), 0, dim + 1, dim) =
				translation.matrix().transpose().block(0, 0, dim + 1, dim);
			Eigen::Vector3d cage_vertex = C.row(cage_vertex_idx);
			C_deformed.row(cage_vertex_idx) = cage_vertex + factor * it->second;
		}

		Eigen::MatrixXd U, U_model(V_model.rows(), 3);
		if (green)
		{
			Eigen::MatrixXd normals_deformed;
			calcNormals(C_deformed, CF, normals_deformed);
			calcScalingFactors(C, C_deformed, CF, normals_deformed);
			U_model = W.transpose() * C_deformed + psi.transpose() * normals_deformed;
		}
		else if (QGC)
		{
			calcNewPositionsTriQuad(C, C_deformed, CF, W, psi_tri, psi_quad, U_model);
		}
		else if (QMVC || MLC || MEC)
		{
			U_model = W.transpose() * C_deformed;
		}
#ifdef WITH_SOMIGLIANA
		else if (somigliana)
		{
			somig_deformer->deform(C_deformed, SOMIGLIANA, static_cast<BulgingType>(somig_bulging_type), somig_bulging, somig_blend_factor);
		}
		else if (MVC)
		{
			somig_deformer->deform(C_deformed, MEANVALUE, static_cast<BulgingType>(somig_bulging_type), somig_bulging, somig_blend_factor);
		}
#endif
		else
		{
			U = M * Transformation;
		}

		if (!MVC && !green && !QMVC && !QGC && !MLC && !MEC && !somigliana)
		{
			for (int j = 0; j < V_model.rows(); ++j)
			{
				U_model.row(j) = U.row(j + model_vertices_offset);
				if (i == 0 && translationFactors.size() > 1)
				{
					Eigen::Vector3d a = U_model.row(j);
					auto const vert = V_model.row(j);
					Eigen::Vector3d b(vert[0], vert[1], vert[2]);
					const float dist = (a - b).squaredNorm();
					assert(dist < igl::FLOAT_EPS);
				}

			}
		}

		auto const prefix = outMeshFile.substr(0, suffix_pos);
		auto const interpolation_factor = translationFactors.size() > 1 ? static_cast<float>(i) / static_cast<float>(translationFactors.size() - 1) : 1;
		auto const middle = std::string("_") + std::to_string(interpolation_factor) + std::string("_");

		if (write_msh)
		{
#ifdef WITH_SOMIGLIANA
		if (somigliana || MVC)
		{
			U_model = somig_deformer->V_.transpose();
		}
#endif
			igl::writeMSH(prefix + middle + variant_string + std::string(".msh"), U_model, Eigen::MatrixXi(), T_model, Eigen::MatrixXi(), tet_tags, std::vector<std::string>(),
				std::vector<Eigen::MatrixXd>(), std::vector<std::string>(), std::vector<Eigen::MatrixXd>(), std::vector<Eigen::MatrixXd>());
		}
		else
		{
#ifdef WITH_SOMIGLIANA
			if (somigliana || MVC)
			{
				const std::string fname = prefix + middle + variant_string;
				somig_deformer->save_mesh(fname.c_str());
			}
			else
#endif
			{
				if (verbosity)
				{
					std::cout << "Writing " << prefix + middle + variant_string + std::string(".obj") << "\n";
				}
				igl::writeOBJ(prefix + middle + variant_string + std::string(".obj"), U_model, T_model);
			}
		}
	}

	return 0;
}
