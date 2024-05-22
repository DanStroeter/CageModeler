#include <cagedeformations/Parametrization.h>

#include <fstream>

std::vector<float> createRegularSampling(int numSamples, float minValue /*= 0.f*/, float maxValue /*= 1.f*/)
{
	if (numSamples == 1)
	{
		return { maxValue };
	}

	std::vector<float> samples(numSamples);

	const float step = (maxValue - minValue) / (static_cast<float>(numSamples) - 1.f);

	for (auto i = 0; i < numSamples; ++i)
	{
		samples[i] = minValue + static_cast<float>(i) * step;
	}

	return samples;
}

Parametrization readParams(std::string paramFile)
{
	Parametrization params;

	std::ifstream in(paramFile);

	in >> params.minFactor;
	in >> params.maxFactor;

	unsigned int numTransformations = 0;

	in >> numTransformations;
	
	for (unsigned int i = 0; i < numTransformations; ++i)
	{
		int cage_vertex_idx = 0;
		double x=0, y=0, z=0;

		in >> cage_vertex_idx;
		in >> x;
		in >> y;
		in >> z;

		params.translations_per_vertex[cage_vertex_idx] = Eigen::Vector3d(x, y, z);
	}
	
	in.close();
	return params;
}

Parametrization fromDeformedCage(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed)
{
	Parametrization params;

	params.minFactor = 0;
	params.maxFactor = 1;

	for (int i = 0; i < C.rows(); ++i)
	{
		const Eigen::Vector3d cage_vertex = C.row(i);
		const Eigen::Vector3d deformed_cage_vertex = C_deformed.row(i);

		auto const distance = (cage_vertex - deformed_cage_vertex).norm();

		if (distance > 1.e-4)
		{
			params.translations_per_vertex[i] = deformed_cage_vertex - cage_vertex;
		}
	}

	return params;
}