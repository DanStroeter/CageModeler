#pragma once

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <OpenMesh/Core/Utils/vector_traits.hh>

namespace OpenMesh
{
	template<typename T, int N, glm::qualifier Q>
	T norm(const glm::vec<N, T, Q>& v)
	{
		return glm::length(v);
	}

	template<typename T, int N, glm::qualifier Q>
	glm::vec<N, T, Q>& vectorize(glm::vec<N, T, Q>& v, T s)
	{
		for (std::size_t i = 0; i < N; i++)
		{
			v[i] = s;
		}

		return v;
	}

	template<int N, typename T, glm::qualifier Q>
	struct vector_traits<glm::vec<N, T, Q>>
	{
		using vector_type = glm::vec<N, T, Q>;
		using value_type = T;

		static constexpr size_t size_ = static_cast<std::size_t>(N);

		static constexpr size_t size()
		{
			return size_;
		}
	};

} // namespace OpenMesh

#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <OpenMesh/Core/Mesh/Handles.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
