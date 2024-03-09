#pragma once

#if PLATFORM_WINDOWS
	#define _USE_MATH_DEFINES
#endif

#include <Mesh/OpenMeshTraits.h>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

struct MeshTraits: OpenMesh::DefaultTraits
{
	using Point = glm::vec3;
	using Normal = glm::vec3;
	using TexCoord1D = float;
	using TexCoord2D = glm::vec2;
	using TexCoord3D = glm::vec3;
	using TextureIndex = int32_t;
	using Color = glm::vec3;

	VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge | OpenMesh::Attributes::Status);
	EdgeAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Status);
	FaceAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Status);
};

using PolyMeshKernel = OpenMesh::PolyMesh_ArrayKernelT<MeshTraits>;

using VertexHandle = PolyMeshKernel::VertexHandle;
using HalfEdgeHandle = PolyMeshKernel::HalfedgeHandle;
using EdgeHandle = PolyMeshKernel::EdgeHandle;
using FaceHandle = PolyMeshKernel::FaceHandle;