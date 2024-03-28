#pragma once

#include <Core/Types.h>

#include <optional>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtx/norm.inl>
#include <glm/trigonometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

using MeshHandle = std::size_t;

struct Ray
{
	glm::vec3 _origin;
	glm::vec3 _direction;
};

struct AABB
{
	glm::vec3 _min = glm::vec3(std::numeric_limits<float>::max());
	glm::vec3 _max = glm::vec3(std::numeric_limits<float>::min());
};

struct MeshGeometry
{
	std::vector<glm::vec3> _positions;
	std::vector<glm::vec3> _colors;
	std::vector<uint32_t> _indices;
	std::vector<std::size_t> _polygonsMapping;
};

struct EigenMesh
{
	Eigen::MatrixXd _vertices;
	Eigen::MatrixXi _faces;
};

namespace GeometryUtils
{
	inline std::vector<glm::vec3> EigenVerticesToGLM(const Eigen::MatrixXd& vertices)
	{
		std::vector<glm::vec3> positions(vertices.cols());
		for (auto i = 0; i < vertices.cols(); ++i)
		{
			positions[i] = glm::vec3(vertices(0, i), vertices(1, i), vertices(2, i));
		}

		return positions;
	}

	inline void ScaleEigenMesh(Eigen::MatrixXd& vertices, const float scaleFactor)
	{
		const auto scaleMat = glm::scale(glm::mat4(1.0f), glm::vec3(scaleFactor));

		for (int i = 0; i < vertices.rows(); ++i)
		{
			const auto scaledVertex = scaleMat * glm::vec4(vertices(i, 0), vertices(i, 1), vertices(i, 2), 1.0f);

			vertices(i, 0) = scaledVertex.x;
			vertices(i, 1) = scaledVertex.y;
			vertices(i, 2) = scaledVertex.z;
		}
	}

	[[nodiscard]] inline glm::quat MakeRotationQuatAxisAngle(const glm::vec3& axis, const float angle)
	{
		const auto qAxis = axis * std::sin(angle / 2);
		const auto qZ = std::cos(angle / 2);

		return glm::quat(qAxis.x, qAxis.y, qAxis.z, qZ);
	}

	[[nodiscard]] inline glm::quat MakeRotationQuatBetweenVectorsSnapped(const glm::vec3& from, const glm::vec3& to, const float angle)
	{
		const auto a = glm::normalize(from);
		const auto b = glm::normalize(to);
		const auto snappedAngle = std::floor(std::acos(glm::dot(a, b)) / angle) * angle;

		return MakeRotationQuatAxisAngle(glm::normalize(glm::cross(a, b)), snappedAngle);
	}

	[[nodiscard]] inline float Cross(const glm::vec2& pointA, const glm::vec2& pointB)
	{
		return pointA.x * pointB.y - pointA.y * pointB.x;
	}

	[[nodiscard]] inline std::optional<glm::vec3> RayPlaneIntersection(const Ray& ray,
		const glm::vec3& planeOrigin,
		const glm::vec3& planeNormal)
	{
		const auto denom = glm::dot(planeNormal, ray._direction);

		// If we are behind the plane, just use a position that is slightly in front of the camera.
		if (denom <= Epsilon)
		{
			return {};
		}

		const glm::vec3 rayToPlane = planeOrigin - ray._origin;
		const float t = glm::dot(rayToPlane, planeNormal) / denom;

		if (t < Epsilon)
		{
			return { };
		}

		return ray._origin + t * ray._direction;
	}

	[[nodiscard]] inline glm::vec3 ProjectPointOnPlane(const glm::vec3& point,
		const glm::vec3& planeOrigin,
		const glm::vec3& planeNormal)
	{
		const auto delta = point - planeOrigin;
		const auto dot = glm::dot(delta, planeNormal);

		return point - dot * planeNormal;
	}

	[[nodiscard]] inline float PointLineDistance(const glm::vec2& point,
		const glm::vec2& lineA,
		const glm::vec2& lineB)
	{
		const auto l2 = glm::length2(lineA - lineB);
		if (std::abs(l2) < Epsilon)
		{
			return glm::length2(lineA - point);
		}

		const auto a = point - lineA;
		const auto b = lineB - lineA;
		const auto d = glm::dot(a, b);
		const auto t = (std::max)(0.0f, (std::min)(1.0f, d / l2));
		const auto projection = lineA + t * b;

		return glm::length(point - projection);
	}

	[[nodiscard]] inline float CalculateDistanceFromPointToTriangle2D(const glm::vec2& p,
		const glm::vec2& p0,
		const glm::vec2& p1,
		const glm::vec2& p2)
	{
		const auto e0 = p1 - p0;
		const auto e1 = p2 - p1;
		const auto e2 = p0 - p2;

		if (Cross(e0, -e2) < Epsilon)
		{
			if (glm::length2(e2) < Epsilon)
			{
				return PointLineDistance(p, p0, p1);
			}
			else
			{
				return PointLineDistance(p, p0, p2);
			}
		}

		const auto v0 = p - p0;
		const auto v1 = p - p1;
		const auto v2 = p - p2;
		const auto pq0 = v0 - e0 * std::clamp(glm::dot(v0, e0) / glm::dot(e0, e0), 0.0f, 1.0f);
		const auto pq1 = v1 - e1 * std::clamp(glm::dot(v1, e1) / glm::dot(e1, e1), 0.0f, 1.0f);
		const auto pq2 = v2 - e2 * std::clamp(glm::dot(v2, e2) / glm::dot(e2, e2), 0.0f, 1.0f);
		const auto s = glm::sign(e0.x * e2.y - e0.y * e2.x);
		const glm::vec2 d = glm::min(glm::min(glm::vec2(glm::dot(pq0, pq0), s * (v0.x * e0.y - v0.y * e0.x)),
			glm::vec2(glm::dot(pq1,pq1), s * (v1.x * e1.y - v1.y * e1.x))),
			glm::vec2(glm::dot(pq2,pq2), s * (v2.x * e2.y - v2.y * e2.x)));

		return -sqrt(d.x) * glm::sign(d.y);
	}

	[[nodiscard]] inline glm::vec3 RotatePointAroundAxisWithAngle(const glm::vec3& point,
		const glm::vec3& axis,
		const float angleDeg)
	{
		const auto rotationMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(angleDeg), axis);

		return glm::vec3(glm::vec4(point, 1.0f) * rotationMatrix);
	}

	[[nodiscard]] inline glm::vec3 ClosestPointRayToRay(const Ray& from, const Ray& to)
	{
		const auto posDiff = to._origin - from._origin;
		const auto dirCross = glm::normalize(glm::cross(to._direction, from._direction));
		const auto posDiffFromProj = glm::dot(posDiff, from._direction) * from._direction;
		const auto posDiffDirCrossProj = glm::dot(posDiff, dirCross) * dirCross;
		const auto rejection = posDiff - posDiffFromProj - posDiffDirCrossProj;
		const auto distToLinePos = glm::length(rejection) / glm::dot(to._direction, glm::normalize(rejection));
		const auto closestPointOnLine = to._origin - to._direction * distToLinePos;

		return closestPointOnLine;
	}

	[[nodiscard]] inline float ClosestPointToRay(const Ray& ray, const glm::vec3& point)
	{
		// Construct a line segment from the ray origin to the point.
		const auto originToPoint = point - ray._origin;

		// Compute the projected length of that line segment onto the ray.
		const auto t = std::max(glm::dot(originToPoint, ray._direction), 0.0f);

		return t;
	}

	[[nodiscard]] inline std::optional<float> IntersectRayTriangle(const Ray& ray,
		const glm::vec3& vA,
		const glm::vec3& vB,
		const glm::vec3& vC)
	{
		const auto edgeA = vB - vA;
		const auto edgeB = vC - vA;
		const auto rayCrossEdgeB = glm::cross(ray._direction, edgeB);
		const auto det = glm::dot(edgeA, rayCrossEdgeB);

		// Ray is parallel to triangle.
		if (std::abs(det) < Epsilon)
		{
			return { };
		}

		const auto invDet = 1.0f / det;
		const auto s = ray._origin - vA;
		const auto u = invDet * glm::dot(s, rayCrossEdgeB);

		if (u < 0 || u > 1)
		{
			return { };
		}

		const auto sCrossEdgeA = glm::cross(s, edgeA);
		const auto v = invDet * glm::dot(ray._direction, sCrossEdgeA);

		if (v < 0 || u + v > 1)
		{
			return { };
		}

		const auto t = invDet * glm::dot(edgeB, sCrossEdgeA);

		if (t > Epsilon)
		{
			return t;
		}

		return { };
	}


	/**
	 * Computes the axis-aligned bounding box of the object.
	 * @return The axis-aligned bounding box of the object.
	 */
	[[nodiscard]] static AABB ComputeAABB(const EigenMesh& mesh)
	{
		AABB aabb { };

		for (int i = 0; i < mesh._vertices.rows(); ++i)
		{
			const glm::vec3 vertex(mesh._vertices(i, 0), mesh._vertices(i, 1), mesh._vertices(i, 2));

			aabb._min = glm::min(aabb._min, vertex);
			aabb._max = glm::max(aabb._max, vertex);
		}

		return aabb;
	}
}