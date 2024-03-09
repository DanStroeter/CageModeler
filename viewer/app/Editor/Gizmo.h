#pragma once

#include <Mesh/GeometryUtils.h>
#include <Mesh/Selection.h>
#include <Rendering/Core/RenderProxy.h>
#include <Tools/Tool.h>

struct ViewInfo;
struct InputActionParams;
class InputSubsystem;
class RenderProxyCollector;
class PolygonMesh;
class Scene;

enum class GizmoType : uint8_t
{
	Translate = 0,
	Rotate,
	Scale,
	MaxNum = 3
};

enum class GizmoAxis : uint8_t
{
	X,
	Y,
	Z
};

struct GizmoHitResult
{
	GizmoAxis _axis = GizmoAxis::X;
};

class Gizmo
{
public:
	explicit Gizmo(Scene& scene);

	/**
	 * Creates a new render proxy for the screen pass.
	 * @param renderProxyCollector A pointer to the render proxy collector that will register the new proxy instance.
	 * @param device A reference to the Vulkan device.
	 * @param renderCommandScheduler A pointer to the render command scheduler to pass to the proxy.
	 * @return A new instance of a render proxy.
	*/
	void CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
		const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler);

	/**
	 * Destroy the render proxy associated with the mesh.
	 * @param renderProxyCollector A pointer to the render proxy collector that will deregister the new proxy instance.
	 */
	void DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector);

	/**
	 * Returns the current gizmo position.
	 * @return The current gizmo position.
	 */
	[[nodiscard]] glm::vec3 GetPosition() const
	{
		return glm::vec3(_matrix[3]);
	}

	/**
	 * Returns a copy of the current gizmo matrix.
	 * @return The gizmo matrix.
	 */
	[[nodiscard]] glm::mat4 GetMatrix() const
	{
		return _matrix;
	}

	/**
	 * Sets the gizmo matrix position, the orientation will be kept the same.
	 * @param viewInfo The view info of the camera.
	 * @param position The new gizmo position.
	 */
	void SetPosition(const ViewInfo& viewInfo, const glm::vec3& position);

	/**
	 * Updates the model matrix of the gizmo to be of constant size.
	 * @param viewInfo The view info of the camera.
	 * @param matrix A new model matrix.
	 */
	void UpdateModelMatrix(const ViewInfo& viewInfo, const glm::mat4& matrix);

	/**
	 * Updates the model matrix of the gizmo to be of constant size.
	 * @param viewInfo The view info of the camera.
	 */
	void UpdateModelMatrix(const ViewInfo& viewInfo);

	/**
	 * Sets the visibility of the object.
	 * @param gizmoType The gizmo type to change.
	 * @param isVisible The visibility of the mesh.
	 */
	void SetVisible(const GizmoType gizmoType, const bool isVisible) const;

	/**
	 * Sets the visibility of all gizmos.
	 * @param isVisible The visibility of the mesh.
	 */
	void SetVisible(const bool isVisible) const;

	/**
	 * Sets the highlighted state of a specific gizmo axis and type.
	 * @param gizmoType The gizmo type to change.
	 * @param gizmoAxis The gizmo axis to change.
	 * @param isHighlighted Whether the gizmo should be colored highlighted.
	 */
	void SetHighlighted(const GizmoType gizmoType, const GizmoAxis gizmoAxis, const bool isHighlighted) const;

	/**
	 * Cast a ray against the gizmo to determine the closest hit location.
	 * @param viewInfo The view info of the camera.
	 * @param gizmoType The gizmo type to check.
	 * @param screenPosition The position of the moouse in screen space.
	 * @return The closest hit on the mesh.
	 */
	[[nodiscard]] std::optional<GizmoHitResult> QueryRayHit(const ViewInfo& viewInfo,
		const GizmoType gizmoType,
		const glm::vec2& screenPosition) const;

private:
	using AxesMeshes = std::array<std::shared_ptr<PolygonMesh>, 3>;

private:
	template <typename T> friend struct ProxyCollectorHelper;

	/**
	 * Calculates the model matrix of the gizmo to be of constant size.
	 * @param viewInfo The view info of the camera.
	 */
	[[nodiscard]] glm::mat4 CalculateModelMatrixInternal(const ViewInfo& viewInfo) const;

	[[nodiscard]] AxesMeshes& GetMesh(const GizmoAxis axis)
	{
		return _meshes[static_cast<std::size_t>(axis)];
	}

private:
	/// The meshes of all 3 axes.
	std::array<AxesMeshes, static_cast<std::size_t>(GizmoType::MaxNum)> _meshes;

	/// The transform matrix of the gizmo.
	glm::mat4 _matrix { 1.0f };
};
