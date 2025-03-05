#pragma once

#include <Rendering/Scene/SceneRenderer.h>
#include <UI/NewProjectPanel.h>
#include <Editor/Scene.h>
#include <Core/Subsystem.h>
#include <Core/DoubleBuffer.h>
#include <Mesh/MeshTransformation.h>
#include <Thread/ThreadPool.h>
#include <Tools/Tool.h>

class ProjectSettingsPanel;
class ProjectOptionsPanel;
class ThreadPool;
class ToolSystem;
class CameraSubsystem;
class MeshOperationSystem;
class DeformationPanel;
class NewProjectPanel;
class ToolBar;
class StatusBar;
class RenderPipelineManager;
class InputSubsystem;

/**
 * This class serves as a mediator between the user interface backend and the current state of the application.
 * That allows for a better separation between the Vulkan side of the UI rendering and the actual UI layout and setup
 * that is specific based on the current state.
 */
class Editor
{
public:
	Editor(const SubsystemPtr<InputSubsystem>& inputSubsystem,
		const SubsystemPtr<CameraSubsystem>& cameraSubsystem);

	void Initialize(const std::shared_ptr<SceneRenderer>& sceneRenderer);

	void RecordUI();

	void Update(const double deltaTime);

private:
	void SetUpUIElements();

	void CreateSceneLights() const;

	/**
	 * Invoked when the new project window is cancelled.
	 */
	void OnNewProjectCancelled();

	/**
	 * Invoked when a new project has been created from the window.
	 */
	void OnNewProjectCreated();

	/**
	 * Invoked when the project settings have been changed and new ones have been applied.
	 */
	void OnProjectSettingsApplied();

	/**
	 * Invoked when the project settings get cancelled.
	 */
	void OnProjectSettingsCancelled();

	/**
	 * Checks for a hit with the currently active gizmo.
	 * @param viewInfo The current camera view info.
	 * @param activeGizmoType Currently active gizmo type (if any).
	 */
	void UpdateGizmoSelection(const ViewInfo& viewInfo, const GizmoType activeGizmoType);

	/**
	 * Checks for any closest mesh elements and highlights or selects them.
	 * @param viewInfo The current camera view info.
	 */
	void UpdateMeshSelection(const ViewInfo& viewInfo);

	/**
	 * Exports the current frame of the deformed mesh as an .OBJ file.
	 * @param filepath A filepath for the output .OBJ file.
	 */
	void ExportCurrentDeformedMesh(std::filesystem::path filepath) const;

	/**
	 * Exports all deformed meshes as an .OBJ file.
	 * @param filepath A filepath for the output .OBJ file.
	 */
	void ExportDeformedMeshes(std::filesystem::path filepath) const;

	/**
	 * Exports the deformed cage as an .OBJ file.
	 * @param filepath A filepath for the output .OBJ file.
	 */
	void ExportDeformedCage(std::filesystem::path filepath) const;

	/**
	 * Exports the influence color map as an .OBJ file.
	 * @param filepath A filepath for the output .OBJ file.
	 */
	void ExportInfluenceColorMap(std::filesystem::path filepath) const;

	/**
	 * Exports the weights as a .DMAT file.
	 * @param filepath A filepath for the output .DMAT file.
	 */
	void ExportWeights(std::filesystem::path filepath) const;

	/**
	 * Triggers a new operation to load the requested mesh data.
	 * @return A new project.
	 */
	[[nodiscard]] MeshOperationResult<std::shared_ptr<ProjectData>> CreateProject() const;

	/**
	 * Computes the weights based on the input mesh and cage files.
	 */
	[[nodiscard]] MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCageWeights(const ProjectData& projectData) const;

	/**
	 * Computes and caches the transformed vertices of the deformed mesh. The data will be stored in the project data
	 * and stored for later use or export.
	 */
	[[nodiscard]] MeshOperationResult<MeshComputeDeformationOperationResult> ComputeDeformedMesh(EigenMesh mesh,
		EigenMesh cage,
		EigenMesh deformedCage,
		const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme weightingScheme,
		const std::shared_ptr<somig_deformer_3>& somiglianaDeformer,
		const int32_t modelVerticesOffset,
		const int32_t numSamples,
		const bool interpolateWeights) const;

	/**
	 * Sets the gizmo position based on the selection.
	 * @param viewInfo The current camera view info.
	 */
	void ResetGizmoPositionFromSelection(const ViewInfo& viewInfo) const;

	/**
	 * Updates the mesh vertex colors based on the project settings and whether the influence map rendering is required.
	 */
	void UpdateMeshVertexColors(const bool shouldRenderInfluenceMap) const;

	/**
	 * Extracts the vertex data from the calculated deformation data and apply it to the current mesh for the given frame index.
	 * @param frameIndex The frame (or sample) index to be applied to the deformed mesh vertices.
	 */
	void UpdateDeformedMeshPositionsFromDeformationData(const std::optional<uint32_t> frameIndex = { });

	/**
	 * Invoked on a LMB click by the user.
	 * @param actionParams Action parameters from the input system.
	 */
	void OnClicked(const InputActionParams& actionParams);
	void OnMouseClickPressed(const InputActionParams& actionParams);
	void OnMouseMoved(const InputActionParams& actionParams);
	void OnMouseClickReleased(const InputActionParams& actionParams);

	/**
	 * Called when we click on the mesh and we want to adjust the selection. This function will go through the highlighted
	 * elements and the current selection type and mark the elements as selected on the mesh on mouse release.
	 * @param actionParams Action parameters from the input system.
	 */
	void OnClickedSelection(const InputActionParams& actionParams);

	/**
	 * Computes the influence map of the current mesh and updates the vertex colors.
	 */
	void OnComputeInfluenceColorMap(const bool shouldRenderInfluenceMap) const;

	/**
	 * Invoked when a new tool is selected.
	 * @param toolType The new tool type.
	 */
	void OnToolSelectionChanged(const ToolType toolType);

	/**
	 * Invoked when a new selection type is selected.
	 * @param selectionType The new selection type.
	 */
	void OnSelectionTypeChanged(const SelectionType selectionType);

	/**
	 * Whenever the frame index in the sequencer changes we have to update the vertices with the pre-computed
	 * vertex data.
	 * @param newFrameIndex The new frame index.
	 */
	void OnSequencerFrameIndexChanged(const uint32_t newFrameIndex);

	/**
	 * Whenever the maximum number of frames in the sequencer changes we have to call the deformation operation.
	 * @param currentFrameIndex The current frame index.
	 * @param numFrames The maximum number of frames.
	 */
	void OnSequencerNumFramesChanged(const uint32_t currentFrameIndex, const uint32_t numFrames);

	/**
	 * Called whenever the sequencer starts a drag action.
	 */
	void OnSequencerStartedDragging();

	/**
	 * Called whenever the sequencer ends a drag action.
	 */
	void OnSequencerEndedDragging();

private:
	/// A pointer to the input system to get input information.
	SubsystemPtr<InputSubsystem> _inputSubsystem = nullptr;

	/// A pointer to the camera system to get view information.
	SubsystemPtr<CameraSubsystem> _cameraSubsystem = nullptr;

	/// The currently active scene.
	std::unique_ptr<Scene> _scene = nullptr;

	/// All gizmos in the scene.
	std::shared_ptr<Gizmo> _gizmo;

	/// The handle to the mesh that is deformed.
	MeshHandle _deformedMeshHandle = InvalidHandle;

	/// The handle to the deformed cage.
	MeshHandle _deformedCageHandle = InvalidHandle;

	/// Executes all operations on a given mesh.
	std::shared_ptr<MeshOperationSystem> _meshOperationSystem = nullptr;

	/// Tool system that keeps track of all active and inactive tools.
	std::shared_ptr<ToolSystem> _toolSystem = nullptr;

	/// Shared thread pool between all objects.
	std::unique_ptr<ThreadPool> _threadPool = nullptr;

	/// Thread queue for tasks that should be executed explicitly on the main thread.
	std::unique_ptr<ThreadSafeQueue<FunctionWrapper>> _mainThreadQueue;

	/// A shared state that keeps all project data up-to-date.
	std::shared_ptr<ProjectData> _projectData = nullptr;

	/// Data that is computed during the weights calculation step. If the value is empty then we haven't finished
	/// the weights computation yet.
	DoubleBuffer<MeshComputeWeightsOperationResult> _weightsData;
	std::atomic<bool> _isComputingWeightsData { false };

	/// Data that is computed by the deformed mesh operation and contains the deformed mesh based on the input
	/// deformed cage for each sample from 1 to NumSamples.
	DoubleBuffer<MeshComputeDeformationOperationResult> _deformationData;
	std::atomic<bool> _isComputingDeformationData { false };

	/// Project model settings for UI bindings.
	std::shared_ptr<ProjectModelData> _projectModel = nullptr;

	/// The status bar at the bottom.
	std::shared_ptr<StatusBar> _statusBar = nullptr;

	/// The bar of all tools.
	std::shared_ptr<ToolBar> _toolBar = nullptr;

	/// The modal panel to create a new project.
	std::shared_ptr<NewProjectPanel> _newProjectPanel = nullptr;

	/// The modal panel to adjust the project settings (only input files).
	std::shared_ptr<ProjectSettingsPanel> _projectSettingsPanel = nullptr;

	/// The panel to set the project settings.
	std::shared_ptr<ProjectOptionsPanel> _projectOptionsPanel = nullptr;

	/// The start position of a rectangle selection drag.
	ImVec2 _selectionRectStartPosition;

	/// The end position of a rectangle selection drag.
	ImVec2 _selectionRectEndPosition;

	/// Currently active mesh transformation object.
	std::optional<MeshTransformation> _activeMeshTransformation;

	/// Stores the current active gizmo type to see if it changed.
	GizmoType _activeGizmoType = GizmoType::MaxNum;

	/// Optionally highlighted gizmo axis if we are hovering over it.
	GizmoAxis _highlightedGizmoAxis = GizmoAxis::X;

	/// The last highlighted vertex.
	VertexHandle _highlightedVertexHandle = VertexHandle();

	/// The currently highlighted edge.
	EdgeHandle _highlightedEdgeHandle = EdgeHandle();

	/// The currently highlighted polygon.
	FaceHandle _highlightedPolygonHandle = FaceHandle();

	/// Whether or not the gizmo has been highlighted.
	uint32_t _isGizmoHighlighted : 1;

	/// Whether or not the gizmo is being transformed by the user clicking on it.
	uint32_t _isGizmoTransformed : 1;

	/// Whether the mouse is currently dragging.
	uint32_t _isDragging : 1;

	/// Whenever we drag from any frame other than the last one we will be previewing the deformation process.
	/// This flag will be reset back to false when we start deforming the mesh again.
	uint32_t _isPreviewingSample : 1;

	/// Whether the mouse is being dragged for a rectangle selection.
	uint32_t _isSelectingRect : 1;

	/// Whether the mouse has been dragged during last mouse down.
	uint32_t _hasDragged : 1;
};
