#pragma once

#include <UI/UIPanel.h>
#include <UI/ProjectModel.h>
#include <Mesh/Operations/MeshWeightsParams.h>

class PolygonMesh;
class MeshOperationSystem;

struct ProjectOptionsPanelModel
{
	std::shared_ptr<ProjectModelData> _projectData = nullptr;
	std::shared_ptr<PolygonMesh> _deformableMesh = nullptr;
	std::shared_ptr<PolygonMesh> _cageMesh = nullptr;
};

class ProjectOptionsPanel final : public UIPanel<ProjectOptionsPanelModel>
{
public:
	ProjectOptionsPanel(const std::shared_ptr<ProjectModelData>& model,
		const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
		std::function<void (const bool)> influenceMapToggled,
		std::function<void ()> projectSettingsApplied);

	void Layout() override;

	void SetModel(const ProjectOptionsPanelModel& model) override;
	void SetModelData(const std::shared_ptr<ProjectModelData>& modelData);

	void SetDeformableMesh(const std::shared_ptr<PolygonMesh>& deformableMesh);
	void SetCageMesh(const std::shared_ptr<PolygonMesh>& cageMesh);

private:
	struct MeshElementsText
	{
		std::string _label;
		std::string _verticesCount;
		std::string _edgesCount;
		std::string _facesCount;
	};

private:
	void PushProjectSettingsTab();

	void UpdateAfterNewModel();

	static MeshElementsText GetMeshElementsText(const char* label,
		const std::size_t numVertices,
		const std::size_t numEdges,
		const std::size_t numFaces);

	static void PushMeshElementsUI(const MeshElementsText& text);

private:
	std::weak_ptr<MeshOperationSystem> _meshOperationSystem;
	ProjectModelData _modifiedProjectModel;
	std::function<void (const bool)> _influenceMapToggled;
	std::function<void ()> _projectSettingsApplied;

	uint32_t _selectedDeformationTypeIndex = -1;
	uint32_t _selectedWeightingSchemeIndex = -1;

#if WITH_SOMIGLIANA
	uint32_t _selectedBulgingTypeIndex = -1;
#endif
};
