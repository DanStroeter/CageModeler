#include <Editor/Editor.h>
#include <Editor/Scene.h>
#include <Input/InputSubsystem.h>
#include <Mesh/Operations/MeshOperationSystem.h>
#include <Mesh/Operations/MeshComputeDeformationOperation.h>
#include <Mesh/Operations/MeshExportInfluenceMapOperation.h>
#include <Mesh/Operations/MeshComputeInfluenceMapOperation.h>
#include <Mesh/Operations/MeshExportWeightsOperation.h>
#include <Mesh/Operations/MeshComputeWeightsOperation.h>
#include <Mesh/Operations/MeshExportOperation.h>
#include <Mesh/Operations/MeshLoadOperation.h>
#include <Mesh/Operations/MeshCageGeneration.h>
#include <Mesh/MeshLibrary.h>
#include <Navigation/CameraSubsystem.h>
#include <UI/UIStyle.h>
#include <UI/StatusBar.h>
#include <UI/ToolBar.h>
#include <UI/ProjectOptionsPanel.h>
#include <UI/ProjectSettingsPanel.h>
#include <UI/NewCagePanel.h>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp> // <- stellt glm::to_string bereit

#include <algorithm>
#include <chrono>
#include <cctype>
#include <thread>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <future>

#include <regex>

namespace
{
	constexpr auto VertexSelectionMinDistanceThresholdSq = 50.0f;
	constexpr auto EdgeSelectionMinDistanceThreshold = 20.0f;
	constexpr auto PolygonSelectionMinDistanceThreshold = 5.0f;

	/**
	 * Draws a selection rectangle on the screen when holding the LMB.
	 * @return A valid tuple of start and end screen space positions when the mouse has been released.
	 */
	void DrawSelectionRect(const ImVec2& startPosition, const ImVec2& endPosition)
	{
		const auto drawList = ImGui::GetForegroundDrawList();
		drawList->AddRect(startPosition, endPosition, ImGui::GetColorU32(IM_COL32(25, 175, 120, 255)));
		drawList->AddRectFilled(startPosition, endPosition, ImGui::GetColorU32(IM_COL32(25, 175, 120, 20)));
	}

	[[nodiscard]] inline TransformationAxis GetTransformationAxisFromGizmoAxis(const GizmoAxis axis)
	{
		switch (axis)
		{
			case GizmoAxis::X:
				return TransformationAxis::X;
			case GizmoAxis::Y:
				return TransformationAxis::Y;
			case GizmoAxis::Z:
				return TransformationAxis::Z;
			default:
				break;
		}

		CheckNoEntry("Invalid axis.");

		return TransformationAxis::X;
	}

	[[nodiscard]] inline TransformationType GetTransformationTypeFromGizmoType(const GizmoType type)
	{
		switch (type)
		{
			case GizmoType::Translate:
				return TransformationType::Translate;
			case GizmoType::Rotate:
				return TransformationType::Rotate;
			case GizmoType::Scale:
				return TransformationType::Scale;
			default:
				break;
		}

		CheckNoEntry("Invalid type.");

		return TransformationType::Translate;
	}

	[[nodiscard]] inline bool HasModifierKeysPressed(const SDL_Keymod modifierKeys)
	{
		return IsSet(modifierKeys, SDL_KMOD_LSHIFT) || IsSet(modifierKeys, SDL_KMOD_LALT) || IsSet(modifierKeys, SDL_KMOD_LGUI);
	}

	[[nodiscard]] std::string ToLower(std::string value)
	{
		std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
		return value;
	}

	struct EvaluationProjectConfig
	{
		std::string _name;
		std::string _coordinateType = "MVC";
		std::string _mesh;
		std::string _cage;
		std::string _deformedCage;
		std::optional<std::string> _embedding;
		std::optional<int32_t> _samples;
	};

	struct EvaluationConfig
	{
		std::string _timingsFile = "timings.txt";
		std::vector<EvaluationProjectConfig> _projects;
	};

	[[nodiscard]] std::optional<std::string> ExtractJsonStringValue(const std::string& objectText, const std::string& key)
	{
		const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
		std::smatch match;
		if (std::regex_search(objectText, match, pattern) && match.size() > 1)
		{
			return match[1].str();
		}
		return std::nullopt;
	}

	[[nodiscard]] std::optional<int32_t> ExtractJsonIntValue(const std::string& objectText, const std::string& key)
	{
		const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+)");
		std::smatch match;
		if (std::regex_search(objectText, match, pattern) && match.size() > 1)
		{
			return static_cast<int32_t>(std::stoi(match[1].str()));
		}
		return std::nullopt;
	}

	[[nodiscard]] std::vector<std::string> ExtractTopLevelObjects(const std::string& arrayText)
	{
		std::vector<std::string> objects;
		int32_t braceDepth = 0;
		bool isInString = false;
		std::size_t objectStart = std::string::npos;
		for (std::size_t i = 0; i < arrayText.size(); ++i)
		{
			const auto c = arrayText[i];
			const auto escaped = (i > 0 && arrayText[i - 1] == '\\');
			if (c == '"' && !escaped)
			{
				isInString = !isInString;
				continue;
			}
			if (isInString)
			{
				continue;
			}
			if (c == '{')
			{
				if (braceDepth == 0)
				{
					objectStart = i;
				}
				++braceDepth;
			}
			else if (c == '}')
			{
				--braceDepth;
				if (braceDepth == 0 && objectStart != std::string::npos)
				{
					objects.push_back(arrayText.substr(objectStart, i - objectStart + 1));
					objectStart = std::string::npos;
				}
			}
		}
		return objects;
	}

	[[nodiscard]] std::optional<EvaluationConfig> ParseEvaluationConfig(const std::string& text)
	{
		EvaluationConfig config;
		if (const auto timingsFile = ExtractJsonStringValue(text, "timingsFile"))
		{
			config._timingsFile = timingsFile.value();
		}
		const auto projectsPos = text.find("\"projects\"");
		if (projectsPos == std::string::npos)
		{
			return std::nullopt;
		}
		const auto arrayStart = text.find('[', projectsPos);
		if (arrayStart == std::string::npos)
		{
			return std::nullopt;
		}
		int32_t bracketDepth = 0;
		bool isInString = false;
		std::size_t arrayEnd = std::string::npos;
		for (std::size_t i = arrayStart; i < text.size(); ++i)
		{
			const auto c = text[i];
			const auto escaped = (i > 0 && text[i - 1] == '\\');
			if (c == '"' && !escaped)
			{
				isInString = !isInString;
				continue;
			}
			if (isInString)
			{
				continue;
			}
			if (c == '[')
			{
				++bracketDepth;
			}
			else if (c == ']')
			{
				--bracketDepth;
				if (bracketDepth == 0)
				{
					arrayEnd = i;
					break;
				}
			}
		}
		if (arrayEnd == std::string::npos)
		{
			return std::nullopt;
		}
		const auto projectsText = text.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
		for (const auto& objectText : ExtractTopLevelObjects(projectsText))
		{
			EvaluationProjectConfig project;
			project._name = ExtractJsonStringValue(objectText, "name").value_or("");
			project._coordinateType = ExtractJsonStringValue(objectText, "coordinateType").value_or("MVC");
			project._mesh = ExtractJsonStringValue(objectText, "mesh").value_or("");
			project._cage = ExtractJsonStringValue(objectText, "cage").value_or("");
			project._deformedCage = ExtractJsonStringValue(objectText, "deformedCage").value_or("");
			project._embedding = ExtractJsonStringValue(objectText, "embedding");
			project._samples = ExtractJsonIntValue(objectText, "samples");
			config._projects.push_back(std::move(project));
		}
		return config;
	}

	[[nodiscard]] std::optional<DeformationType> ParseDeformationType(const std::string& value)
	{
		static const std::unordered_map<std::string, DeformationType> mapping = {
			{ "mvc", DeformationType::MVC },
			{ "qmvc", DeformationType::QMVC },
			{ "harmonic", DeformationType::Harmonic },
			{ "bbw", DeformationType::BBW },
			{ "lbc", DeformationType::LBC },
			{ "mec", DeformationType::MEC },
			{ "mlc", DeformationType::MLC },
			{ "green", DeformationType::Green },
			{ "qgc", DeformationType::QGC },
			{ "somigliana", DeformationType::Somigliana },
			{ "pmvcserialoffset", DeformationType::PMVCSerialOffset },
			{ "pmvcserialnooffset", DeformationType::PMVCSerialNoOffset },
			{ "pmvcringoffset", DeformationType::PMVCRingOffset },
			{ "pmvcringnooffset", DeformationType::PMVCRingNoOffset },
			{ "pmvcalloffset", DeformationType::PMVCAllOffset },
			{ "pmvcallnooffset", DeformationType::PMVCAllNoOffset },
			{ "pmvccpuoffset", DeformationType::PMVCCpuOffset },
			{ "pmvccpunooffset", DeformationType::PMVCCpuNoOffset },
			{ "raytracing", DeformationType::Raytracing }
		};

		const auto it = mapping.find(ToLower(value));
		if (it == mapping.end())
		{
			return std::nullopt;
		}

		return it->second;
	}
}

Editor::Editor(const SubsystemPtr<InputSubsystem>& inputSubsystem,
	const SubsystemPtr<CameraSubsystem>& cameraSubsystem)
	: _inputSubsystem(inputSubsystem)
	, _cameraSubsystem(cameraSubsystem)
	, _isGizmoHighlighted(false)
	, _isGizmoTransformed(false)
	, _isDragging(false)
	, _isSelectingRect(false)
	, _hasDragged(false)
{
	inputSubsystem->RegisterInputActionMapping(InputActionMapping { "EditorClick", SDL_KMOD_NONE, SDL_BUTTON_LEFT, { }});
	inputSubsystem->RegisterInputActionEntry(InputActionEntry { "EditorClick",
		[this]<typename ParamsType>(ParamsType&& actionParams)
		{
			OnClicked(std::forward<ParamsType>(actionParams));
		}});
}

void Editor::Initialize(const std::shared_ptr<SceneRenderer>& sceneRenderer, const std::shared_ptr<CubemapManager>& cubemapRenderer, const std::shared_ptr<Raytracer>& raytracer)
{
	_scene = std::make_unique<Scene>(sceneRenderer);
	_cubemapRenderer = cubemapRenderer;
	_raytracer = raytracer;

	// Sets up all the scene lights before initializing the renderer. Hacky!
	CreateSceneLights();

	sceneRenderer->Initialize();

	_meshOperationSystem = std::make_shared<MeshOperationSystem>();
	_projectData = std::make_shared<ProjectData>();
	_threadPool = std::make_unique<ThreadPool>(2);
	_mainThreadQueue = std::make_unique<ThreadSafeQueue<FunctionWrapper>>();

	_toolSystem = std::make_shared<ToolSystem>();
	_toolSystem->SetSelectionChangedDelegate([this]<typename T>(T&& toolType) {
		OnToolSelectionChanged(std::forward<T>(toolType));
	});

	// Adjusts the UI style and set up all UI elements.
	UIStyle::SetStyle();
	SetUpUIElements();

	// Create the scene gizmo.
	const auto& viewInfo = _cameraSubsystem->GetCamera().GetViewInfo();
	_gizmo = std::make_shared<Gizmo>(*_scene);
	_gizmo->UpdateModelMatrix(viewInfo);

//#if BUILD_DEVELOPMENT
	_newProjectPanel = std::make_shared<NewProjectPanel>(_meshOperationSystem,
		[this] { OnNewProjectCancelled(); },
		[this] { OnNewProjectCreated(); });

	_projectModel->_deformationType = DeformationType::PMVCAllNoOffset;
	//_projectModel->_meshFilepath = "assets/meshes/tri.obj";
	//_projectModel->_cageFilepath = "assets/meshes/sphere_cages_triangulated.obj";
	_projectModel->_meshFilepath = "assets/meshes/armadilloman.obj";
	_projectModel->_cageFilepath = "assets/meshes/armadilloman_cages_triangulated.obj";
	_projectModel->_embeddingFilepath = "assets/meshes/bishop_cages_triangulated_embedding.msh";
	_projectModel->_deformedCageFilepath = "assets/meshes/armadilloman_cages_triangulated_deformed_8.obj";
	_newProjectPanel->SetModel(_projectModel);
	_projectOptionsPanel->SetModelData(_projectModel);

	StartEvaluation();
	//OnNewProjectCreated();
//#endif
}

void Editor::StartEvaluation()
{
	const auto kEvaluationRoot = std::filesystem::path("evaluation");
	constexpr auto kEvaluationConfig = "projects.json";

	const auto evaluationRoot = std::filesystem::absolute(kEvaluationRoot);
	const auto configPath = evaluationRoot / kEvaluationConfig;
	if (!std::filesystem::exists(configPath))
	{
		LOG_WARN("Evaluation config '{}' does not exist. Skipping evaluation run.", configPath.string());
		return;
	}

	std::ifstream configFile(configPath);
	if (!configFile.is_open())
	{
		LOG_ERROR("Unable to open evaluation config '{}'.", configPath.string());
		return;
	}

	const std::string configContent((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
	const auto parsedConfig = ParseEvaluationConfig(configContent);
	if (!parsedConfig.has_value())
	{
		LOG_ERROR("Failed to parse evaluation config '{}'.", configPath.string());
		return;
	}

	const auto timingOutputPath = evaluationRoot / parsedConfig->_timingsFile;
	std::ofstream timingOutput(timingOutputPath, std::ios::out | std::ios::trunc);
	if (!timingOutput.is_open())
	{
		LOG_ERROR("Unable to open timing output file '{}'.", timingOutputPath.string());
		return;
	}

#ifdef NDEBUG
	constexpr auto buildType = "Release";
#else
	constexpr auto buildType = "Debug/Development";
#endif

	timingOutput << "BuildType=" << buildType << "\n";
	timingOutput << "ProjectCount=" << parsedConfig->_projects.size() << "\n";

	
	_isEvaluationMode = true;

	for (std::size_t i = 0; i < parsedConfig->_projects.size(); ++i)
	{
		const auto& project = parsedConfig->_projects[i];
		if (project._mesh.empty() || project._cage.empty() || project._deformedCage.empty())
		{
			LOG_WARN("Skipping project {} due to missing required file keys (mesh/cage/deformedCage).", i);
			continue;
		}

		const auto deformationType = ParseDeformationType(project._coordinateType);
		if (!deformationType.has_value())
		{
			LOG_WARN("Skipping project {} due to unsupported coordinateType '{}'.", i, project._coordinateType);
			continue;
		}

		const auto projectName = project._name.empty() ? (std::string("project_") + std::to_string(i)) : project._name;
		const auto projectOutputDir = evaluationRoot / projectName;
		std::filesystem::create_directories(projectOutputDir);

		_projectModel->_deformationType = *deformationType;
		_projectModel->_meshFilepath = evaluationRoot / project._mesh;
		_projectModel->_cageFilepath = evaluationRoot / project._cage;
		_projectModel->_deformedCageFilepath = evaluationRoot / project._deformedCage;

		const auto start = std::chrono::steady_clock::now();

		_projectCreationFailed.store(false, std::memory_order_seq_cst);

		auto completionPromise = std::make_shared<std::promise<void>>();
		auto completionFuture = completionPromise->get_future();

		OnNewProjectCreated(completionPromise);

		while (completionFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
		{
			if (_projectCreationFailed.load(std::memory_order_seq_cst))
			{
				break;
			}

			FunctionWrapper mainThreadFunction;
			while (_mainThreadQueue->TryPop(mainThreadFunction))
			{
				mainThreadFunction();
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		FunctionWrapper mainThreadFunction;
		while (_mainThreadQueue->TryPop(mainThreadFunction))
		{
			mainThreadFunction();
		}

		if (_projectCreationFailed.load(std::memory_order_seq_cst))
		{
			LOG_WARN("Evaluation project '{}' failed.", projectName);
			timingOutput << projectName << "," << project._coordinateType << ",FAILED\n";
			continue;
		}

		const auto end = std::chrono::steady_clock::now();
		const auto elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();

		ExportWeights(projectOutputDir / "weights.dmat");
		ExportDeformedCage(projectOutputDir / "deformed_cage.obj");
		ExportDeformedMeshes(projectOutputDir / "deformed_mesh.obj");

		timingOutput << projectName << "," << project._coordinateType << "," << elapsedMs << '\n';
		LOG_INFO("Evaluation project '{}' finished in {} ms.", projectName, elapsedMs);
	}

	_isEvaluationMode = false;
}

void Editor::CreateSceneLights() const
{
	_scene->AddLightSource(PointLight(glm::vec3(4.0f, 4.0f, 4.0f), 0.75f));
	_scene->AddLightSource(PointLight(glm::vec3(-3.0f, -7.5f, 6.0f), 1.08f));
	_scene->AddLightSource(PointLight(glm::vec3(0.0f, 8.0f, 6.0f), 0.92f));
	_scene->AddLightSource(PointLight(glm::vec3(8.0f, -8.0f, 8.0f), 1.52f));
}

void Editor::RecordUI()
{
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New Project..."))
			{
				_newProjectPanel = std::make_shared<NewProjectPanel>(_meshOperationSystem,
					[this] { OnNewProjectCancelled(); },
					[this] { OnNewProjectCreated(); });
				_newProjectPanel->Present();
			}

			const auto weightsData = _weightsData.LockRead();
			const auto hasComputedWeights = (weightsData->_weights.cols() > 0 &&
				!_isComputingWeightsData.load(std::memory_order_relaxed) &&
				!_isComputingDeformationData.load(std::memory_order_relaxed));

			ImGui::BeginDisabled(!hasComputedWeights);
			{
				if (ImGui::BeginMenu("Export"))
				{
					if (ImGui::MenuItem("Current Frame Mesh...", nullptr))
					{
						const auto filepath = UIHelpers::PresentExportFilePopup({ { "Mesh (.obj)", "obj" } }, "Untitled.obj");

						if (filepath.has_value())
						{
							ExportCurrentDeformedMesh(filepath.value());
						}
					}

					if (ImGui::MenuItem("Meshes...", nullptr))
					{
						const auto filepath = UIHelpers::PresentExportFilePopup({ { "Mesh (.obj)", "obj" } }, "Untitled.obj");

						if (filepath.has_value())
						{
							ExportDeformedMeshes(filepath.value());
						}
					}

					if (ImGui::MenuItem("Deformed Cage...", nullptr))
					{
						const auto filepath = UIHelpers::PresentExportFilePopup({ { "Mesh (.obj)", "obj" } }, "Untitled.obj");

						if (filepath.has_value())
						{
							ExportDeformedCage(filepath.value());
						}
					}

					if (ImGui::MenuItem("Influence Color Map...", nullptr))
					{
						const auto filepath = UIHelpers::PresentExportFilePopup({ { "Mesh (.obj)", "obj" } }, "Untitled.obj");

						if (filepath.has_value())
						{
							ExportInfluenceColorMap(filepath.value());
						}
					}

					ImGui::BeginDisabled(_projectModel->_deformationType != DeformationType::BBW && _projectModel->_deformationType != DeformationType::LBC);
					{
						if (ImGui::MenuItem("Weights...", nullptr))
						{
							const auto filepath = UIHelpers::PresentExportFilePopup({ { "Weights (.dmat)", "dmat" } }, "Untitled.dmat");

							if (filepath.has_value())
							{
								ExportWeights(filepath.value());
							}
						}
					}
					ImGui::EndDisabled();

					ImGui::EndMenu();
				}
			}
			ImGui::EndDisabled();

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Edit"))
		{
			if (ImGui::MenuItem("Project Settings..."))
			{
				_projectSettingsPanel = std::make_shared<ProjectSettingsPanel>(_projectModel,
					_meshOperationSystem,
					[this] { OnProjectSettingsCancelled(); },
					[this] { OnProjectSettingsApplied(); });
				_projectSettingsPanel->Present();
			}

			if (ImGui::MenuItem("New Cage..."))
			{	
				_newCagePanel = std::make_shared<NewCagePanel>(_projectModel,
					_meshOperationSystem,
					[this] { OnNewCageCancelled(); },
					[this] { OnNewCageCreated(); });
				_newCagePanel->Present();
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}

	_statusBar->Layout();
	_toolBar->Layout();
	_projectOptionsPanel->Layout();

	if (_newProjectPanel != nullptr)
	{
		_newProjectPanel->Layout();
	}

	if (_projectSettingsPanel != nullptr)
	{
		_projectSettingsPanel->Layout();
	}

	if (_newCagePanel != nullptr)
	{
		_newCagePanel->Layout();
	}

	// If we are making a rectangle selection on the screen we want to do it before we process any mesh selection, so we can use the data.
	if (_isSelectingRect)
	{
		DrawSelectionRect(_selectionRectStartPosition, _selectionRectEndPosition);
	}
}

void Editor::Update(const double deltaTime)
{
	FunctionWrapper mainThreadFunction;
	while (_mainThreadQueue->TryPop(mainThreadFunction))
	{
		mainThreadFunction();
	}

	if (_newProjectPanel != nullptr && _newProjectPanel->IsModalPanelVisible())
	{
		return;
	}

	if (_projectSettingsPanel != nullptr && _projectSettingsPanel->IsModalPanelVisible())
	{
		return;
	}

	if (_newCagePanel != nullptr && _newCagePanel->IsModalPanelVisible())
	{
		return;
	}

	// Update the matrices of all gizmos.
	const auto& camera = _cameraSubsystem->GetCamera();
	const auto& viewInfo = camera.GetViewInfo();
	_gizmo->UpdateModelMatrix(viewInfo);

	// Shoot a ray to the currently active gizmo type to determine the selection.
	const auto activeGizmoType = _toolBar->GetActiveGizmoType();

	if (activeGizmoType != GizmoType::MaxNum)
	{
		UpdateGizmoSelection(viewInfo, activeGizmoType);
	}

	if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
	{
		return;
	}

	// Processes the current actions and selects elements from the mesh.
	UpdateMeshSelection(viewInfo);

	// We are okay to update all proxies now since they have been already marked dirty.
	_scene->UpdateDirtyRenderProxies();
}

void Editor::UpdateMeshSelection(const ViewInfo& viewInfo)
{
	if (_inputSubsystem->GetMouseButtonsState() == MouseButtonsState::LeftPressed ||
		_isDragging ||
		_isSelectingRect)
	{
		return;
	}

	const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
	const auto mousePosition = _inputSubsystem->GetMousePosition();

	// If we have the gizmo selected or highlighted we don't want to highlight any mesh elements.
	if (_isGizmoHighlighted || _isGizmoTransformed)
	{
		if (_statusBar->GetActiveSelectionType() == SelectionType::Vertex)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Vertex>();
			selection.UnhighlightAll();
		}
		else if (_statusBar->GetActiveSelectionType() == SelectionType::Edge)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Edge>();
			selection.UnhighlightAll();
		}
		else if (_statusBar->GetActiveSelectionType() == SelectionType::Polygon)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Polygon>();
			selection.UnhighlightAll();
		}

		return;
	}

	if (_statusBar->GetActiveSelectionType() == SelectionType::Vertex)
	{
		// If we are doing vertex selection first cache the projected vertices into screen space.
		deformedCageMesh->MarkCachedGeometryDirty();
		deformedCageMesh->CacheProjectedPointsWorldToScreen(viewInfo);

		auto selection = deformedCageMesh->GetSelection<SelectionType::Vertex>();
		const auto hit = deformedCageMesh->QueryClosestPointScreenSpace(viewInfo,
			mousePosition,
			VertexSelectionMinDistanceThresholdSq);

		if (hit.has_value())
		{
			if (_highlightedVertexHandle != hit->_vertexHandle && _highlightedVertexHandle.is_valid())
			{
				selection.Unhighlight(_highlightedVertexHandle);
			}

			selection.Highlight(hit->_vertexHandle);

			_highlightedVertexHandle = hit->_vertexHandle;
		}
		else
		{
			if (_highlightedVertexHandle.is_valid())
			{
				selection.Unhighlight(_highlightedVertexHandle);
			}

			_highlightedVertexHandle = VertexHandle();
		}
	}
	else if (_statusBar->GetActiveSelectionType() == SelectionType::Edge)
	{
		// If we are doing vertex selection first cache the projected vertices into screen space.
		deformedCageMesh->MarkCachedGeometryDirty();
		deformedCageMesh->CacheProjectedPointsWorldToScreen(viewInfo);

		auto selection = deformedCageMesh->GetSelection<SelectionType::Edge>();
		const auto hit = deformedCageMesh->QueryClosestEdgeScreenSpace(viewInfo,
			mousePosition,
			EdgeSelectionMinDistanceThreshold);

		if (hit.has_value())
		{
			if (_highlightedEdgeHandle != hit->_edgeHandle && _highlightedEdgeHandle.is_valid())
			{
				selection.Unhighlight(_highlightedEdgeHandle);
			}

			selection.Highlight(hit->_edgeHandle);

			_highlightedEdgeHandle = hit->_edgeHandle;
		}
		else
		{
			if (_highlightedEdgeHandle.is_valid())
			{
				selection.Unhighlight(_highlightedEdgeHandle);
			}

			_highlightedEdgeHandle = EdgeHandle();
		}
	}
	else if (_statusBar->GetActiveSelectionType() == SelectionType::Polygon)
	{
		auto selection = deformedCageMesh->GetSelection<SelectionType::Polygon>();
		const auto worldRay = viewInfo.DeprojectScreenToWorldRay(mousePosition);
		const auto hit = deformedCageMesh->QueryRayHit(worldRay);

		if (hit._polyHandle.is_valid())
		{
			if (_highlightedPolygonHandle != hit._polyHandle && _highlightedPolygonHandle.is_valid())
			{
				selection.Unhighlight(_highlightedPolygonHandle);
			}

			selection.Highlight(hit._polyHandle);

			_highlightedPolygonHandle = hit._polyHandle;
		}
		else
		{
			selection.UnhighlightAll();

			_highlightedPolygonHandle = FaceHandle();
		}
	}
}

void Editor::SetUpUIElements()
{
	// Create both pointers in the project model.
	_projectModel = std::make_shared<ProjectModelData>();

	_statusBar = std::make_shared<StatusBar>(_meshOperationSystem);
	_projectOptionsPanel = std::make_shared<ProjectOptionsPanel>(_projectModel,
		_meshOperationSystem,
		[this](const bool shouldCompute) { OnComputeInfluenceColorMap(shouldCompute); },
		[this] { OnNewProjectCreated(); });
	_toolBar = std::make_shared<ToolBar>(_inputSubsystem, _meshOperationSystem, _toolSystem);
}

void Editor::OnNewProjectCancelled()
{
	_newProjectPanel = nullptr;
}

void Editor::OnProjectSettingsApplied()
{
	// Manually update the model here on the project options panel, otherwise the temporary "modified" state will not update.
	_projectOptionsPanel->SetModelData(_projectModel);

	// Re-create the project since we changed the data model.
	OnNewProjectCreated();
}

void Editor::OnNewCageCreated()
{
	// Remove the current cage
	if (_deformedCageHandle != InvalidHandle)
	{
		_scene->RemoveMesh(_deformedCageHandle);
		_deformedCageHandle = InvalidHandle;
	}
	if (_deformedMeshHandle != InvalidHandle)
	{
		_scene->RemoveMesh(_deformedMeshHandle);
		_deformedMeshHandle = InvalidHandle;
	}

	_threadPool->Submit([this](){

		// Update _projectModel if user creates a new project
		if(_newCagePanel != nullptr) {
			auto _panelModel = _newCagePanel->GetModel();
			_projectModel = std::make_shared<ProjectModelData>(*_panelModel);
		}
		
		auto result = GenerateCage(_projectModel->_cageGenerationType, _projectData->_mesh, _projectData->_mesh, _newCagePanel->GetSetting());

		if (result.HasError())
		{
			LOG_DEBUG("Error creating new cage.");
			return;
		}

		const auto translation = glm::translate(glm::mat4(1.0f), glm::vec3(_projectData->_centerOffset));
		const auto scale = glm::scale(glm::mat4(1.0f), glm::vec3(_projectData->_scalingFactor));
		const auto newModelMatrix = scale * translation;	
		
		_deformedMeshHandle = _scene->AddMesh(_projectData->_mesh._vertices, _projectData->_mesh._faces);
		const auto mesh = _scene->GetMesh(_deformedMeshHandle);
		mesh->SetModelMatrix(newModelMatrix);

		_deformedCageHandle = _scene->AddCage(result.GetValue()._cage._vertices, result.GetValue()._cage._faces);
		const auto cageMesh = _scene->GetMesh(_deformedCageHandle);
		cageMesh->SetModelMatrix(newModelMatrix);

		_isComputingWeightsData.store(true, std::memory_order_seq_cst);

		_projectData->_cage = result.GetValue()._cage;

		auto weightsResult = ComputeCageWeights(*_projectData);

		_weightsData.Update(std::move(weightsResult.GetValue()._skinningMatrix),
			std::move(weightsResult.GetValue()._weights),
			std::move(weightsResult.GetValue()._interpolatedWeights),
			std::move(weightsResult.GetValue()._psi),
			std::move(weightsResult.GetValue()._psiTri),
			std::move(weightsResult.GetValue()._psiQuad));

		_isComputingWeightsData.store(false, std::memory_order_seq_cst);

		if (_newCagePanel != nullptr)
		{
			_newCagePanel->Dismiss();
			_newCagePanel = nullptr;
		}
	});
	return;
}

void Editor::OnNewProjectCreated(const std::shared_ptr<std::promise<void>>& completionPromise)
{
	LOG_DEBUG("Set Cage and Mesh");

	_projectCreationFailed.store(false, std::memory_order_seq_cst);

	if (_projectModel->CheckMissingFiles())
	{
		_statusBar->SetError("Unable to load all files, check if some of them are missing.");
		_projectCreationFailed.store(true, std::memory_order_seq_cst);
		return;
	}

	_isComputingWeightsData.store(true, std::memory_order_seq_cst);
	_isComputingDeformationData.store(false, std::memory_order_seq_cst);

	auto projectModelSnapshot = std::make_shared<ProjectModelData>(*_projectModel);
	if (_newProjectPanel != nullptr)
	{
		projectModelSnapshot = std::make_shared<ProjectModelData>(*_newProjectPanel->GetModel());
	}

	const bool isEvaluationMode = _isEvaluationMode;

	_threadPool->Submit([this, completionPromise, projectModelSnapshot, isEvaluationMode]()
	{
		const auto fail = [this]()
		{
			_isComputingWeightsData.store(false, std::memory_order_seq_cst);
			_isComputingDeformationData.store(false, std::memory_order_seq_cst);
			_projectCreationFailed.store(true, std::memory_order_seq_cst);
		};

		auto projectResult = _meshOperationSystem->ExecuteOperation<MeshLoadOperation>(
			projectModelSnapshot->_deformationType,
			projectModelSnapshot->_LBCWeightingScheme,
			projectModelSnapshot->_meshFilepath.value(),
			projectModelSnapshot->_cageFilepath.value_or(std::filesystem::path()),
			projectModelSnapshot->_deformedCageFilepath,
			projectModelSnapshot->_weightsFilepath,
			projectModelSnapshot->_embeddingFilepath,
			projectModelSnapshot->_parametersFilepath,
			projectModelSnapshot->_numBBWSteps,
			projectModelSnapshot->_numSamples,
			projectModelSnapshot->_scalingFactor,
			projectModelSnapshot->_interpolateWeights,
			projectModelSnapshot->_findOffset,
			projectModelSnapshot->_noOffset,
			projectModelSnapshot->_somigNu,
			projectModelSnapshot->_somiglianaDeformer);

		if (projectResult.HasError())
		{
			_mainThreadQueue->Push([this, error = std::move(projectResult.GetError())]() mutable
			{
				_statusBar->SetError(std::move(error));
			});

			fail();
			return;
		}

		auto projectData = projectResult.GetValue();

		using WeightsResult = decltype(ComputeCageWeights(*projectData));
		std::future<WeightsResult> future;

		if (DeformationTypeHelpers::IsPMVC(projectModelSnapshot->_deformationType))
		{
			auto promise = std::make_shared<std::promise<WeightsResult>>();
			future = promise->get_future();

			_mainThreadQueue->Push([this, projectData, promise]() mutable
			{
				try
				{
					_cubemapRenderer->SetCage(projectData->_cage);
					_cubemapRenderer->SetMesh(projectData->_mesh);
					_cubemapRenderer->Initialize();
					promise->set_value(_cubemapRenderer->ComputeCoordinates(projectData->_deformationType));
				}
				catch (...)
				{
					promise->set_exception(std::current_exception());
				}
			});
		}
		else if (projectModelSnapshot->_deformationType == DeformationType::Raytracing)
		{
			auto promise = std::make_shared<std::promise<WeightsResult>>();
			future = promise->get_future();

			_mainThreadQueue->Push([this, projectData, promise]() mutable
			{
				try
				{
					_raytracer->SetCage(projectData->_cage);
					_raytracer->SetMesh(projectData->_mesh);
					_raytracer->Initialize();
					promise->set_value(_raytracer->ComputeCoordinates());
				}
				catch (...)
				{
					promise->set_exception(std::current_exception());
				}
			});
		}
		else
		{
			std::promise<WeightsResult> promise;
			future = promise.get_future();
			promise.set_value(ComputeCageWeights(*projectData));
		}

		auto weightsResult = future.get();
		/*try
		{
			weightsResult = future.get();
		}
		catch (const std::exception& e)
		{
			_mainThreadQueue->Push([this, msg = std::string(e.what())]() mutable
			{
				_statusBar->SetError(std::move(msg));
			});

			fail();
			return;
		}
		catch (...)
		{
			_mainThreadQueue->Push([this]()
			{
				_statusBar->SetError("Unknown error during main-thread weight computation.");
			});

			fail();
			return;
		}*/

		if (weightsResult.HasError())
		{
			_mainThreadQueue->Push([this, error = std::move(weightsResult.GetError())]() mutable
			{
				_statusBar->SetError(std::move(error));
			});

			fail();
			return;
		}

		_isComputingDeformationData.store(true, std::memory_order_seq_cst);
		_isComputingWeightsData.store(false, std::memory_order_seq_cst);

		_weightsData.Update(std::move(weightsResult.GetValue()._skinningMatrix),
			std::move(weightsResult.GetValue()._weights),
			std::move(weightsResult.GetValue()._interpolatedWeights),
			std::move(weightsResult.GetValue()._psi),
			std::move(weightsResult.GetValue()._psiTri),
			std::move(weightsResult.GetValue()._psiQuad));

		const auto& mesh = projectData->_mesh;
		const auto& cage = projectData->_cage;
		const auto& defCage = projectData->_deformedCage;

		LOG_DEBUG("MESH vertices: {} x {}", mesh._vertices.rows(), mesh._vertices.cols());
		LOG_DEBUG("CAGE vertices: {} x {}", cage._vertices.rows(), cage._vertices.cols());
		LOG_DEBUG("DEF CAGE vertices: {} x {}", defCage._vertices.rows(), defCage._vertices.cols());

		auto deformedMeshResult = ComputeDeformedMesh(projectData->_mesh,
			projectData->_cage,
			projectData->_deformedCage,
			projectData->_deformationType,
			projectData->_LBCWeightingScheme,
			projectData->_somiglianaDeformer,
			projectData->_modelVerticesOffset,
			projectData->_numSamples,
			projectData->CanInterpolateWeights());

		if (deformedMeshResult.HasError())
		{
			_mainThreadQueue->Push([this, error = std::move(deformedMeshResult.GetError())]() mutable
			{
				_statusBar->SetError(std::move(error));
			});

			fail();
			return;
		}

		_deformationData.Update(std::move(deformedMeshResult.GetValue()._vertexData));

		if (isEvaluationMode)
		{
			// Evaluation/offscreen mode:
			// keep computed data, skip all scene/render proxy churn.
			_projectData = projectData;

			_isComputingDeformationData.store(false, std::memory_order_seq_cst);

			if (completionPromise != nullptr)
			{
				try
				{
					completionPromise->set_value();
				}
				catch (const std::future_error&)
				{
				}
			}

			return;
		}

		_mainThreadQueue->Push([this, projectData, completionPromise]() mutable
		{
			const auto& viewInfo = _cameraSubsystem->GetCamera().GetViewInfo();
			_gizmo->SetPosition(viewInfo, glm::vec3(0.0f));

			if (_deformedMeshHandle != InvalidHandle)
			{
				_scene->RemoveMesh(_deformedMeshHandle);
				_deformedMeshHandle = InvalidHandle;
			}

			if (_deformedCageHandle != InvalidHandle)
			{
				_scene->RemoveMesh(_deformedCageHandle);
				_deformedCageHandle = InvalidHandle;
			}

			_projectData = projectData;

			const auto translation = glm::translate(glm::mat4(1.0f), glm::vec3(_projectData->_centerOffset));
			const auto scale = glm::scale(glm::mat4(1.0f), glm::vec3(_projectData->_scalingFactor));
			const auto newModelMatrix = scale * translation;

			_deformedMeshHandle = _scene->AddMesh(_projectData->_mesh._vertices, _projectData->_mesh._faces);
			const auto deformedMesh = _scene->GetMesh(_deformedMeshHandle);
			deformedMesh->SetModelMatrix(newModelMatrix);

			_deformedCageHandle = _scene->AddCage(_projectData->_deformedCage._vertices, _projectData->_deformedCage._faces);
			const auto cageMesh = _scene->GetMesh(_deformedCageHandle);
			cageMesh->SetModelMatrix(newModelMatrix);

			const auto renderInfluenceMap = _projectModel->CanRenderInfluenceMap();
			if (renderInfluenceMap)
			{
				UpdateMeshVertexColors(renderInfluenceMap);
			}

			_projectOptionsPanel->SetDeformableMesh(_scene->GetMesh(_deformedMeshHandle));
			_projectOptionsPanel->SetCageMesh(_scene->GetMesh(_deformedCageHandle));

			_toolBar->SetModel(std::make_shared<ToolBarModel>(_projectData));

			_statusBar->SetModel(std::make_shared<StatusBarModel>(_projectData,
				[this]<typename T>(T && selectionType) { OnSelectionTypeChanged(std::forward<T>(selectionType)); },
				[this](const uint32_t newFrameIndex) { OnSequencerFrameIndexChanged(newFrameIndex); },
				[this](const uint32_t frameIndex, const uint32_t numFrames) { OnSequencerNumFramesChanged(frameIndex, numFrames); },
				[this]() { OnSequencerStartedDragging(); },
				[this]() { OnSequencerEndedDragging(); }));

			UpdateDeformedMeshPositionsFromDeformationData();

			{
				auto bvhBuilder = _scene->BeginGeometryBVH();
				bvhBuilder.AddGeometry(_deformedMeshHandle);
				bvhBuilder.AddGeometry(_deformedCageHandle);
			}

			if (_newProjectPanel != nullptr)
			{
				_newProjectPanel->Dismiss();
				_newProjectPanel = nullptr;
			}

			if (_projectSettingsPanel != nullptr)
			{
				_projectSettingsPanel->Dismiss();
				_projectSettingsPanel = nullptr;
			}

			_isComputingDeformationData.store(false, std::memory_order_seq_cst);

			if (completionPromise != nullptr)
			{
				try
				{
					completionPromise->set_value();
				}
				catch (const std::future_error&)
				{
				}
			}
		});
	});
}

void Editor::OnProjectSettingsCancelled()
{
	_projectSettingsPanel = nullptr;
}

void Editor::OnNewCageCancelled()
{
	_newCagePanel = nullptr;
}

void Editor::UpdateGizmoSelection(const ViewInfo& viewInfo, const GizmoType activeGizmoType)
{
	const auto mousePosition = _inputSubsystem->GetMousePosition();
	const auto closestHit = _gizmo->QueryRayHit(viewInfo, activeGizmoType, mousePosition);

	if (closestHit.has_value())
	{
		if (!_isGizmoHighlighted || _highlightedGizmoAxis != closestHit->_axis)
		{
			_gizmo->SetHighlighted(activeGizmoType, _highlightedGizmoAxis, false);

			_highlightedGizmoAxis = closestHit->_axis;
			_isGizmoHighlighted = true;

			_gizmo->SetHighlighted(activeGizmoType, closestHit->_axis, true);
		}
	}
	else
	{
		// If we didn't hit anything, but we also have had something highlighted, then unhighlight.
		if (_isGizmoHighlighted && !_isGizmoTransformed)
		{
			_isGizmoHighlighted = false;

			_gizmo->SetHighlighted(activeGizmoType, _highlightedGizmoAxis, false);
		}
	}
}

void Editor::OnClicked(const InputActionParams& actionParams)
{
	if ((_newProjectPanel != nullptr && _newProjectPanel->IsModalPanelVisible()) ||
		(_projectSettingsPanel != nullptr && _projectSettingsPanel->IsModalPanelVisible()) ||
		ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
	{
		return;
	}

	// If we don't have a valid mesh.
	if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
	{
		return;
	}

	// If we are computing the weights of a new mesh we don't want to be able to do anything until we have it ready.
	if (_isComputingWeightsData.load(std::memory_order_relaxed))
	{
		return;
	}

	if (actionParams._keyState == KeyState::KeyDown)
	{
		OnMouseClickPressed(actionParams);
	}
	else if (actionParams._keyState == KeyState::KeyPressed)
	{
		OnMouseMoved(actionParams);
	}
	else if (actionParams._keyState == KeyState::KeyUp)
	{
		OnMouseClickReleased(actionParams);
	}
}

void Editor::OnMouseClickPressed(const InputActionParams& actionParams)
{
	// Update the camera focus point if we have hit the cage or any other object in the scene.
	// const auto currentMouseRay = viewInfo.DeprojectScreenToWorldRay(mousePosition);
	// const auto meshHitResult = _scene->QueryClosestMesh(currentMouseRay);
	//
	// if (meshHitResult.has_value())
	// {
	// 	camera.SetPointOfInterest(meshHitResult->_worldPosition);
	// }

	const auto& camera = _cameraSubsystem->GetCamera();
	const auto& viewInfo = camera.GetViewInfo();
	const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();
	const auto mousePosition = _inputSubsystem->GetMousePosition();

	if (_isGizmoHighlighted)
	{
		_isGizmoTransformed = true;

		_activeMeshTransformation = MeshTransformation(viewInfo,
			_scene->GetMesh(_deformedCageHandle),
			_statusBar->GetActiveSelectionType(),
			GetTransformationTypeFromGizmoType(_activeGizmoType),
			GetTransformationAxisFromGizmoAxis(_highlightedGizmoAxis),
			_gizmo->GetMatrix(),
			mousePosition,
			prevMousePosition);

		// We bail early here otherwise we will end up selecting a vertex.
		return;
	}

	const auto hasValidHighlight = _highlightedVertexHandle.is_valid() || _highlightedEdgeHandle.is_valid() || _highlightedPolygonHandle.is_valid();
	const auto hasActiveSelection = hasValidHighlight || _isGizmoHighlighted || _isDragging;
	const auto modifierKeys = _inputSubsystem->GetKeyModifiers();
	const auto hasModifierKeys = IsSet(modifierKeys, SDL_KMOD_LSHIFT) || IsSet(modifierKeys, SDL_KMOD_LALT) || IsSet(modifierKeys, SDL_KMOD_LGUI);

	// If we are making a rectangle selection on the screen we want to do it before we process any mesh selection, so we can use the data.
	if (!hasActiveSelection && !hasModifierKeys)
	{
		_selectionRectStartPosition = ImVec2(mousePosition.x, mousePosition.y);
		_selectionRectEndPosition = _selectionRectStartPosition;
		_isSelectingRect = true;
	}
}

void Editor::OnMouseMoved(const InputActionParams& actionParams)
{
	const auto &camera = _cameraSubsystem->GetCamera();
	const auto &viewInfo = camera.GetViewInfo();
	const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();
	const auto mousePosition = _inputSubsystem->GetMousePosition();

	// If we are doing a rectangle selection skip anything else.
	if (_isSelectingRect)
	{
		_selectionRectEndPosition = ImVec2(mousePosition.x, mousePosition.y);

		return;
	}

	// If we have dragged the mouse then we mark it as dragged until mouse up event.
	const auto mouseDelta = _inputSubsystem->GetMousePosition() - _inputSubsystem->GetPreviousMousePosition();
	_isDragging |= (_isGizmoHighlighted && (glm::length2(mouseDelta) > Epsilon));
	_hasDragged |= _isDragging;

	if (_activeMeshTransformation.has_value() && _isDragging && !HasModifierKeysPressed(actionParams._modifierKeys))
	{
		// First transform the gizmo based on the mouse input.
		_activeMeshTransformation->Transform(viewInfo,
			mousePosition,
			prevMousePosition);

		ResetGizmoPositionFromSelection(viewInfo);
	}
}

void Editor::OnMouseClickReleased(const InputActionParams& actionParams)
{
	_activeMeshTransformation.reset();

	const auto& camera = _cameraSubsystem->GetCamera();
	const auto& viewInfo = camera.GetViewInfo();
	const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();
	const auto mousePosition = _inputSubsystem->GetMousePosition();

	// If we are doing a rectangle selection skip anything else.
	if (_isSelectingRect)
	{
		// If we are doing selection first cache the projected vertices into screen space.
		const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
		deformedCageMesh->CacheProjectedPointsWorldToScreen(viewInfo);

		const auto selectionType = _statusBar->GetActiveSelectionType();
		const glm::vec2 rectMin(std::min(_selectionRectStartPosition.x, _selectionRectEndPosition.x), std::min(_selectionRectStartPosition.y, _selectionRectEndPosition.y));
		const glm::vec2 rectMax(std::max(_selectionRectStartPosition.x, _selectionRectEndPosition.x), std::max(_selectionRectStartPosition.y, _selectionRectEndPosition.y));

		if (selectionType == SelectionType::Vertex)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Vertex>();
			selection.SelectInRectangle(viewInfo, rectMin, rectMax);
		}
		else if (selectionType == SelectionType::Edge)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Edge>();
			selection.SelectInRectangle(viewInfo, rectMin, rectMax);
		}
		else if (selectionType == SelectionType::Polygon)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Polygon>();
			selection.SelectInRectangle(viewInfo, rectMin, rectMax);
		}

		_isSelectingRect = false;

		ResetGizmoPositionFromSelection(viewInfo);

		return;
	}

	if (_isGizmoTransformed && _hasDragged && !HasModifierKeysPressed(actionParams._modifierKeys))
	{
		if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
		{
			return;
		}

		// Re-compute the deformed mesh and update the render proxy.
		_threadPool->Submit([this,
			mesh = _scene->GetMesh(_deformedMeshHandle)->CopyAsEigen(),
			cage = _projectData->_cage,
			deformedMesh = _scene->GetMesh(_deformedCageHandle)->CopyAsEigen(),
			somiglianaDeformer = _projectData->_somiglianaDeformer,
			deformationType = _projectData->_deformationType,
			weightingScheme = _projectData->_LBCWeightingScheme,
			modelVerticesOffset = _projectData->_modelVerticesOffset,
			numSamples = _projectData->_numSamples,
			interpolateWeights = _projectData->CanInterpolateWeights()]() mutable
		{
			_isComputingDeformationData.store(true, std::memory_order_seq_cst);

			auto deformedMeshResult = ComputeDeformedMesh(std::move(mesh),
				std::move(cage),
				std::move(deformedMesh),
				deformationType,
				weightingScheme,
				somiglianaDeformer,
				modelVerticesOffset,
				numSamples,
				interpolateWeights);

			_deformationData.Update(std::move(deformedMeshResult.GetValue()._vertexData));

			_isComputingDeformationData.store(false, std::memory_order_seq_cst);

			// Copy the matrix to transpose in-place, so we can iterate over it in the correct memory data layout.
			_mainThreadQueue->Push([this]() mutable
			{
				// Update the positions of the mesh.
				UpdateDeformedMeshPositionsFromDeformationData();

				// We only recompute the vertex colors if they were previously on.
				if (_projectModel->_renderInfluenceMap)
				{
					UpdateMeshVertexColors(_projectModel->_renderInfluenceMap);
				}

				// Rebuild the entire BVH.
				{
					auto bvhBuilder = _scene->BeginGeometryBVH();
					bvhBuilder.AddGeometry(_deformedMeshHandle);
					bvhBuilder.AddGeometry(_deformedCageHandle);
				}
			});
		});
	}

	OnClickedSelection(actionParams);

	_isDragging = false;
	_hasDragged = false;
	_isGizmoTransformed = false;
}

void Editor::OnClickedSelection(const InputActionParams& actionParams)
{
	if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
	{
		return;
	}

	const auto& viewInfo = _cameraSubsystem->GetCamera().GetViewInfo();
	const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
	const auto selectionType = _statusBar->GetActiveSelectionType();

	if (!_isDragging && !_hasDragged)
	{
		// If we are doing selection first cache the projected vertices into screen space.
		deformedCageMesh->CacheProjectedPointsWorldToScreen(viewInfo);

		const auto mousePosition = _inputSubsystem->GetMousePosition();

		if (selectionType == SelectionType::Vertex)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Vertex>();
			auto hit = deformedCageMesh->QueryClosestPointScreenSpace(viewInfo,
				mousePosition,
				VertexSelectionMinDistanceThresholdSq);

			if (hit.has_value())
			{
				const auto handles = std::span(&hit->_vertexHandle, 1);

				if (IsSet(actionParams._modifierKeys, SDL_KMOD_LALT))
				{
					selection.Deselect(handles);
				}
				else if (IsSet(actionParams._modifierKeys, SDL_KMOD_LSHIFT))
				{
					selection.Select(handles);
				}
				else
				{
					selection.DeselectAll();
					selection.Select(handles);
				}
			}
		}
		else if (selectionType == SelectionType::Edge)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Edge>();
			auto hit = deformedCageMesh->QueryClosestEdgeScreenSpace(viewInfo, mousePosition, EdgeSelectionMinDistanceThreshold);

			if (hit.has_value())
			{
				const auto handles = std::span(&hit->_edgeHandle, 1);

				if (IsSet(actionParams._modifierKeys, SDL_KMOD_LALT))
				{
					selection.Deselect(handles);
				}
				else if (IsSet(actionParams._modifierKeys, SDL_KMOD_LSHIFT))
				{
					selection.Select(handles);
				}
				else
				{
					selection.DeselectAll();
					selection.Select(handles);
				}
			}
		}
		else if (selectionType == SelectionType::Polygon)
		{
			auto selection = deformedCageMesh->GetSelection<SelectionType::Polygon>();
			const auto worldRay = viewInfo.DeprojectScreenToWorldRay(mousePosition);
			auto hit = deformedCageMesh->QueryRayHit(worldRay);

			if (hit._polyHandle.is_valid())
			{
				const auto handles = std::span(&hit._polyHandle, 1);

				if (IsSet(actionParams._modifierKeys, SDL_KMOD_LALT))
				{
					selection.Deselect(handles);
				}
				else if (IsSet(actionParams._modifierKeys, SDL_KMOD_LSHIFT))
				{
					selection.Select(handles);
				}
				else
				{
					selection.DeselectAll();
					selection.Select(handles);
				}
			}
		}

		ResetGizmoPositionFromSelection(viewInfo);
	}
}

void Editor::OnToolSelectionChanged(const ToolType toolType)
{
	const auto activeGizmoType = _toolBar->GetActiveGizmoType();

	// Set the visibility of the gizmos based on the selected tool from the tool bar.
	if (_activeGizmoType != activeGizmoType)
	{
		_activeGizmoType = activeGizmoType;

		for (std::size_t i = 0; i < static_cast<std::size_t>(GizmoType::MaxNum); ++i)
		{
			const auto gizmoType = static_cast<GizmoType>(i);
			const auto isVisible = (gizmoType == activeGizmoType);

			_gizmo->SetVisible(gizmoType, isVisible);
		}
	}

	if (toolType == Tools::Transform::Translate)
	{
		const auto& viewInfo = _cameraSubsystem->GetCamera().GetViewInfo();

		ResetGizmoPositionFromSelection(viewInfo);
	}
}

void Editor::OnSelectionTypeChanged(const SelectionType selectionType)
{
	const auto& viewInfo = _cameraSubsystem->GetCamera().GetViewInfo();
	const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
	deformedCageMesh->CacheProjectedPointsWorldToScreen(viewInfo);

	// If we are doing vertex selection first cache the projected vertices into screen space.
	if (selectionType == SelectionType::Vertex)
	{
		deformedCageMesh->SetWireframeRenderMode(WireframeRenderMode::Points | WireframeRenderMode::Edges);
	}
	else if (selectionType == SelectionType::Edge)
	{
		deformedCageMesh->SetWireframeRenderMode(WireframeRenderMode::Edges);
	}
	else if (selectionType == SelectionType::Polygon)
	{
		deformedCageMesh->SetWireframeRenderMode(WireframeRenderMode::Edges | WireframeRenderMode::Polygons);
	}
	else
	{
		deformedCageMesh->SetWireframeRenderMode(WireframeRenderMode::None);
	}

	ResetGizmoPositionFromSelection(viewInfo);
}

void Editor::OnSequencerFrameIndexChanged(const uint32_t newFrameIndex)
{
	if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
	{
		return;
	}

	// Update deformed mesh vertices with the new sample index data.
	UpdateDeformedMeshPositionsFromDeformationData(newFrameIndex);

	// We only recompute the vertex colors if they were previously on.
	if (_projectModel->_renderInfluenceMap)
	{
		UpdateMeshVertexColors(_projectModel->_renderInfluenceMap);
	}
}

void Editor::OnSequencerNumFramesChanged(const uint32_t currentFrameIndex, const uint32_t numFrames)
{
	if (_deformedMeshHandle == InvalidHandle || _deformedCageHandle == InvalidHandle)
	{
		return;
	}

	// Re-compute the deformed mesh and update the render proxy.
	_threadPool->Submit([this,
		mesh = _scene->GetMesh(_deformedMeshHandle)->CopyAsEigen(),
		cage = _projectData->_cage,
		deformedMesh = _scene->GetMesh(_deformedCageHandle)->CopyAsEigen(),
		somiglianaDeformer = _projectData->_somiglianaDeformer,
		deformationType = _projectData->_deformationType,
		weightingScheme = _projectData->_LBCWeightingScheme,
		modelVerticesOffset = _projectData->_modelVerticesOffset,
		numSamples = _projectData->_numSamples,
		interpolateWeights = _projectData->CanInterpolateWeights(),
		currentFrameIndex]() mutable
	{
		_isComputingDeformationData.store(true, std::memory_order_seq_cst);

		auto deformedMeshResult = ComputeDeformedMesh(std::move(mesh),
			std::move(cage),
			std::move(deformedMesh),
			deformationType,
			weightingScheme,
			somiglianaDeformer,
			modelVerticesOffset,
			numSamples,
			interpolateWeights);

		_deformationData.Update(std::move(deformedMeshResult.GetValue()._vertexData));

		_isComputingDeformationData.store(false, std::memory_order_seq_cst);

		// Copy the matrix to transpose in-place, so we can iterate over it in the correct memory data layout.
		_mainThreadQueue->Push([this, currentFrameIndex]() mutable
		{
			// Update the positions of the mesh.
			UpdateDeformedMeshPositionsFromDeformationData(currentFrameIndex);

			// We only recompute the vertex colors if they were previously on.
			if (_projectModel->_renderInfluenceMap)
			{
				UpdateMeshVertexColors(_projectModel->_renderInfluenceMap);
			}
		});
	});
}

void Editor::OnSequencerStartedDragging()
{
	const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
	deformedCageMesh->SetVisible(false);
}

void Editor::OnSequencerEndedDragging()
{
	const auto deformedCageMesh = _scene->GetMesh(_deformedCageHandle);
	deformedCageMesh->SetVisible(true);
}

void Editor::UpdateDeformedMeshPositionsFromDeformationData(const std::optional<uint32_t> frameIndex)
{
	// Copy the matrix to transpose in-place, so we can iterate over it in the correct memory data layout.
	const auto deformationData = _deformationData.LockRead();
	const auto unpackedFrameIndex = frameIndex.value_or(deformationData->_vertexData.size() - 1);
	const auto meshPositions = deformationData->_vertexData[unpackedFrameIndex]._vertices.transpose();
	std::vector<glm::vec3> positions = GeometryUtils::EigenVerticesToGLM(meshPositions);

	const auto deformedMesh = _scene->GetMesh(_deformedMeshHandle);
	deformedMesh->SetPositions(positions);

	_statusBar->SetCurrentFrame(unpackedFrameIndex);
}

void Editor::ExportCurrentDeformedMesh(std::filesystem::path filepath) const
{
	CheckFormat(!_isComputingDeformationData.load(std::memory_order_relaxed), "The weights and the deformation mesh haven't been computed yet to export.");

	const auto deformationData = _deformationData.LockRead();

	_meshOperationSystem->ExecuteOperation<DeformedMeshExportOperation>(
		*deformationData,
		_projectData->_deformationType,
		_projectData->_LBCWeightingScheme,
		_statusBar->GetCurrentFrameIndex(),
		_projectData->_somiglianaDeformer,
		_projectData->_mesh._faces,
		std::move(filepath),
		1.0f / _projectData->_scalingFactor);
}

void Editor::ExportDeformedMeshes(std::filesystem::path filepath) const
{
	CheckFormat(!_isComputingDeformationData.load(std::memory_order_relaxed), "The weights and the deformation mesh haven't been computed yet to export.");

	const auto deformationData = _deformationData.LockRead();

	_meshOperationSystem->ExecuteOperation<DeformedMeshExportOperation>(
		*deformationData,
		_projectData->_deformationType,
		_projectData->_LBCWeightingScheme,
		std::optional<std::size_t>(),
		_projectData->_somiglianaDeformer,
		_projectData->_mesh._faces,
		std::move(filepath),
		1.0f / _projectData->_scalingFactor);
}

void Editor::ExportDeformedCage(std::filesystem::path filepath) const
{
	_meshOperationSystem->ExecuteOperation<MeshExportOperation>(
		_projectData->_deformationType,
		_projectData->_LBCWeightingScheme,
		_projectData->_deformedCage._faces,
		_projectData->_deformedCage._vertices,
		std::move(filepath));
}

void Editor::ExportInfluenceColorMap(std::filesystem::path filepath) const
{
	CheckFormat(!_isComputingWeightsData.load(std::memory_order_relaxed), "The weights and the deformation mesh haven't been computed yet to export.");

	auto weights = *_weightsData.LockRead();

	_meshOperationSystem->ExecuteOperation<MeshExportInfluenceMapOperation>(
		_projectData->_deformationType,
		_projectData->_LBCWeightingScheme,
		_projectData->_somiglianaDeformer,
		_projectData->_mesh,
		_projectData->_cage,
		_projectData->_parametrization.value(),
		std::move(filepath),
		std::move(weights),
		_projectData->_modelVerticesOffset,
		_projectData->CanInterpolateWeights());
}

void Editor::OnComputeInfluenceColorMap(const bool shouldRenderInfluenceMap) const
{
	UpdateMeshVertexColors(shouldRenderInfluenceMap && _projectModel->CanRenderInfluenceMap());
}

void Editor::ExportWeights(std::filesystem::path filepath) const
{
	CheckFormat(!_isComputingWeightsData.load(std::memory_order_relaxed), "The weights and the deformation mesh haven't been computed yet to export.");

	const auto weightsData = _weightsData.LockRead();

	_meshOperationSystem->ExecuteOperation<MeshExportWeightsOperation>(
		_projectData->_deformationType,
		_projectData->_LBCWeightingScheme,
		std::move(filepath),
		weightsData->_weights,
		_projectData->_b,
		_projectData->_bc,
		*_projectData->_embedding,
		_projectData->_numBBWSteps);
}

MeshOperationResult<std::shared_ptr<ProjectData>> Editor::CreateProject() const
{
	return _meshOperationSystem->ExecuteOperation<MeshLoadOperation>(
		_projectModel->_deformationType,
		_projectModel->_LBCWeightingScheme,
		_projectModel->_meshFilepath.value(),
		_projectModel->_cageFilepath.value_or(std::filesystem::path()),
		_projectModel->_deformedCageFilepath,
		_projectModel->_weightsFilepath,
		_projectModel->_embeddingFilepath,
		_projectModel->_parametersFilepath,
		_projectModel->_numBBWSteps,
		_projectModel->_numSamples,
		_projectModel->_scalingFactor,
		_projectModel->_interpolateWeights,
		_projectModel->_findOffset,
		_projectModel->_noOffset,
		_projectModel->_somigNu,
		_projectModel->_somiglianaDeformer
		);
}

MeshOperationResult<MeshComputeWeightsOperationResult> Editor::ComputeCageWeights(const ProjectData& projectData) const
{
	return _meshOperationSystem->ExecuteOperation<MeshComputeWeightsOperation>(
		projectData._deformationType,
		projectData._LBCWeightingScheme,
		projectData._mesh,
		projectData._cage,
		projectData._embedding,
		projectData._weights,
		projectData._somiglianaDeformer,
		projectData._cagePoints,
		projectData._normals,
		projectData._b,
		projectData._bc,
		projectData.CanInterpolateWeights(),
		projectData._numBBWSteps,
		projectData._numSamples);
}

MeshOperationResult<MeshCageGenerationResult> Editor::GenerateCage(const CageGenerationMethod cageGenerationMethod,
	EigenMesh mesh,
	EigenMesh cage,
	NewCageSetting setting) const
{
	return _meshOperationSystem->ExecuteOperation<MeshCageGeneration>(
		cageGenerationMethod,
		std::move(mesh),
		std::move(cage),
		setting
	);
}

MeshOperationResult<MeshComputeDeformationOperationResult> Editor::ComputeDeformedMesh(EigenMesh mesh,
	EigenMesh cage,
	EigenMesh deformedCage,
	const DeformationType deformationType,
	const LBC::DataSetup::WeightingScheme weightingScheme,
	const std::shared_ptr<somig_deformer_3>& somiglianaDeformer,
	const int32_t modelVerticesOffset,
	const int32_t numSamples,
	const bool interpolateWeights) const
{
	auto weights = *_weightsData.LockRead();

	return _meshOperationSystem->ExecuteOperation<MeshComputeDeformationOperation>(
		deformationType,
		weightingScheme,
		somiglianaDeformer,
		std::move(mesh),
		std::move(cage),
		std::move(deformedCage),
		std::move(weights),
		modelVerticesOffset,
		numSamples,
		interpolateWeights);
}

void Editor::ResetGizmoPositionFromSelection(const ViewInfo& viewInfo) const
{
	const auto activeSelectionType = _statusBar->GetActiveSelectionType();
	const auto cageMesh = _scene->GetMesh(_deformedCageHandle);

	const auto updateGizmoVisibility = [this](const auto& selection)
	{
		if (selection.HasSelection())
		{
			const auto activeGizmoType = _toolBar->GetActiveGizmoType();

			for (std::size_t i = 0; i < static_cast<std::size_t>(GizmoType::MaxNum); ++i)
			{
				const auto gizmoType = static_cast<GizmoType>(i);
				const auto isVisible = (gizmoType == activeGizmoType);

				_gizmo->SetVisible(gizmoType, isVisible);
			}
		}
		else
		{
			// Hide the gizmo if we have no selection.
			_gizmo->SetVisible(false);
		}
	};

	glm::vec3 averageSelPosition(0.0f);

	if (activeSelectionType == SelectionType::Vertex)
	{
		// Get the current vertex selection and apply the average position to the gizmo.
		const auto selection = cageMesh->GetSelection<SelectionType::Vertex>();
		const auto vertexSelection = selection.GetSelection();

		updateGizmoVisibility(selection);

		for (const auto vertexHandle : vertexSelection)
		{
			averageSelPosition += cageMesh->GetAverageVertexPosition(vertexHandle);
		}

		averageSelPosition /= std::max(vertexSelection.size(), 1_sz);
	}
	else if (activeSelectionType == SelectionType::Edge)
	{
		// Get the current vertex selection and apply the average position to the gizmo.
		const auto selection = cageMesh->GetSelection<SelectionType::Edge>();
		const auto uniqueVertices = selection.GetVertexSelection();

		updateGizmoVisibility(selection);

		for (const auto vertexHandle : uniqueVertices)
		{
			averageSelPosition += cageMesh->GetAverageVertexPosition(vertexHandle);
		}

		averageSelPosition /= std::max(uniqueVertices.size(), 1_sz);
	}
	else if (activeSelectionType == SelectionType::Polygon)
	{
		// Get the current vertex selection and apply the average position to the gizmo.
		const auto selection = cageMesh->GetSelection<SelectionType::Polygon>();
		const auto faceSelection = selection.GetSelection();

		updateGizmoVisibility(selection);

		for (const auto faceHandle : faceSelection)
		{
			averageSelPosition += cageMesh->GetAverageVertexPosition(faceHandle);
		}

		averageSelPosition /= std::max(faceSelection.size(), 1_sz);
	}

	// Update the gizmo position.
	_gizmo->SetPosition(viewInfo, averageSelPosition);
}

void Editor::UpdateMeshVertexColors(const bool shouldRenderInfluenceMap) const
{
	const auto deformedMesh = _scene->GetMesh(_deformedMeshHandle);

	if (shouldRenderInfluenceMap)
	{
		CheckFormat(!_isComputingWeightsData.load(std::memory_order_relaxed), "The weights and the deformation mesh haven't been computed yet to export.");

		auto weights = *_weightsData.LockRead();

		const auto vertexColorsResult = _meshOperationSystem->ExecuteOperation<MeshComputeInfluenceMapOperation>(
			_projectData->_deformationType,
			_projectData->_LBCWeightingScheme,
			_projectData->_somiglianaDeformer,
			_projectData->_mesh._vertices,
			_projectData->_parametrization.value(),
			std::move(weights),
			_projectData->_modelVerticesOffset,
			_projectData->CanInterpolateWeights());

		CheckFormat(!vertexColorsResult.HasError(), "The vertex colors should not fail.");

		const auto colors = vertexColorsResult.GetValue()._vertexColors.transpose();

		std::vector<glm::vec3> vertexColors(colors.cols());

		for (auto i = 0; i < colors.cols(); ++i)
		{
			vertexColors[i] = glm::vec3(colors(0, i), colors(1, i), colors(2, i));
		}

		deformedMesh->SetColors(vertexColors, true);
	}
	else
	{
		std::vector<glm::vec3> vertexColors(deformedMesh->GetNumVertices());

		for (auto& vertexColor : vertexColors)
		{
			vertexColor = glm::vec3(0.0f);
		}

		deformedMesh->SetColors(vertexColors, false);
	}
}
