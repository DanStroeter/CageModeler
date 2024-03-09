#pragma once

#include <Core/Subsystem.h>
#include <Rendering/Core/Instance.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/Synchronization.h>
#include <Rendering/Core/RenderSurface.h>
#include <Rendering/Core/Swapchain.h>
#include <Editor/Editor.h>
#include <UI/UserInterfaceBackend.h>

class CameraSubsystem;
class RenderCommandScheduler;
class WindowSubsystem;
class ResourceManager;
class RenderPipelineManager;

class RenderSubsystem final : public Subsystem
{
	DECLARE_SUBSYSTEM(RenderingSubsystem)

public:
	//~BEGIN Subsystem
	void Initialize(const SubsystemsCollection& collection) override;
	void Update(const double deltaTime) override { }
	void Deinitialize() override;
	//~END Subsystem

	void InitializeEditor(const std::shared_ptr<Editor>& editor);

	void OnWindowResized();

	void Render(const double deltaTime);

private:
	void ReleaseResource();
	void CreateFrameSynchronization();

	void UpdateUniforms();

	[[nodiscard]] VkRenderPass CreateRenderPass() const;

private:
	// Temporary friend class.
	friend class UserInterfaceBackend;

private:
	RenderResourceRef<Instance> _instance;
	RenderResourceRef<Device> _device;
	RenderResourceRef<RenderSurface> _surface;
	RenderResourceRef<Swapchain> _swapchain;
	RenderResourceRef<DescriptorPool> _descriptorPool;

	std::shared_ptr<RenderCommandScheduler> _renderCommandScheduler = nullptr;
	std::shared_ptr<RenderResourceManager> _renderResourceManager = nullptr;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;

	/// The proxy collector where we register render proxies.
	std::shared_ptr<RenderProxyCollector> _renderProxyCollector = nullptr;

	std::unique_ptr<UserInterfaceBackend> _uiBackend = nullptr;
	std::shared_ptr<Editor> _editor = nullptr;
	std::shared_ptr<SceneRenderer> _sceneRenderer = nullptr;

	VkRenderPass _renderPass = VK_NULL_HANDLE;
	std::vector<VkFramebuffer> _framebuffers;

	std::shared_ptr<PolygonMesh> _deformableMesh = nullptr;
	std::shared_ptr<PolygonMesh> _cage = nullptr;

	std::vector<RenderResourceRef<Semaphore>> _imageAvailableSemaphores;
	std::vector<RenderResourceRef<Semaphore>> _renderFinishedSemaphores;
	std::vector<RenderResourceRef<Fence>> _imageInFlightFences;

	uint32_t _currentFrameIndex = 0;

#if BUILD_DEVELOPMENT
	VkDebugUtilsMessengerEXT _debugMessenger = VK_NULL_HANDLE;
#endif

	SubsystemPtr<WindowSubsystem> _windowSubsystem = nullptr;
	SubsystemPtr<CameraSubsystem> _cameraSubsystem = nullptr;
};
