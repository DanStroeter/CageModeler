#pragma once

#include <volk.h>

#include <Core/Subsystem.h>

struct SDL_Window;

class WindowSubsystem final : public Subsystem
{
	DECLARE_SUBSYSTEM(WindowSubsystem)

public:
	//~BEGIN Subsystem
	void Initialize(const SubsystemsCollection& collection) override;
	void Update(const double deltaTime) override { }
	void Deinitialize() override;
	//~END Subsystem

	/**
	 * Queries the required Vulkan instance extensions from the window and returns them.
	 *
	 * @return The required instance extensions.
	 */
	[[nodiscard]] static std::vector<const char*> GetRequiredInstanceExtensions();

	/**
	 * Creates a new Vulkan surface to render on.
	 *
	 * @param instance A Vulkan instance.
	 * @return A new Vulkan render surface.
	 */
	[[nodiscard]] VkSurfaceKHR CreateVulkanSurface(const VkInstance instance) const;

	/**
	 * Queries the current window drawable size in pixels.
	 *
	 * @return The drawable window size.
	 */
	[[nodiscard]] VkExtent2D GetDrawableSize() const;

	/**
	 * Queries the current window size.
	 *
	 * @return The drawable window size.
	 */
	[[nodiscard]] VkExtent2D GetWindowSize() const;

private:
	// Temporary friend class.
	friend class RenderSubsystem;

private:
	/// A pointer to the SDL window.
	SDL_Window* _window = nullptr;
};
