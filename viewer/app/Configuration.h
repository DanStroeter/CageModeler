#ifndef MESHCAGEDEFORMATIONS_CONFIGURATION_H_IN
#define MESHCAGEDEFORMATIONS_CONFIGURATION_H_IN

#include <string>

namespace MCD
{

static inline constexpr bool EnableVulkanValidation = false;
static inline constexpr std::string_view ProjectName = "CageModeler-App";
static inline constexpr std::string_view ProjectVersion = "1.0";
static inline constexpr int32_t ProjectVersionMajor { 1 };
static inline constexpr int32_t ProjectVersionMinor { 0 };
static inline constexpr int32_t ProjectVersionPatch {  };
static inline constexpr int32_t ProjectVersionTweak {  };
static inline constexpr std::string_view GitSHA = "";

} // namespace CageDeformations

#endif //MESHCAGEDEFORMATIONS_CONFIGURATION_H_IN
