struct MaterialInfo
{
	vec3 AmbientReflectivity;
	vec3 DiffuseReflectivity;
	vec3 SpecularReflectivity;
	float Shininess;
};

struct LightInfo
{
	vec4 PositionAndIntensity;
};

struct PerFrameData
{
	mat4 View;
	mat4 Projection;
	vec2 ViewportSize;
};

struct PerObjectData
{
	mat4 Model;
	mat3 NormalMatrix;
};

vec3 sRGBToLinear(vec3 rgb)
{
	// See https://gamedev.stackexchange.com/questions/92015/optimized-linear-to-srgb-glsl
	return mix(pow((rgb + 0.055) * (1.0 / 1.055), vec3(2.4)),
		rgb * (1.0/12.92),
		lessThanEqual(rgb, vec3(0.04045)));
}

vec3 LinearToSRGB(vec3 rgb)
{
	// See https://gamedev.stackexchange.com/questions/92015/optimized-linear-to-srgb-glsl
	return mix(1.055 * pow(rgb, vec3(1.0 / 2.4)) - 0.055,
		rgb * 12.92,
		lessThanEqual(rgb, vec3(0.0031308)));
}
