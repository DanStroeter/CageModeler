#version 450

vec3 GridPlanePositions[6] = vec3[](
	vec3(1.0, 1.0, 0.0), vec3(-1.0, -1.0, 0.0), vec3(-1.0, 1.0, 0.0),
	vec3(-1.0, -1.0, 0.0), vec3(1.0, 1.0, 0.0), vec3(1.0, -1.0, 0.0)
);

void main()
{
	vec3 GridPosition = GridPlanePositions[gl_VertexIndex].xyz;

	gl_Position = vec4(GridPosition, 1.0);
}
