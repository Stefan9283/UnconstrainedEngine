#version 330 core

uniform vec3 color;
uniform vec3 cameraPos;
uniform float flatColorsON;
in vec3 FragPos;
in vec3 Norm;
out vec4 FragColor;
void main()
{
	if (flatColorsON == 0.0f) {
		vec3 lightDir = normalize(cameraPos - FragPos);
		vec3 norm = normalize(Norm);
		float diff = max(dot(norm, lightDir), 0.0);
		vec3 diffuse = color * diff;
		FragColor = vec4(diffuse, 1.0f);
	} else FragColor = vec4(color, 1.0f);
};
