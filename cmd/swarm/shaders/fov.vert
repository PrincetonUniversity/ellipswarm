#version 330 core

uniform vec2 vp[2]; // viewport

layout(location = 5) in vec2 fpos;
layout(location = 6) in float attractivity;

out struct {
	float attractivity;
} Attr;

void main()
{
	vec2 size = vp[1] - vp[0];
	gl_Position = vec4(
		2.0 * (fpos.x - vp[0].x) / size.x - 1.0,
		2.0 * (fpos.y - vp[0].y) / size.y - 1.0,
		0.0, 1.0);
	Attr.attractivity = attractivity;
}
