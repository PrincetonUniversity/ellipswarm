#version 330 core

uniform vec2 vp[2]; // viewport

layout(location = 0) in vec2 pos;
layout(location = 1) in vec2 vel;
layout(location = 2) in float width;
layout(location = 3) in float offset;
layout(location = 4) in vec4 color;

out Data {
	vec2 vel;
	vec2 size;
	float offset;
	vec4 color;
} Attr;

void main()
{
	vec2 size = vp[1] - vp[0];
	gl_Position = vec4(
		2.0 * (pos.x - vp[0].x) / size.x - 1.0,
		2.0 * (pos.y - vp[0].y) / size.y - 1.0,
		0.0, 1.0);
	Attr.vel = vel;
	Attr.size = vec2(1.0, width) / size.x;
	Attr.offset = offset;
	Attr.color = color;
}
