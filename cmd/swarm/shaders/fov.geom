#version 330 core

layout(lines) in;
layout(triangle_strip, max_vertices=3) out;

uniform vec2 vp[2]; // viewport
uniform vec2 pos;   // position of focal individual

in Data {
	float attractivity;
} Attr[2];

out vec4 vertexColor;

const float alpha = 0.3;

void main()
{
	vec2 size = vp[1] - vp[0];
	gl_Position = vec4(
		2.0 * (pos.x - vp[0].x) / size.x - 1.0,
		2.0 * (pos.y - vp[0].y) / size.y - 1.0,
		0.0, 1.0);

	// color based on attractivity
	float x =  4.0 * Attr[0].attractivity;
	vec4 col = vec4(
		1.0 - smoothstep(0.0, 1.0, x),
		smoothstep(-1.0, 0.0, x),
		smoothstep(-1.0, 0.0, x) - smoothstep(0.0, 1.0, x),
		alpha);
	vertexColor = col;
	EmitVertex();
	gl_Position = gl_in[0].gl_Position;
	vertexColor = col;
	EmitVertex();
	gl_Position = gl_in[1].gl_Position;
	vertexColor = col;
	EmitVertex();
	EndPrimitive();
}
