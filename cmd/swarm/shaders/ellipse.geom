#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices=21) out;

in struct {
	float dir;
	vec2 size;
	float offset;
	vec4 color;
} Attr[1];

out vec4 vertexColor;

const int N = 13; // number of points around ellipse

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void main()
{
	mat4 rot = mat4( cos(Attr[0].dir), sin(Attr[0].dir), 0.0, 0.0,
	                -sin(Attr[0].dir), cos(Attr[0].dir), 0.0, 0.0,
	                0.0, 0.0, 1.0, 0.0,
	                0.0, 0.0, 0.0, 1.0);

	// ellipse body
	for (int i = 0; i < N; ++i)
	{
		float t = 2.0 * M_PI * float(i) / float(N - 1);
		vec4 point = vec4(Attr[0].size.x * (cos(t) - Attr[0].offset), Attr[0].size.y * sin(t), 0.0, 0.0);
		gl_Position = gl_in[0].gl_Position + rot * point;
		vertexColor = Attr[0].color;
		EmitVertex();
		if (i % 3 == 0)
		{
			gl_Position = gl_in[0].gl_Position + rot * vec4(-Attr[0].offset * Attr[0].size.x, 0.0, 0.0, 0.0);
			vertexColor = Attr[0].color;
			EmitVertex();
		}
	}
	EndPrimitive();

	// show position of the eye
	gl_Position = gl_in[0].gl_Position + rot * vec4(Attr[0].size.y, 0.0, 0.0, 0.0);
	vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
	EmitVertex();
	gl_Position = gl_in[0].gl_Position + rot * vec4(-Attr[0].size.y, 0.5 * Attr[0].size.y, 0.0, 0.0);
	vertexColor = vec4(0.0, 0.0, 0.0, 1.0);
	EmitVertex();
	gl_Position = gl_in[0].gl_Position + rot * vec4(-Attr[0].size.y,-0.5 * Attr[0].size.y, 0.0, 0.0);
	vertexColor = vec4(0.0, 0.0, 0.0, 1.0);
	EmitVertex();
	EndPrimitive();
}
