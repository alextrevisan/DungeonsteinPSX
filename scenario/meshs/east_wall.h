#ifndef __left_wall_h_
#define __left_wall_h_

#include <psxgte.h>

struct east_wall {

	const SVECTOR vertices[9] = {
		{ -300, -600, -300 },
		{ -300, 0, -300 },
		{ -300, -600, 300 },
		{ -300, 0, 300 },
		{ -300, -300, -300 },
		{ -300, -300, 300 },
		{ -300, -600, 0 },
		{ -300, 0, 0 },
		{ -300, -300, 0 },
	};

	const DVECTOR uvs[16] = {
		{ 0, 63 },
		{ 0, 0 },
		{ 63, 0 },
		{ 63, 63 },
		{ 0, 63 },
		{ 0, 0 },
		{ 63, 0 },
		{ 63, 63 },
		{ 0, 63 },
		{ 0, 0 },
		{ 63, 0 },
		{ 63, 63 },
		{ 0, 63 },
		{ 0, 0 },
		{ 63, 0 },
		{ 63, 63 },
	};

	const SVECTOR normals[1] = {
		{ -4095, 0, 0 },
	};

	struct face4
	{
		short vertice0,uv0,normal0;
		short vertice1,uv1,normal1;
		short vertice2,uv2,normal2;
		short vertice3,uv3,normal3;
	};

	const face4 quads[4] = {
		{ 8, 0, 0, 5, 1, 0, 3, 2, 0, 7, 3, 0 },
		{ 6, 4, 0, 2, 5, 0, 5, 6, 0, 8, 7, 0 },
		{ 0, 8, 0, 6, 9, 0, 8, 10, 0, 4, 11, 0 },
		{ 4, 12, 0, 8, 13, 0, 7, 14, 0, 1, 15, 0 },
	};

	struct face3
	{
		unsigned short vertice0,uv0,normal0;
		unsigned short vertice1,uv1,normal1;
		unsigned short vertice2,uv2,normal2;
	};

	const face3 tris[0] = {
	};

	int x = 0, y = 0, z = 0;
	TIM_IMAGE* texture = nullptr;
};

#endif //__left_wall_h_
