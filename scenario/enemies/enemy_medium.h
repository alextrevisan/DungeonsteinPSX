#ifndef __enemy_medium_h_
#define __enemy_medium_h_

#include <psxgte.h>

struct enemy_medium {

	const SVECTOR vertices[9] = {
		{ 90, -300, 0 },
		{ 90, 0, 0 },
		{ -90, -300, 0 },
		{ -90, 0, 0 },
		{ 90, -150, 0 },
		{ -90, -150, 0 },
		{ 0, -300, 0 },
		{ 0, 0, 0 },
		{ 0, -150, 0 },
	};

	const DVECTOR uvs[9] = {
		{ 16, 29 },
		{ 33, 29 },
		{ 33, 57 },
		{ 16, 57 },
		{ 16, 0 },
		{ 33, 0 },
		{ 0, 0 },
		{ 0, 29 },
		{ 0, 57 },
	};

	const SVECTOR normals[1] = {
		{ 0, 0, -4095 },
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
		{ 6, 4, 0, 2, 5, 0, 5, 1, 0, 8, 0, 0 },
		{ 0, 6, 0, 6, 4, 0, 8, 0, 0, 4, 7, 0 },
		{ 4, 7, 0, 8, 0, 0, 7, 3, 0, 1, 8, 0 },
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

#endif //__enemy_medium_h_
