    #ifndef _CLIP_H
#define _CLIP_H

#include <psxgte.h>
#include <psxgpu.h>

/* tri_clip
 *
 * Returns non-zero if a triangle (v0, v1, v2) is outside 'clip'.
 *
 * clip			- Clipping area
 * v0,v1,v2		- Triangle coordinates
 *
 */
constexpr char CLIP_LEFT = 1;
constexpr char CLIP_RIGHT = 2;
constexpr char CLIP_TOP	= 3;
constexpr char CLIP_BOTTOM	= 4;
//1858
namespace
{
template<RECT clip>
const unsigned char test_clip(const DVECTOR* rect)
{
	unsigned char ret = 0;
	if (rect->vx < clip.x)
		ret |= (1 << (CLIP_LEFT - 1));
	if (rect->vx > (clip.x + clip.w))
		ret |= (1 << (CLIP_RIGHT - 1));
	if (rect->vy < clip.y)
		ret |= (1 << (CLIP_TOP - 1));
	if (rect->vy > (clip.y + clip.h))
		ret |= (1 << (CLIP_BOTTOM - 1));
	return ret;
}
} // namespace
//1858
template<RECT clip>
static constexpr int tri_clip(const DVECTOR* v0, const DVECTOR* v1, const DVECTOR* v2) {
	
	// Returns non-zero if a triangle is outside the screen boundaries
	
	const auto c0 = test_clip<clip>(v0);
	const auto c1 = test_clip<clip>(v1);
	if ( ( c0 & c1 ) == 0 )
		return 0;

	const auto c2 = test_clip<clip>(v2);
	if ( ( c0 & c2 ) == 0 )
		return 0;
	
	if ( ( c1 & c2 ) == 0 )
		return 0;

	return 1;
}

/* quad_clip
 *
 * Returns non-zero if a quad (v0, v1, v2, v3) is outside 'clip'.
 *
 * clip			- Clipping area
 * v0,v1,v2,v3	- Quad coordinates
 *
 */
template<RECT clip>
int quad_clip(DVECTOR *v0, DVECTOR *v1, DVECTOR *v2, DVECTOR *v3) {
	
	// Returns non-zero if a quad is outside the screen boundaries
	
	short c[4];

	c[0] = test_clip<clip>(v0);
	c[1] = test_clip<clip>(v1);
	c[2] = test_clip<clip>(v2);
	c[3] = test_clip<clip>(v3);

	if ( ( c[0] & c[1] ) == 0 )
		return 0;
	if ( ( c[1] & c[2] ) == 0 )
		return 0;
	if ( ( c[2] & c[3] ) == 0 )
		return 0;
	if ( ( c[3] & c[0] ) == 0 )
		return 0;
	if ( ( c[0] & c[2] ) == 0 )
		return 0;
	if ( ( c[1] & c[3] ) == 0 )
		return 0;

	return 1;
}
#endif // _CLIP_H