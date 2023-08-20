#include <sys/types.h>
#include <stdio.h>
#include <psxgpu.h>
#include <psxgte.h>
#include <psxpad.h>
#include <psxapi.h>
#include <psxetc.h>
#include <inline_c.h>
#include <gtemac.h>

#include <scenario/meshs/level01.h>

#include <scenario/meshs/east_wall.h>
#include <scenario/meshs/west_wall.h>
#include<scenario/meshs/north_wall.h>
#include<scenario/meshs/south_wall.h>

#include <scenario/meshs/ceilling.h>
#include <scenario/meshs/ground.h>

#include <scenario/enemies/enemy_medium.h>

#include "clip.h"
#include "engine/fast_draw_functions.h"

#include "engine/TextureManager.h"

// OT and Packet Buffer sizes
#define OT_LEN 4096
#define PACKET_LEN 32768

// Screen resolution
#define SCREEN_XRES 320
#define SCREEN_YRES 240

// Screen center position
#define CENTERX SCREEN_XRES >> 1
#define CENTERY SCREEN_YRES >> 1
#define gte_getir1( )			\
	({ long r0;					\
	__asm__ volatile (			\
	"mfc2	%0, $9;"			\
	: "=r"( r0 )				\
	:							\
	);							\
	r0; })

/* Get the IR3 register from the GTE */
#define gte_getir3( )			\
	({ long r0;					\
	__asm__ volatile (			\
	"mfc2	%0, $11;"			\
	: "=r"( r0 )				\
	:							\
	);							\
	r0; })
namespace
{
    int fps;
    int fps_counter;
    int fps_measure;

    void vsync_cb(void)
    {
        fps_counter++;
        if (fps_counter >= 60)
        {
            fps = fps_measure;
            fps_measure = 0;
            fps_counter = 0;
        }
    }
} // namespace

template <typename T, typename U>
struct is_same
{
    static constexpr bool value = false;
};

template <typename T>
struct is_same<T, T> //specialization
{
    static constexpr bool value = true;
};

// Double buffer structure
typedef struct
{
    DISPENV disp;        // Display environment
    DRAWENV draw;        // Drawing environment
    u_long _orderingTable[OT_LEN];    // Ordering table
    uint8_t _packetBuffer[PACKET_LEN]; // Packet buffer
} DB;


extern u_long ground_tim[];
TIM_IMAGE ground_texture;

extern u_long goblin_tim[];
TIM_IMAGE goblin_texture;

constexpr RECT screen_clip{0, 0, SCREEN_XRES, SCREEN_YRES};

// Pad data buffer
uint8_t pad_buff[2][34];

// For easier handling of vertex indexes
typedef struct
{
    short v0, v1, v2, v3;
} INDEX;

// Cube vertices
SVECTOR cube_verts[] = {
    {-1, -1, -1, 0},
    {1, -1, -1, 0},
    {-1, 1, -1, 0},
    {1, 1, -1, 0},
    {1, -1, 1, 0},
    {-1, -1, 1, 0},
    {1, 1, 1, 0},
    {-1, 1, 1, 0}};

// Cube face normals
SVECTOR cube_norms[] = {
    {0, 0, -ONE, 0},
    {0, 0, ONE, 0},
    {0, -ONE, 0, 0},
    {0, ONE, 0, 0},
    {-ONE, 0, 0, 0},
    {ONE, 0, 0, 0}};

// Cube vertex indices
INDEX cube_indices[] = {
    {0, 1, 2, 3},
    {4, 5, 6, 7},
    {5, 4, 0, 1},
    {6, 7, 3, 2},
    {0, 2, 5, 7},
    {3, 1, 6, 4}};

// Number of faces of cube
#define CUBE_FACES 6

// Light color matrix
// Each column represents the color matrix of each light source and is
// used as material color when using gte_ncs() or multiplied by a
// source color when using gte_nccs(). 4096 is 1.0 in this matrix
// A column of zeroes effectively disables the light source.
MATRIX color_mtx = {
    ONE>>1, 0, 0, // Red
    ONE>>1, 0, 0,   // Green
    ONE>>1, 0, 0  // Blue
};

// Light matrix
// Each row represents a vector direction of each light source.
// An entire row of zeroes effectively disables the light source.
MATRIX light_mtx = {
    /* X,  Y,  Z */
    -4058, -2048, -2048,
    0, 0, 0,
    0, 0, 0};

#define gte_SetDQA( r0 ) __asm__ volatile (		\
	"addu	$12,$0,%0;"						\
	"ctc2	$12, $27"					\
	:							\
	: "r"( r0 ) 			\
	: "a0" )

//-----------------------------------------------------------------------------

#define gte_SetDQB( r0 ) __asm__ volatile (		\
	"addu	$12,$0,%0;"				\
	"ctc2	$12, $28"					\
	:							\
	: "r"( r0 )             \
	: "$12"  )

class Graphics
{
    // Double buffer variables
    DB db[2];
    int db_active = 0;
    uint8_t *db_nextpri;

    DISPENV* disp;
    DRAWENV* draw;
    u_long* _orderingTable;
    uint8_t* _packetBuffer;
	enum BlendMode {
		Blend = 0,
		Add = 1,
		Sub = 2,
		Mul =  3
	};
public:
    template <typename GeometryType, bool BackfaceCulling = true, bool TriangleClip = true, bool ComputeNormal = true>
    inline void Draw(const SVECTOR (&values)[], const SVECTOR &normal, TIM_IMAGE *texture = nullptr, const DVECTOR (&uvs)[] = {}, const CVECTOR (&color)[] = {})
    {
        GeometryType* pol4 = (GeometryType *)db_nextpri;
        // Load the first 3 vertices of a quad to the GTE
        gte_ldv3_f(
            values[0],
            values[1],
            values[2]);
        // Rotation, Translation and Perspective Triple
        gte_rtpt_b();
        int p;
        if constexpr (BackfaceCulling)
        {
            // Compute normal clip for backface culling
            gte_nclip_b();

            // Get result
            gte_stopz_m(p);

            // Skip this face if backfaced
            if (p < 0)
                return;
        }
        // Calculate average Z for depth sorting
        gte_avsz3_b();
        gte_stotz_m(p);

        // Skip if clipping off
        // (the shift right operator is to scale the depth precision)
        if (((p ) <= 0) || ((p ) >= OT_LEN))
            return;
        
        if constexpr(is_same<GeometryType, POLY_F4>::value)
        {
            // Initialize a quad primitive
            setPolyF4((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
            // Set the projected vertices to the primitive
            gte_stsxy3_f4(pol4);
        }

        if constexpr(is_same<GeometryType, POLY_GT3>::value)
        {
            // Initialize a quad primitive
            setPolyGT3((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
            // Set the projected vertices to the primitive
            gte_stsxy3_gt3(pol4);
        }

        if constexpr(is_same<GeometryType, POLY_GT4>::value)
        {
            // Initialize a quad primitive
            setPolyGT4((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
            // Set the projected vertices to the primitive
            gte_stsxy3_gt4(pol4);
        }

		if constexpr(is_same<GeometryType, POLY_G4>::value)
        {
            // Initialize a quad primitive
            setPolyG4((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
			
            // Set the projected vertices to the primitive
            gte_stsxy3_g4(pol4);
        }

        if constexpr(is_same<GeometryType, POLY_F3>::value)
        {
            // Initialize a quad primitive
            setPolyF3((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
            // Set the projected vertices to the primitive
            gte_stsxy3_f3(pol4);
        }
        if constexpr (is_same<GeometryType, POLY_FT4>::value)
        {
            // Initialize a quad primitive
            setPolyFT4((GeometryType *)pol4);
            //setSemiTrans(pol4, 1);
            
            // Set the projected vertices to the primitive
            gte_stsxy3_ft4(pol4);
        }

        
        if constexpr(is_same<GeometryType, POLY_F4>::value || 
					 is_same<GeometryType, POLY_FT4>::value || 
					 is_same<GeometryType, POLY_GT4>::value || 
					 is_same<GeometryType, POLY_G4>::value)
        {
            // Compute the last vertex and set the result
            gte_ldv0_f(values[3]);
            gte_rtps_b();
            gte_stsxy(&pol4->x3);
        }
        // Test if quad is off-screen, discard if so
        if constexpr (TriangleClip && (is_same<GeometryType, POLY_F4>::value || is_same<GeometryType, POLY_GT4>::value))
        {
            if (quad_clip<screen_clip>(
                    (DVECTOR *)&pol4->x0, (DVECTOR *)&pol4->x1,
                    (DVECTOR *)&pol4->x2, (DVECTOR *)&pol4->x3))
                return;
        }
		
/*

        const int fog_near = 0;
        const int fog_far = 1800;
        //246,215,176
        int fog_r = 246;
        int fog_g = 215;
        int fog_b = 176;
        const CVECTOR fogRGB{fog_r, fog_g, fog_b};

        int dist = gte_getir1();

        // Compute the fog factor
        int fog_factor = (dist - fog_near) * 4096 / (fog_far - fog_near);
        fog_factor = fog_factor > ONE ? ONE : fog_factor;//clamp(fog_factor, 0, 4096);
        fog_factor = fog_factor < 0 ? 0 : fog_factor;

        // Blend the color with the fog color
        int r = color.r;//(color.r >> 16) & 0xFF;
        int g = color.g;//(vertex_color >> 8) & 0xFF;
        int b = color.b;//vertex_color & 0xFF;
        r = (r * (4096 - fog_factor) + fog_r * fog_factor) >> 12;
        g = (g * (4096 - fog_factor) + fog_g * fog_factor) >> 12;
        b = (b * (4096 - fog_factor) + fog_b * fog_factor) >> 12;
        

        CVECTOR out;
        CVECTOR fogColor {r,g,b};
        //gte_DpqColor(&fogColor, p, &out);
        setRGB0(pol4, fogColor.r, fogColor.g, fogColor.b);
        */
       
       /*** estava funcionando 
        int dist = gte_getir3()<<1;
        CVECTOR out;
        
        gte_DpqColor(&color, dist, &out);
        setRGB0(pol4, out.r, out.g, out.b);
        if constexpr (ComputeNormal)
        {
            gte_ldrgb(&pol4->r0);

            // Load the face normal
            gte_ldv0_f(normal);

            // Normal Color Single
            gte_ncs_b();

            // Store result to the primitive
            gte_strgb(&pol4->r0);
        }
        ***/

        CVECTOR out_color;
        gte_DpqColor(&color[0], (p<<2)+p, &out_color);
        setRGB0(pol4, out_color.r, out_color.g, out_color.b);
        gte_ldrgb(&pol4->r0);

        if constexpr (is_same<GeometryType, POLY_FT3>::value || 
                      is_same<GeometryType, POLY_FT4>::value || 
                      is_same<GeometryType, POLY_GT4>::value || 
                      is_same<GeometryType, POLY_GT3>::value)
        {


            // Set tpage
            pol4->tpage = getTPage(texture->mode, 0, texture->prect->x, texture->prect->y);

            if(texture->mode&0x8)
            {
                // Set CLUT
                setClut(pol4, texture->crect->x, texture->crect->y);
            }
            //setUV3
            pol4->u0 = uvs[0].vx;
            pol4->v0 = uvs[0].vy;

            pol4->u1 = uvs[1].vx;
            pol4->v1 = uvs[1].vy;

            pol4->u2 = uvs[2].vx;
            pol4->v2 = uvs[2].vy;
            if constexpr (is_same<GeometryType, POLY_FT4>::value || is_same<GeometryType, POLY_GT4>::value)
            {
                pol4->u3 = uvs[3].vx;
                pol4->v3 = uvs[3].vy;
            }
        }

        // Sort primitive to the ordering table
        addPrim(_orderingTable + (p ), pol4);

        // Advance to make another primitive
        pol4++;
        db_nextpri = (uint8_t*) pol4;
    }
    void display()
    {

        fps_measure++;
        // Wait for GPU to finish drawing and vertical retrace
        DrawSync( 0 );
        VSync( 0 );

        // Swap buffers
        db_active ^= 1;
        db_nextpri = db[db_active]._packetBuffer;

        // Clear the OT of the next frame
        ClearOTagR(db[db_active]._orderingTable, OT_LEN);

        // Apply display/drawing environments
        PutDrawEnv(&db[db_active].draw);
        PutDispEnv(&db[db_active].disp);

        // Enable display
        SetDispMask(1);

        // Start drawing the OT of the last buffer
        DrawOTag(db[1 - db_active]._orderingTable + (OT_LEN - 1));
        _orderingTable = &db[db_active]._orderingTable[0];
    }
    void init()
    {

        // Reset the GPU, also installs a VSync event handler
        ResetGraph(0);

        // Set display and draw environment areas
        // (display and draw areas must be separate, otherwise hello flicker)
        SetDefDispEnv(&db[0].disp, 0, SCREEN_YRES, SCREEN_XRES, SCREEN_YRES);
        SetDefDrawEnv(&db[0].draw, 0, 0, SCREEN_XRES, SCREEN_YRES);
        

        // Enable draw area clear and dither processing
        setRGB0(&db[0].draw, 0,0,0);
        db[0].draw.isbg = 1;
        db[0].draw.dtd = 1;

        // Define the second set of display/draw environments
        SetDefDispEnv(&db[1].disp, 0, 0, SCREEN_XRES, SCREEN_YRES);
        SetDefDrawEnv(&db[1].draw, 0, SCREEN_YRES, SCREEN_XRES, SCREEN_YRES);


        setRGB0(&db[1].draw, 0,0,0);
        db[1].draw.isbg = 1;
        db[1].draw.dtd = 1;

        // Apply the drawing environment of the first double buffer
        PutDrawEnv(&db[0].draw);

        // Clear both ordering tables to make sure they are clean at the start
        ClearOTagR(db[0]._orderingTable, OT_LEN);
        ClearOTagR(db[1]._orderingTable, OT_LEN);

        // Set primitive pointer address
        db_nextpri = db[0]._packetBuffer;

        // Set clip region
        // setRECT( &screen_clip, 0, 0, SCREEN_XRES, SCREEN_YRES );

        // Initialize the GTE
        InitGeom();

        // Set GTE offset (recommended method  of centering)
        gte_SetGeomOffset(CENTERX, CENTERY);

        // Set screen depth (basically FOV control, W/2 works best)
        gte_SetGeomScreen(CENTERX);

        // Set light ambient color and light color matrix
        gte_SetBackColor(0,0,0);
		gte_SetFarColor(0,0,0);
        gte_SetColorMatrix(&color_mtx);

        VSyncCallback(vsync_cb);
        // Init BIOS pad driver and set pad buffers (buffers are updated
        // automatically on every V-Blank)
        InitPAD(&pad_buff[0][0], 34, &pad_buff[1][0], 34);

        // Start pad
        StartPAD();

        // Don't make pad driver acknowledge V-Blank IRQ (recommended)
        ChangeClearPAD(0);

        // Load font and open a text stream
        FntLoad(960, 0);
        FntOpen(0, 8, 320, 216, 0, 100);

        _orderingTable = &db[db_active]._orderingTable[0];

    }
};

Graphics *graphics;
SVECTOR verts[17][17]; // Vertex array for floor
int px, py;
void draw_map(MATRIX *mtx);
template<typename T>
T abs(T value)
{
    if(value < 0)
        return value * -1;
    return value;
}




level01 level_01;
const auto map = level_01.tiles;
auto mobs = level_01.mobs;

 struct
    {
        int x,y;
    }start;
    
int currentTileX;
int currentTileZ;

int main()
{
    start.x = 2;
    start.y = 2;
    currentTileX = start.x;
    currentTileZ = start.y;
    int lastTileX = currentTileX;
    int lastTileZ = currentTileZ;

    VECTOR cam_pos; // Camera position (in fixed point integers)
    SVECTOR cam_rot; // Camera view angle (in fixed point integers)

    VECTOR tpos;      // Translation value for matrix calculations
    MATRIX mtx, lmtx; // Rotation matrices for geometry and lighting

    PADTYPE *pad; // Pad structure pointer for parsing controller

    graphics = new Graphics();
    graphics->init();

    TextureManager::LoadTexture(ground_tim, ground_texture);
    TextureManager::LoadTexture(goblin_tim, goblin_texture);

    // Set coordinates to the vertex array for the floor
    for (py = 0; py < 17; py++)
    {
        for (px = 0; px < 17; px++)
        {

            setVector(&verts[py][px],
                      (100 * (px - 8)) - 50,
                      0,
                      (100 * (py - 8)) - 50);
        }
    }

    // Camera default coordinates
    setVector(&cam_pos, 0, 0, 0);
    setVector(&cam_rot, 0, 0, 0);
    int targetZ = 2;
    int targetX = 2;
    int targetRot = 0;
    cam_pos = {(start.x*600)<<12,-250<<12,(start.y*600)<<12};
    // Main loop
    while (1)
    {
        const auto playerPosition = cam_pos;
        // Set pad buffer data to pad pointer
        pad = (PADTYPE *)&pad_buff[0][0];

        if (pad->stat == 0)
        {
            const char LSX = pad->ls_x - 128;
            const char LSY = pad->ls_y - 128;
            const char RSX = pad->rs_x - 128;
            const char RSY = pad->rs_y - 128;

            const auto playerCanMove = abs(playerPosition.vz - targetZ) < 5 
                                       && abs(playerPosition.vx - targetX) < 5
                                       && abs(targetRot - cam_rot.vy) < 100;

            if(playerCanMove)
            {
                if(!(pad->btn&PAD_L1))
                {
                    currentTileZ += (icos(targetRot+1024)>>12);
                    currentTileX -= (isin(targetRot+1024)>>12);
                }
                if(!(pad->btn&PAD_R1))
                {
                    currentTileZ += (icos(targetRot-1024)>>12);
                    currentTileX -= (isin(targetRot-1024)>>12);
                }

                if(!(pad->btn&PAD_UP))
                {
                    currentTileZ += (icos(targetRot)>>12);
                    currentTileX -= (isin(targetRot)>>12);
                }
                if(!(pad->btn&PAD_DOWN))
                {
                    currentTileZ -= (icos(targetRot)>>12);
                    currentTileX += (isin(targetRot)>>12);
                }
                

                if(!(pad->btn&PAD_LEFT))
                {
                    targetRot += 1024;
                }
                if(!(pad->btn&PAD_RIGHT))
                {
                    targetRot -= 1024;
                }
            }

            if(currentTileX >= 0 && currentTileX <= 39 &&
               currentTileZ >= 0 && currentTileZ <= 39 &&
               level_01.collision[currentTileX][currentTileZ]==0)
            {
                targetZ = (currentTileZ * 600)<<12;
                targetX = (currentTileX * 600)<<12;
                lastTileX = currentTileX;
                lastTileZ = currentTileZ;
            }
            else
            {
                currentTileX = lastTileX;
                currentTileZ = lastTileZ;
            }

            static constexpr auto moveSpeed= 20<<12;
            
            if(cam_pos.vz > targetZ)
                cam_pos = {cam_pos.vx, cam_pos.vy , cam_pos.vz - moveSpeed};
            if(cam_pos.vz < targetZ)
                cam_pos = {cam_pos.vx, cam_pos.vy , cam_pos.vz + moveSpeed};

            if(cam_pos.vx > targetX)
                cam_pos = {cam_pos.vx - moveSpeed, cam_pos.vy , cam_pos.vz};
            if(cam_pos.vx < targetX)
                cam_pos = {cam_pos.vx + moveSpeed, cam_pos.vy , cam_pos.vz};

            if(cam_rot.vy < targetRot)
                cam_rot.vy += 64;
            if(cam_rot.vy > targetRot)
                cam_rot.vy -= 64;
            if(cam_rot.vy - targetRot < 68 && cam_rot.vy - targetRot > -68)
                cam_rot.vy = targetRot;
        }        

        FntPrint(-1, "FPS=%d\n",
                 fps);
        // Print out some info
        FntPrint(-1, "BUTTONS=%04x\n", pad->btn);
        
        // First-person camera mode
        RotMatrix(&cam_rot, &mtx);
        
        tpos.vx = -cam_pos.vx >> 12;
        tpos.vy = -cam_pos.vy >> 12;
        tpos.vz = -cam_pos.vz >> 12;
        ApplyMatrixLV(&mtx, &tpos, &tpos);
        TransMatrix(&mtx, &tpos);

        // Set rotation and translation matrix
        gte_SetRotMatrix(&mtx);
        gte_SetTransMatrix(&mtx);
        
        //draw_tree(&mtx, &position, &treeRot);
        draw_map(&mtx);


        // Flush text to drawing area
        FntFlush(-1);

        // Swap buffers and draw the primitives
        graphics->display();
    }

    return 0;
}

template<typename T>
void render3DModel(const T& model, TIM_IMAGE* texture)
{
    const auto tri_size = sizeof(model.tris)/sizeof(north_wall::face3);
    for(int tri = 0; tri < tri_size; ++tri)
    {
        const auto triIndex = model.tris[tri];
        const SVECTOR triangle[3] = {
            model.vertices[triIndex.vertice0],
            model.vertices[triIndex.vertice1],
            model.vertices[triIndex.vertice2]
        };

        const SVECTOR normal = model.normals[triIndex.normal0];
		const DVECTOR uvs[] = {model.uvs[triIndex.uv1], model.uvs[triIndex.uv0], model.uvs[triIndex.uv2]};
        const CVECTOR colors[] = {{63,63,63}};
        graphics->Draw<POLY_FT3>(triangle, normal, texture, uvs, colors);
    }
    const auto quad_size = sizeof(model.quads)/sizeof(north_wall::face4);
    for(int index = 0; index < quad_size; ++index)
    {
        const auto triIndex = model.quads[index];
        const SVECTOR quad[4] = {
            model.vertices[triIndex.vertice1],
            model.vertices[triIndex.vertice0],
            model.vertices[triIndex.vertice2],
            model.vertices[triIndex.vertice3]
        };

        const SVECTOR normal = model.normals[triIndex.normal0];
        const DVECTOR uvs[] = {model.uvs[triIndex.uv1], model.uvs[triIndex.uv0], model.uvs[triIndex.uv2], model.uvs[triIndex.uv3]};
        const CVECTOR colors[] = {{63,63,63}};
        graphics->Draw<POLY_FT4>(quad, normal, texture, uvs, colors);
    }
}

#define min2(a,b) ((a<b)?a:b)
#define max2(a,b) ((a>b)?a:b)

template<typename T>
int min(T a,T b) { return (a<b?a:b);}

template<typename T>
int min(T a,T b, T c) { return min(a,min(b,c));}

template<typename T>
int max(T a,T b) { return (a>b?a:b);}   

template<typename T>
int max(T a,T b, T c) { return max(a,max(b,c));}

void draw_map(MATRIX *mtx)
{

    int i, p;
    POLY_F4 *pol4;

    constexpr auto mapsize = 6;

    static constexpr ground tileGround {.texture = &ground_texture};
    static constexpr ceilling tileCeilling {.texture = &ground_texture};

    static constexpr north_wall fwall {.texture = &ground_texture};
    static constexpr south_wall bwall {.texture = &ground_texture};
    static constexpr east_wall lwall {.texture = &ground_texture};
    static constexpr west_wall rwall {.texture = &ground_texture};

    const auto startx = currentTileX-mapsize>0?currentTileX-mapsize:0;
    const auto endx = currentTileX+mapsize<40?currentTileX+mapsize:40;
    const auto startz = currentTileZ-mapsize>0?currentTileZ-mapsize:0;
    const auto endz = currentTileZ+mapsize<40?currentTileZ+mapsize:40;
    for(int x=startx; x<endx; ++x)
    {
        const int east_map_idx = max(0, x - 1);
        const int west_map_idx = min(x + 1, endx);

        for(int z=startz; z<endz; ++z)
        {
            const int south_map_idx = max(0, z - 1);
            const int north_map_idx = min(z + 1, endz);

            //PushMatrix();
            MATRIX rmtx{0};

            VECTOR mobPositin = {x*600,0,z*600};

            TransMatrix( &rmtx, &mobPositin );
            CompMatrixLV( mtx, &rmtx, &rmtx );

            gte_SetTransMatrix( &rmtx );

            static constexpr int OPEN_TILE = 1;
            static constexpr int COLLISION_TILE = 2;
            static constexpr int GOBLIN_TILE = 63;

            if(map[x][z]==OPEN_TILE)
            {
                render3DModel(tileCeilling, tileGround.texture);
                render3DModel(tileGround, tileGround.texture);
            }
            if(map[x][z]==COLLISION_TILE)
            {
                if(map[west_map_idx][z]==1)
                    render3DModel(rwall, rwall.texture);
                if(map[east_map_idx][z]==1)
                    render3DModel(lwall, lwall.texture);
                if(map[x][south_map_idx]==1)
                    render3DModel(south_wall{}, bwall.texture);
                if(map[x][north_map_idx]==1)
                    render3DModel(north_wall{}, fwall.texture);
            }
            if(mobs[x][z] == GOBLIN_TILE)
            {
                render3DModel(enemy_medium{}, &goblin_texture);
            }

            //PopMatrix();            
        }
    }
    
}

