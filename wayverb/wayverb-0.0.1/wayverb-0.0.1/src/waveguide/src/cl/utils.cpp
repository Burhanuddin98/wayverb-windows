#include "waveguide/cl/utils.h"

namespace wayverb {
namespace waveguide {
namespace cl_sources {

const char* utils{R"(
#define no_neighbor (~(uint)(0))
#define PORTS (6)
#define EDGE_PORTS (12)

typedef enum {
    id_port_nx = 0,
    id_port_px = 1,
    id_port_ny = 2,
    id_port_py = 3,
    id_port_nz = 4,
    id_port_pz = 5,
} PortDirection;

//  IWB edge-diagonal directions: 12 neighbors at distance sqrt(2)*h.
//  Each is a combination of two face directions.
typedef enum {
    id_edge_nxny = 0,
    id_edge_nxpy = 1,
    id_edge_pxny = 2,
    id_edge_pxpy = 3,
    id_edge_nxnz = 4,
    id_edge_nxpz = 5,
    id_edge_pxnz = 6,
    id_edge_pxpz = 7,
    id_edge_nynz = 8,
    id_edge_nypz = 9,
    id_edge_pynz = 10,
    id_edge_pypz = 11,
} EdgeDirection;

bool locator_outside(int3 locator, int3 dim);
bool locator_outside(int3 locator, int3 dim) {
    return any(locator < (int3)(0)) || any(dim <= locator);
}

int3 to_locator(size_t index, int3 dim);
int3 to_locator(size_t index, int3 dim) {
    const int xrem = index % dim.x, xquot = index / dim.x;
    const int yrem = xquot % dim.y, yquot = xquot / dim.y;
    const int zrem = yquot % dim.z;
    return (int3)(xrem, yrem, zrem);
}

size_t to_index(int3 locator, int3 dim);
size_t to_index(int3 locator, int3 dim) {
    return locator.x + locator.y * dim.x + locator.z * dim.x * dim.y;
}

uint neighbor_index(int3 locator, int3 dim, PortDirection pd);
uint neighbor_index(int3 locator, int3 dim, PortDirection pd) {
    switch (pd) {
        case id_port_nx: {
            locator += (int3)(-1, 0, 0);
            break;
        }
        case id_port_px: {
            locator += (int3)(1, 0, 0);
            break;
        }
        case id_port_ny: {
            locator += (int3)(0, -1, 0);
            break;
        }
        case id_port_py: {
            locator += (int3)(0, 1, 0);
            break;
        }
        case id_port_nz: {
            locator += (int3)(0, 0, -1);
            break;
        }
        case id_port_pz: {
            locator += (int3)(0, 0, 1);
            break;
        }
    }
    if (locator_outside(locator, dim))
        return no_neighbor;
    return to_index(locator, dim);
}

//  Edge-diagonal neighbor index: returns the index of the neighbor at the
//  given edge direction, or no_neighbor if outside the mesh.
uint edge_neighbor_index(int3 locator, int3 dim, EdgeDirection ed);
uint edge_neighbor_index(int3 locator, int3 dim, EdgeDirection ed) {
    switch (ed) {
        case id_edge_nxny: locator += (int3)(-1, -1,  0); break;
        case id_edge_nxpy: locator += (int3)(-1,  1,  0); break;
        case id_edge_pxny: locator += (int3)( 1, -1,  0); break;
        case id_edge_pxpy: locator += (int3)( 1,  1,  0); break;
        case id_edge_nxnz: locator += (int3)(-1,  0, -1); break;
        case id_edge_nxpz: locator += (int3)(-1,  0,  1); break;
        case id_edge_pxnz: locator += (int3)( 1,  0, -1); break;
        case id_edge_pxpz: locator += (int3)( 1,  0,  1); break;
        case id_edge_nynz: locator += (int3)( 0, -1, -1); break;
        case id_edge_nypz: locator += (int3)( 0, -1,  1); break;
        case id_edge_pynz: locator += (int3)( 0,  1, -1); break;
        case id_edge_pypz: locator += (int3)( 0,  1,  1); break;
    }
    if (locator_outside(locator, dim))
        return no_neighbor;
    return to_index(locator, dim);
}

//  Sum pressures at all 12 edge-diagonal neighbors.
//  Returns the sum and the count of valid (inside-mesh) neighbors via pointer.
float sum_edge_neighbors(const global float* current,
                         int3 locator, int3 dim, int* count);
float sum_edge_neighbors(const global float* current,
                         int3 locator, int3 dim, int* count) {
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < EDGE_PORTS; ++i) {
        const uint idx = edge_neighbor_index(locator, dim, (EdgeDirection)i);
        if (idx != no_neighbor) {
            sum += current[idx];
            ++n;
        }
    }
    *count = n;
    return sum;
}

float3 compute_node_position(const mesh_descriptor descriptor, int3 locator);
float3 compute_node_position(const mesh_descriptor descriptor, int3 locator) {
    return descriptor.min_corner + convert_float3(locator) * descriptor.spacing;
}

)"};

}  // namespace cl_sources
}  // namespace waveguide
}  // namespace wayverb
