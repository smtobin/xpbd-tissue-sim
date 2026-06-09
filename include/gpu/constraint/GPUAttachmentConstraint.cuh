#ifndef __GPU_ATTACHMENT_CONSTRAINT_CUH
#define __GPU_ATTACHMENT_CONSTRAINT_CUH

#include "gpu/common/gcc_hostdevice.hpp"
#include "gpu/common/helper.cuh"
#include "gpu/constraint/GPUPositionReference.cuh"

#include "utils/CudaHelperMath.h"

struct GPUOffsetAttachmentConstraint
{
    float3 attached_pos;
    float3 attachment_offset;
    GPUPositionReference positions[1];
    float alpha;

    __host__ GPUOffsetAttachmentConstraint( int v0_ind, float inv_m0,
                                      const Vec3r& attach_pt, const Vec3r& offset,
                                      float alpha_)
    {
        positions[0].inv_mass = inv_m0;
        positions[0].index = v0_ind;

        attached_pos.x = attach_pt[0];
        attached_pos.y = attach_pt[1];
        attached_pos.z = attach_pt[2];

        attachment_offset.x = offset[0];
        attachment_offset.y = offset[1];
        attachment_offset.z = offset[2];

        alpha = alpha_;
    }

    __device__ GPUOffsetAttachmentConstraint() {}

    constexpr __host__ __device__ static int numPositions() { return 1; } 

    constexpr __host__ __device__ static bool isInequality() { return false; }

    // __device__ void _loadVertices(float* x)
    // {
    //     x[0] = positions[0].ptr[0];  x[1] = positions[0].ptr[1];  x[2] = positions[0].ptr[2];
    //     x[3] = positions[1].ptr[0];  x[4] = positions[1].ptr[1];  x[5] = positions[1].ptr[2];
    //     x[6] = positions[2].ptr[0];  x[7] = positions[2].ptr[1];  x[8] = positions[2].ptr[2];
    //     x[9] = positions[3].ptr[0];  x[10] = positions[3].ptr[1]; x[11] = positions[3].ptr[2];
    // }
    __device__ void _loadVertices(const float* vertices, float3* x) const
    {
        *reinterpret_cast<float3*>(x)   = *reinterpret_cast<const float3*>(vertices + 3*positions[0].index);
    }

    __device__ void evaluate(const float* vertices, float* C) const
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 diff = x - (attached_pos + attachment_offset);
        *C = length(diff);
    }

    __device__ void gradient(const float* vertices, float* delC) const
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 attach_pt = attached_pos + attachment_offset;
        const float dist = length( x - attach_pt );
        delC[0] = (x.x - attach_pt.x) / dist;
        delC[1] = (x.y - attach_pt.y) / dist;
        delC[2] = (x.z - attach_pt.z) / dist;
    }

    __device__ void evaluateWithGradient(const float* vertices, float* C, float* delC) const
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 attach_pt = attached_pos + attachment_offset;
        const float dist = length( x - attach_pt );
        *C = dist;
        
        delC[0] = (x.x - attach_pt.x) / dist;
        delC[1] = (x.y - attach_pt.y) / dist;
        delC[2] = (x.z - attach_pt.z) / dist;
    }
};

#endif // __GPU_ATTACHMENT_CONSTRAINT_CUH