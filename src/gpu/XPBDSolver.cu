#include "gpu/XPBDSolver.cuh"

// #include "gpu/common/helper.cuh"
#include "../src/gpu/common/helper.cu"

#include "gpu/constraint/GPUHydrostaticConstraint.cuh"
#include "gpu/constraint/GPUDeviatoricConstraint.cuh"
#include "gpu/constraint/GPUStaticDeformableCollisionConstraint.cuh"
#include "gpu/constraint/GPURigidDeformableCollisionConstraint.cuh"
#include "gpu/constraint/GPUOffsetAttachmentConstraint.cuh"

#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#include "gpu/projector/GPUConstraintProjector.cuh"
#include "gpu/projector/GPUNeohookeanCombinedConstraintProjector.cuh"

__device__ void ElementJacobiSolve(int elem_index, const int* elements, int num_elements, const float* vertices, const float* masses, const float* volumes, const float* Qs, float lambda, float mu, float dt, float* new_vertices)
{
    // extract quantities for element
    int elem[4];
    for (int i = 0; i < 4; i++) { elem[i] = elements[4*elem_index + i]; }

    const float* elem0_ptr = vertices + 3*elem[0];
    const float* elem1_ptr = vertices + 3*elem[1];
    const float* elem2_ptr = vertices + 3*elem[2];
    const float* elem3_ptr = vertices + 3*elem[3];
    float x[12];
    x[0] = elem0_ptr[0];  x[1] = elem0_ptr[1];  x[2] = elem0_ptr[2];
    x[3] = elem1_ptr[0];  x[4] = elem1_ptr[1];  x[5] = elem1_ptr[2];
    x[6] = elem2_ptr[0];  x[7] = elem2_ptr[1];  x[8] = elem2_ptr[2];
    x[9] = elem3_ptr[0];  x[10] = elem3_ptr[1]; x[11] = elem3_ptr[2];

    float Q[9];
    for (int i = 0; i < 9; i++) { Q[i] = Qs[9*elem_index+i]; }

    float inv_m[4];
    for (int i = 0; i < 4; i++) { inv_m[i] = 1.0/masses[elem[i]]; }
    
    
    const float gamma = mu / lambda;

    // compute F
    float X[9];
    X[0] = x[0] - x[9]; X[1] = x[1] - x[10]; X[2] = x[2] - x[11];
    X[3] = x[3] - x[9]; X[4] = x[4] - x[10]; X[5] = x[5] - x[11];
    X[6] = x[6] - x[9]; X[7] = x[7] - x[10]; X[8] = x[8] - x[11];

    // X[0] = elem0_ptr[0] - x4.x; X[1] = elem0_ptr[1] - x4.y; X[2] = elem0_ptr[2] - x4.z;
    // X[3] = elem1_ptr[0] - x4.x; X[4] = elem1_ptr[1] - x4.y; X[5] = elem1_ptr[2] - x4.z;
    // X[6] = elem2_ptr[0] - x4.x; X[7] = elem2_ptr[1] - x4.y; X[8] = elem2_ptr[2] - x4.z;
    
    float F[9];
    Mat3Mul(X, Q, F);

    // compute hydrostatic constraint and its gradient
    float C_h_grad[12];

    // C_h = det(F) - (1 + gamma)
    const float C_h = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+gamma);

    const float alpha_h = 1.0/(lambda * volumes[elem_index]);

    float F_cross[9];
    Vec3Cross(F+3, F+6, F_cross);   // 2nd column of F crossed with 3rd column
    Vec3Cross(F+6, F, F_cross+3);   // 3rd column of F crossed with 1st column
    Vec3Cross(F, F+3, F_cross+6);   // 1st column of F crossed with 2nd column

    Mat3MulTranspose(F_cross, Q, C_h_grad);
    C_h_grad[9] = -C_h_grad[0] - C_h_grad[3] - C_h_grad[6];
    C_h_grad[10] = -C_h_grad[1] - C_h_grad[4] - C_h_grad[7];
    C_h_grad[11] = -C_h_grad[2] - C_h_grad[5] - C_h_grad[8];

    // printf("Ch_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_h_grad[0], C_h_grad[1], C_h_grad[2], C_h_grad[3], C_h_grad[4], C_h_grad[5], C_h_grad[6], C_h_grad[7], C_h_grad[8], C_h_grad[9], C_h_grad[10], C_h_grad[11]);

    // compute deviatoric constraint and its gradient
    float C_d_grad[12];

    // C_d = frob(F)
    const float C_d = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    const float inv_C_d = 1.0/C_d;

    const float alpha_d = 1.0/(mu * volumes[elem_index]);

    Mat3MulTranspose(F, Q, C_d_grad);
    for (int i = 0; i < 9; i++) { C_d_grad[i] *= inv_C_d; }
    C_d_grad[9] =  -C_d_grad[0] - C_d_grad[3] - C_d_grad[6];
    C_d_grad[10] = -C_d_grad[1] - C_d_grad[4] - C_d_grad[7];
    C_d_grad[11] = -C_d_grad[2] - C_d_grad[5] - C_d_grad[8];

    // printf("Cd_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_d_grad[0], C_d_grad[1], C_d_grad[2], C_d_grad[3], C_d_grad[4], C_d_grad[5], C_d_grad[6], C_d_grad[7], C_d_grad[8], C_d_grad[9], C_d_grad[10], C_d_grad[11]);

    // solve the 2x2 system
    float A[4];
    A[0] = alpha_h / (dt * dt);
    A[1] = 0;
    A[2] = 0;
    A[3] = alpha_d / (dt * dt);
    for (int i = 0; i < 4; i++)
    {
        A[0] += inv_m[i] * (C_h_grad[3*i]*C_h_grad[3*i] + C_h_grad[3*i+1]*C_h_grad[3*i+1] + C_h_grad[3*i+2]*C_h_grad[3*i+2]);
        A[3] += inv_m[i] * (C_d_grad[3*i]*C_d_grad[3*i] + C_d_grad[3*i+1]*C_d_grad[3*i+1] + C_d_grad[3*i+2]*C_d_grad[3*i+2]);
        A[1] += inv_m[i] * (C_h_grad[3*i]*C_d_grad[3*i] + C_h_grad[3*i+1]*C_d_grad[3*i+1] + C_h_grad[3*i+2]*C_d_grad[3*i+2]);
    }
    A[2] = A[1];

    float k[2];
    k[0] = -C_h; // TODO: include current lambda (assuming its 0 right now)
    k[1] = -C_d; // TODO: include current lambda

    const float detA = A[0]*A[3] - A[1]*A[2];
    const float dlam_h = (k[0]*A[3] - k[1]*A[2]) / detA;
    const float dlam_d = (k[1]*A[0] - k[0]*A[1]) / detA;

    // printf("elem: %i  dlam_h: %.10f  dlam_d: %.10f\n", elem_index, dlam_h, dlam_d);
    // const float dlam_h = 0;
    // const float dlam_d = 0;

    // // compute the coordinate updates
    for (int i = 0; i < 4; i++)
    {
        float pos_update_x = inv_m[i] * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        float pos_update_y = inv_m[i] * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        float pos_update_z = inv_m[i] * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
        // printf("pos update %i: %f, %f, %f\n", i, pos_update_x, pos_update_y, pos_update_z);
        float* v_ptr = new_vertices + 3*elem[i];
        atomicAdd(v_ptr, pos_update_x);
        atomicAdd(v_ptr + 1, pos_update_y);
        atomicAdd(v_ptr + 2, pos_update_z);
        // coord_updates[12*elem_index + 3*i] =   inv_m[i] * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        // coord_updates[12*elem_index + 3*i+1] = inv_m[i] * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        // coord_updates[12*elem_index + 3*i+2] = inv_m[i] * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
    }

    // printf("new gpu verts:\nv1: %f, %f, %f\nv2: %f, %f, %f\nv3: %f, %f, %f\nv4: %f, %f, %f\n", new_vertices[0], new_vertices[1], new_vertices[2], new_vertices[3], new_vertices[4], new_vertices[5], new_vertices[6], new_vertices[7], new_vertices[8], new_vertices[9], new_vertices[10], new_vertices[11]);
}
__global__ void XPBDJacobiSolve(const int* elements, int num_elements, const float* vertices, const float* masses, const float* volumes, const float* Qs, float lambda, float mu, float dt, float* new_vertices)
{
    int elem_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (elem_index >= num_elements)    return;

    ElementJacobiSolve(elem_index, elements, num_elements, vertices, masses, volumes, Qs, lambda, mu, dt, new_vertices);
}

__host__ void LaunchXPBDJacobiSolve(const int* elements, int num_elements, const float* vertices,
    const float* masses, const float* volumes, const float* Qs,
    float lambda, float mu, float dt,
    float* new_vertices)
{
    int num_blocks = (num_elements + CUDA_BLOCK_SIZE - 1) / CUDA_BLOCK_SIZE;
    XPBDJacobiSolve<<<num_blocks, CUDA_BLOCK_SIZE>>>(elements, num_elements, vertices, masses, volumes, Qs, lambda, mu, dt, new_vertices);
}

__host__ void LaunchCopyVertices(const float* src_vertices, float* dst_vertices, int num_vertices)
{
    int num_blocks = (3*num_vertices + CUDA_BLOCK_SIZE - 1) / CUDA_BLOCK_SIZE;
    CopyVertices<<<num_blocks, CUDA_BLOCK_SIZE>>>(src_vertices, dst_vertices, 3*num_vertices);
}

template<class ConstraintProjector>
__global__ void ProjectConstraints(ConstraintProjector* projectors, int num_projectors, const float* vertices, float* new_vertices)
{
    int proj_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (proj_index >= num_projectors)   return;

    // load ConstraintProjectors into shared mem using coalesced global memory accesses
    int proj_size_float = sizeof(ConstraintProjector) / sizeof(float);
    int proj_global_start = proj_size_float * BLOCK_SIZE * blockIdx.x;

    __shared__ ConstraintProjector proj_smem[BLOCK_SIZE];

    float* proj_smem_float = reinterpret_cast<float*>(proj_smem);
    float* proj_gmem_float = reinterpret_cast<float*>(projectors + BLOCK_SIZE * blockIdx.x);
    int skip = min(num_projectors - blockIdx.x * BLOCK_SIZE, BLOCK_SIZE);
    for (int i = 0; i < sizeof(ConstraintProjector)/sizeof(float); i++)
    {
        proj_smem_float[threadIdx.x + skip*i] = proj_gmem_float[threadIdx.x + skip*i];
        // proj_smem_float[i] = proj_gmem_float[i];
    }
    __syncthreads();

    ConstraintProjector* bad_proj = proj_smem + threadIdx.x;
    if (bad_proj->valid)
    {
        bad_proj->initialize();
        bad_proj->project(vertices, new_vertices);
    }
}

template<class ConstraintProjector>
__host__ void LaunchProjectConstraints(ConstraintProjector* projectors, int num_projectors, const float* vertices, float* new_vertices)
{
    int num_blocks = (num_projectors + BLOCK_SIZE - 1) / BLOCK_SIZE;
    ProjectConstraints<ConstraintProjector><<<num_blocks, BLOCK_SIZE>>>(projectors, num_projectors, vertices, new_vertices);
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

template __host__ void LaunchProjectConstraints<GPUCombinedConstraintProjector<true, GPUDeviatoricConstraint, GPUHydrostaticConstraint>> (GPUCombinedConstraintProjector<true, GPUDeviatoricConstraint, GPUHydrostaticConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<true, GPUDeviatoricConstraint>> (GPUConstraintProjector<true, GPUDeviatoricConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<true, GPUHydrostaticConstraint>> (GPUConstraintProjector<true, GPUHydrostaticConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<true, GPUStaticDeformableCollisionConstraint>> (GPUConstraintProjector<true, GPUStaticDeformableCollisionConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<true, GPURigidDeformableCollisionConstraint>> (GPUConstraintProjector<true, GPURigidDeformableCollisionConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<true, GPUOffsetAttachmentConstraint>> (GPUConstraintProjector<true, GPUOffsetAttachmentConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);

template __host__ void LaunchProjectConstraints<GPUCombinedConstraintProjector<false, GPUDeviatoricConstraint, GPUHydrostaticConstraint>> (GPUCombinedConstraintProjector<false, GPUDeviatoricConstraint, GPUHydrostaticConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<false, GPUDeviatoricConstraint>> (GPUConstraintProjector<false, GPUDeviatoricConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<false, GPUHydrostaticConstraint>> (GPUConstraintProjector<false, GPUHydrostaticConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<false, GPUStaticDeformableCollisionConstraint>> (GPUConstraintProjector<false, GPUStaticDeformableCollisionConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<false, GPURigidDeformableCollisionConstraint>> (GPUConstraintProjector<false, GPURigidDeformableCollisionConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);
template __host__ void LaunchProjectConstraints<GPUConstraintProjector<false, GPUOffsetAttachmentConstraint>> (GPUConstraintProjector<false, GPUOffsetAttachmentConstraint>* projectors, int num_projectors, const float* vertices, float* new_vertices);

__global__ void CopyVertices(const float* src_vertices, float* dst_vertices, int num_vertices)
{
    int coord_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (coord_index >= 3*num_vertices) return;

    dst_vertices[coord_index] = src_vertices[coord_index];
}

__host__ void CopyVerticesMemcpy(float* dst_vertices, const float* src_vertices, int num_vertices)
{
    CHECK_CUDA_ERROR(cudaMemcpy(dst_vertices, src_vertices, num_vertices*3*sizeof(float), cudaMemcpyDeviceToDevice));
}




// template <>
template<bool IsFirstOrder>
__host__ GPUCombinedConstraintProjector<IsFirstOrder, GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector(const GPUDeviatoricConstraint& constraint1_, const GPUHydrostaticConstraint& constraint2_, float dt_)
    : dt(dt_), alpha_d(constraint1_.alpha), alpha_h(constraint2_.alpha), gamma(constraint2_.gamma), valid(true)
{
    for (int i = 0; i < 4; i++)
    {
        positions[i].index = constraint1_.positions[i].index;
        positions[i].inv_mass = constraint1_.positions[i].inv_mass;
    }
    // printf("indices: %i, %i, %i, %i\n", positions[0].index, positions[1].index, positions[2].index, positions[3].index);

    for (int i = 0; i < 9; i++)
    {
        Q[i] = constraint1_.Q[i];
    }
}
    
// template <>
template<bool IsFirstOrder>
__host__ GPUCombinedConstraintProjector<IsFirstOrder, GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector(GPUDeviatoricConstraint&& constraint1_, GPUHydrostaticConstraint&& constraint2_, float dt_)
    : dt(dt_), alpha_d(constraint1_.alpha), alpha_h(constraint2_.alpha), gamma(constraint2_.gamma), valid(true)
{
    for (int i = 0; i < 4; i++)
    {
        positions[i].index = constraint1_.positions[i].index;
        positions[i].inv_mass = constraint1_.positions[i].inv_mass;
    }

    for (int i = 0; i < 9; i++)
    {
        Q[i] = constraint1_.Q[i];
    }
}

// template<>
template<bool IsFirstOrder>
__device__ GPUCombinedConstraintProjector<IsFirstOrder, GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector()
    : valid(false)
{

}

// template<>
template<bool IsFirstOrder>
__device__ void GPUCombinedConstraintProjector<IsFirstOrder, GPUDeviatoricConstraint, GPUHydrostaticConstraint>::initialize()
{
    lambda[0] = 0;
    lambda[1] = 0;
}

// template<>
template<bool IsFirstOrder>
__device__ void GPUCombinedConstraintProjector<IsFirstOrder, GPUDeviatoricConstraint, GPUHydrostaticConstraint>::project(const float* vertices, float* new_vertices)
{
    // printf("indices: %i, %i, %i, %i\n", positions[0].index, positions[1].index, positions[2].index, positions[3].index);
    float x[12];
    *reinterpret_cast<float3*>(x) = *reinterpret_cast<const float3*>(vertices + 3*positions[0].index);
    *reinterpret_cast<float3*>(x+3) = *reinterpret_cast<const float3*>(vertices + 3*positions[1].index);
    *reinterpret_cast<float3*>(x+6) = *reinterpret_cast<const float3*>(vertices + 3*positions[2].index);
    *reinterpret_cast<float3*>(x+9) = *reinterpret_cast<const float3*>(vertices + 3*positions[3].index);
    // x[0] = 0; x[1] = 0; x[2] = 0; x[3] = 0; x[4] = 0; x[5] = 0;
    // x[6] = 0; x[7] = 0; x[8] = 0; x[9] = 0; x[10] = 0; x[11] = 0;

    // compute F
    float X[9];
    X[0] = x[0] - x[9]; X[1] = x[1] - x[10]; X[2] = x[2] - x[11];
    X[3] = x[3] - x[9]; X[4] = x[4] - x[10]; X[5] = x[5] - x[11];
    X[6] = x[6] - x[9]; X[7] = x[7] - x[10]; X[8] = x[8] - x[11];
    
    float F[9];
    Mat3Mul(X, Q, F);

    // compute hydrostatic constraint and its gradient
    float C_h_grad[12];

    // C_h = det(F) - (1 + gamma)
    const float C_h = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+gamma);

    float F_cross[9];
    Vec3Cross(F+3, F+6, F_cross);   // 2nd column of F crossed with 3rd column
    Vec3Cross(F+6, F, F_cross+3);   // 3rd column of F crossed with 1st column
    Vec3Cross(F, F+3, F_cross+6);   // 1st column of F crossed with 2nd column

    Mat3MulTranspose(F_cross, Q, C_h_grad);
    C_h_grad[9] = -C_h_grad[0] - C_h_grad[3] - C_h_grad[6];
    C_h_grad[10] = -C_h_grad[1] - C_h_grad[4] - C_h_grad[7];
    C_h_grad[11] = -C_h_grad[2] - C_h_grad[5] - C_h_grad[8];

    // printf("Ch_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_h_grad[0], C_h_grad[1], C_h_grad[2], C_h_grad[3], C_h_grad[4], C_h_grad[5], C_h_grad[6], C_h_grad[7], C_h_grad[8], C_h_grad[9], C_h_grad[10], C_h_grad[11]);

    // compute deviatoric constraint and its gradient
    float C_d_grad[12];

    // C_d = frob(F)
    const float C_d = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    const float inv_C_d = 1.0/C_d;

    Mat3MulTranspose(F, Q, C_d_grad);
    for (int i = 0; i < 9; i++) { C_d_grad[i] *= inv_C_d; }
    C_d_grad[9] =  -C_d_grad[0] - C_d_grad[3] - C_d_grad[6];
    C_d_grad[10] = -C_d_grad[1] - C_d_grad[4] - C_d_grad[7];
    C_d_grad[11] = -C_d_grad[2] - C_d_grad[5] - C_d_grad[8];

    // printf("Cd_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_d_grad[0], C_d_grad[1], C_d_grad[2], C_d_grad[3], C_d_grad[4], C_d_grad[5], C_d_grad[6], C_d_grad[7], C_d_grad[8], C_d_grad[9], C_d_grad[10], C_d_grad[11]);

    // solve the 2x2 system
    float A[4];

    float alpha_h_tilde;
    float alpha_d_tilde;
    if constexpr (IsFirstOrder)
    {
        alpha_h_tilde = alpha_h / dt;
        alpha_d_tilde = alpha_d / dt;
    }
    else
    {
        alpha_h_tilde = alpha_h / (dt * dt);
        alpha_d_tilde = alpha_d / (dt * dt);
    }

    A[0] = alpha_d_tilde;
    A[3] = alpha_h_tilde;
    A[1] = 0;
    A[2] = 0;

    for (int i = 0; i < 4; i++)
    {
        A[0] += positions[i].inv_mass * (C_d_grad[3*i]*C_d_grad[3*i] + C_d_grad[3*i+1]*C_d_grad[3*i+1] + C_d_grad[3*i+2]*C_d_grad[3*i+2]);
        A[3] += positions[i].inv_mass * (C_h_grad[3*i]*C_h_grad[3*i] + C_h_grad[3*i+1]*C_h_grad[3*i+1] + C_h_grad[3*i+2]*C_h_grad[3*i+2]);
        A[1] += positions[i].inv_mass * (C_h_grad[3*i]*C_d_grad[3*i] + C_h_grad[3*i+1]*C_d_grad[3*i+1] + C_h_grad[3*i+2]*C_d_grad[3*i+2]);
    }
    A[2] = A[1];

    float k[2];
    k[0] = -C_d - alpha_d_tilde * lambda[0]; // TODO: include current lambda (assuming its 0 right now)
    k[1] = -C_h - alpha_h_tilde * lambda[1]; // TODO: include current lambda

    const float detA = A[0]*A[3] - A[1]*A[2];
    const float dlam_d = (k[0]*A[3] - k[1]*A[2]) / detA;
    const float dlam_h = (k[1]*A[0] - k[0]*A[1]) / detA;

    lambda[0] += dlam_d;
    lambda[1] += dlam_h;

    // printf("dlam_h: %.10f  dlam_d: %.10f\n", dlam_h, dlam_d);
    // const float dlam_h = 0;
    // const float dlam_d = 0;

    // // compute the coordinate updates
    for (int i = 0; i < 4; i++)
    {
        float pos_update_x = positions[i].inv_mass * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        float pos_update_y = positions[i].inv_mass * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        float pos_update_z = positions[i].inv_mass * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
        // printf("pos update %i: %f, %f, %f\n", i, pos_update_x, pos_update_y, pos_update_z);
        float* v_ptr = new_vertices + 3*positions[i].index;
        atomicAdd(v_ptr, pos_update_x);
        atomicAdd(v_ptr + 1, pos_update_y);
        atomicAdd(v_ptr + 2, pos_update_z);
        // coord_updates[12*elem_index + 3*i] =   inv_m[i] * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        // coord_updates[12*elem_index + 3*i+1] = inv_m[i] * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        // coord_updates[12*elem_index + 3*i+2] = inv_m[i] * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
    }
}

template class GPUCombinedConstraintProjector<true, GPUDeviatoricConstraint, GPUHydrostaticConstraint>;
template class GPUCombinedConstraintProjector<false, GPUDeviatoricConstraint, GPUHydrostaticConstraint>;