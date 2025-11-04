// #include <petsc.h>
#include <petscksp.h>

#include <iostream>

#define N 10

int main()
{
    Mat A;              // Stiffness matrix
    Vec b, x;           // Load vector and solution vector
    KSP ksp;            // Linear solver context
    PetscErrorCode ierr;
    PetscInt its;
    PetscReal norm, error;
    
    ierr = PetscInitialize(NULL, NULL, NULL, NULL); CHKERRQ(ierr);
    
    // Create matrix and vectors
    ierr = MatCreate(PETSC_COMM_WORLD, &A); CHKERRQ(ierr);
    ierr = MatSetSizes(A, PETSC_DECIDE, PETSC_DECIDE, N, N); 
           CHKERRQ(ierr);
    ierr = MatSetFromOptions(A); CHKERRQ(ierr);
    ierr = MatSetUp(A); CHKERRQ(ierr);
    
    // Preallocate matrix (approximately 9 non-zeros per row for structured mesh)
    ierr = MatSeqAIJSetPreallocation(A, 9, NULL); CHKERRQ(ierr);
    ierr = MatMPIAIJSetPreallocation(A, 9, NULL, 9, NULL); CHKERRQ(ierr);
    
    ierr = VecCreate(PETSC_COMM_WORLD, &b); CHKERRQ(ierr);
    ierr = VecSetSizes(b, PETSC_DECIDE, N); CHKERRQ(ierr);
    ierr = VecSetFromOptions(b); CHKERRQ(ierr);
    
    ierr = VecDuplicate(b, &x); CHKERRQ(ierr);
    
    // Assemble system by looping over elements
    PetscPrintf(PETSC_COMM_WORLD, "Assembling system...\n");
    

    MatSetValue(A, 0, 0, 2, INSERT_VALUES);
    MatSetValue(A, 1, 0, -1, INSERT_VALUES);
    MatSetValue(A, N-1, N-1, 2, INSERT_VALUES);
    MatSetValue(A, N-2, N-1, -1, INSERT_VALUES);
    VecSetValue(b, 0, 0, INSERT_VALUES);
    VecSetValue(b, N-1, N-1, INSERT_VALUES);
    for (int i = 1; i < N-1; i++)
    {
        double row[3] = {-1, 2, -1};
        const int nodes1[3] = {i-1, i, i+1};
        double rhs = i;
        MatSetValues(A, 3, nodes1, 1, &i, row, INSERT_VALUES);
        VecSetValue(b, i, rhs, INSERT_VALUES);
    }

    MatAssemblyBegin(A, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(A, MAT_FINAL_ASSEMBLY);
    VecAssemblyBegin(b);
    VecAssemblyEnd(b);

    // Create and setup linear solver
    ierr = KSPCreate(PETSC_COMM_WORLD, &ksp); CHKERRQ(ierr);
    ierr = KSPSetOperators(ksp, A, A); CHKERRQ(ierr);
    ierr = KSPSetFromOptions(ksp); CHKERRQ(ierr);
    
    // Solve the system
    PetscPrintf(PETSC_COMM_WORLD, "Solving linear system...\n");
    ierr = KSPSolve(ksp, b, x); CHKERRQ(ierr);

     // Check convergence
    ierr = KSPGetIterationNumber(ksp, &its); CHKERRQ(ierr);
    ierr = KSPGetResidualNorm(ksp, &norm); CHKERRQ(ierr);
    PetscPrintf(PETSC_COMM_WORLD, "Solution converged in %D iterations, "
                "residual norm: %g\n", its, (double)norm);

    VecView(x, PETSC_VIEWER_STDOUT_WORLD);
}