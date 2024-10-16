
#include "../include/somigliana/green_core.h"
#include "../include/somigliana/green_core.inl"

#include <Eigen/Dense>

__global__ void mvc_kernel(scalar_t *d_PHI,
                           const scalar_t *d_V,
                           const index_t  *d_cageF,
                           const scalar_t *d_cageV,
                           const index_t nv,
                           const index_t ncf,
                           const index_t ncv) {
  unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  mvc_kernel(d_PHI,
    d_V,
    d_cageF,
    d_cageV,
    nv,
    ncf,
    ncv,
    index);
}

__global__ void green_kernel(scalar_t *d_phix,
                             scalar_t *d_phiy,
                             scalar_t *d_phiz,
                             scalar_t *d_psi,
                             const scalar_t *d_V,
                             const index_t  *d_cageF,
                             const scalar_t *d_cageV,
                             const scalar_t *d_cageN,
                             const index_t nv,
                             const index_t ncf,
                             const index_t ncv,
                             const scalar_t *d_qp,
                             const scalar_t *d_qw,
                             const index_t nq) {
  unsigned int thread_index = blockIdx.x*blockDim.x + threadIdx.x;
  unsigned int index = thread_index/ncf, f = thread_index%ncf;

  green_kernel(d_phix,
    d_phiy,
    d_phiz,
    d_psi,
    d_V,
    d_cageF,
    d_cageV,
    d_cageN,
    nv,
    ncf,
    ncv,
    d_qp,
    d_qw,
    nq,
    index,
    f);
}

__global__ void green_kernel_post(scalar_t *d_phi,
                                  const scalar_t *d_phix,
                                  const scalar_t *d_phiy,
                                  const scalar_t *d_phiz,
                                  const index_t *d_cageF,
                                  const index_t nv,
                                  const index_t ncf,
                                  const index_t ncv) {
  unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  green_kernel_post(d_phi,
    d_phix,
    d_phiy,
    d_phiz,
    d_cageF,
    nv,
    ncf,
    ncv,
    index);
}

__global__ void somig_kernel(const scalar_t nu,
                             scalar_t *d_PHIx,
                             scalar_t *d_PHIy,
                             scalar_t *d_PHIz,
                             scalar_t *d_PSI ,
                             const scalar_t *d_V,
                             const index_t  *d_cageF,
                             const scalar_t *d_cageV,
                             const scalar_t *d_cageN,
                             const index_t nv,
                             const index_t ncf,
                             const index_t ncv,
                             const scalar_t *d_qp,
                             const scalar_t *d_qw,
                             const index_t nq)  {
  unsigned int thread_index = blockIdx.x*blockDim.x + threadIdx.x;
  unsigned int index = thread_index/ncf,
                f = thread_index%ncf;

  somig_kernel(nu,
    d_PHIx,
    d_PHIy,
    d_PHIz,
    d_PSI,
    d_V,
    d_cageF,
    d_cageV,
    d_cageN,
    nv,
    ncf,
    ncv,
    d_qp,
    d_qw,
    nq,
    index,
    f);
  }

__global__ void somig_kernel_post(scalar_t *d_PHI,
                                  const scalar_t *d_PHIx,
                                  const scalar_t *d_PHIy,
                                  const scalar_t *d_PHIz,
                                  const index_t *d_cageF,
                                  const index_t nv,
                                  const index_t ncf) {
  unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  somig_kernel_post(d_PHI,
    d_PHIx,
    d_PHIy,
    d_PHIz,
    d_cageF,
    nv,
    ncf,
    index);
}

extern "C" {

  void mvc_gpu(scalar_t *d_PHI,
               const scalar_t *d_V,
               const index_t  *d_cageF,
               const scalar_t *d_cageV,
               const index_t nv,
               const index_t ncf,
               const index_t ncv) {
    const unsigned int blocksize = 256;
    const unsigned int numBlocks = (nv+blocksize-1)/blocksize;
    mvc_kernel<<< numBlocks, blocksize >>>
        (d_PHI, d_V, d_cageF, d_cageV, nv, ncf, ncv);
  }

  void green_gpu(scalar_t *d_phix,
                 scalar_t *d_phiy,
                 scalar_t *d_phiz,
                 scalar_t *d_psi,
                 const scalar_t *d_V,
                 const index_t  *d_cageF,
                 const scalar_t *d_cageV,
                 const scalar_t *d_cageN,
                 const index_t nv,
                 const index_t ncf,
                 const index_t ncv,
                 const scalar_t *d_qp,
                 const scalar_t *d_qw,
                 const index_t nq) {
    // parallel through basis columns
    const unsigned int blocksize = 256;
    const unsigned int numBlocks = (ncf*nv+blocksize-1)/blocksize;
    green_kernel<<< numBlocks, blocksize >>>
        (d_phix, d_phiy, d_phiz, d_psi,
         d_V, d_cageF, d_cageV, d_cageN, nv, ncf, ncv,
         d_qp, d_qw, nq);
  }

  void somig_gpu(const scalar_t nu,
                 scalar_t *d_PHIx,
                 scalar_t *d_PHIy,
                 scalar_t *d_PHIz,
                 scalar_t *d_PSI ,
                 const scalar_t *d_V,
                 const index_t  *d_cageF,
                 const scalar_t *d_cageV,
                 const scalar_t *d_cageN,
                 const index_t nv,
                 const index_t ncf,
                 const index_t ncv,
                 const scalar_t *d_qp,
                 const scalar_t *d_qw,
                 const index_t nq) {
    // parallel through basis entries
    const unsigned int blocksize = 256;
    const unsigned int numBlocks = (ncf*nv+blocksize-1)/blocksize;
    somig_kernel<<< numBlocks, blocksize >>>
        (nu,
         d_PHIx,
         d_PHIy,
         d_PHIz,
         d_PSI,
         d_V,
         d_cageF,
         d_cageV,
         d_cageN,
         nv,
         ncf,
         ncv,
         d_qp,
         d_qw,
         nq);
  }

  // reduce phixyz to phi
  void green_post_gpu(scalar_t *d_phi,
                      const scalar_t *d_phix,
                      const scalar_t *d_phiy,
                      const scalar_t *d_phiz,
                      const index_t  *d_cageF,
                      const index_t nv,
                      const index_t ncf,
                      const index_t ncv) {
    // parallel through basis columns
    const unsigned int blocksize = 256;
    const unsigned int numBlocks = (nv+blocksize-1)/blocksize;
    green_kernel_post<<< numBlocks, blocksize >>>
        (d_phi, d_phix, d_phiy, d_phiz, d_cageF, nv, ncf, ncv);
  }

  // reduce PHIxyz to PHI
  void somig_post_gpu(scalar_t *d_PHI,
                      const scalar_t *d_PHIx,
                      const scalar_t *d_PHIy,
                      const scalar_t *d_PHIz,
                      const index_t  *d_cageF,
                      const index_t nv,
                      const index_t ncf) {
    // parallel through basis columns
    const unsigned int blocksize = 256;
    const unsigned int numBlocks = (nv+blocksize-1)/blocksize;
    somig_kernel_post<<< numBlocks, blocksize >>>
        (d_PHI, d_PHIx, d_PHIy, d_PHIz, d_cageF, nv, ncf);
  }
  
}
