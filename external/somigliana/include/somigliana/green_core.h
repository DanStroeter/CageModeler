#ifndef GREEN_CORE_H
#define GREEN_CORE_H

#ifdef SOMIG_WITH_CUDA
#include <cuda_runtime.h>
#include "helper_cuda.h"
#endif // WITH_CUDA

#include <iostream>
#include <cstring>

typedef float scalar_t;
typedef int   index_t;

extern "C" {
  
#ifdef SOMIG_WITH_CUDA
  void mvc_gpu(scalar_t *d_PHI,
               const scalar_t *d_V,
               const index_t  *d_cageF,
               const scalar_t *d_cageV,               
               const index_t nv,
               const index_t ncf,
               const index_t ncv);
#endif // SOMIG_WITH_CUDA

  void mvc_cpu(scalar_t *h_PHI,
               const scalar_t *h_V,
               const index_t  *h_cageF,
               const scalar_t *h_cageV,               
               const index_t nv,
               const index_t ncf,
               const index_t ncv);

#ifdef SOMIG_WITH_CUDA
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
                 const index_t nq);
#endif // SOMIG_WITH_CUDA

  void green_cpu(scalar_t *h_phix,
                 scalar_t *h_phiy,
                 scalar_t *h_phiz,
                 scalar_t *h_psi,
                 const scalar_t *h_V,
                 const index_t  *h_cageF,
                 const scalar_t *h_cageV,
                 const scalar_t *h_cageN,
                 const index_t nv,
                 const index_t ncf,
                 const index_t ncv,
                 const scalar_t *h_qp,
                 const scalar_t *h_qw,
                 const index_t nq);

#ifdef SOMIG_WITH_CUDA
  void green_post_gpu(scalar_t *d_phi,
                      const scalar_t *d_phix,
                      const scalar_t *d_phiy,
                      const scalar_t *d_phiz,
                      const index_t  *d_cageF,
                      const index_t nv,
                      const index_t ncf,
                      const index_t ncv);  
#endif // SOMIG_WITH_CUDA

  void green_post_cpu(scalar_t *h_phi,
                      const scalar_t *h_phix,
                      const scalar_t *h_phiy,
                      const scalar_t *h_phiz,
                      const index_t  *h_cageF,
                      const index_t nv,
                      const index_t ncf,
                      const index_t ncv);  

#ifdef SOMIG_WITH_CUDA
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
                 const index_t nq);
#endif // SOMIG_WITH_CUDA

  void somig_cpu(const scalar_t nu,
                 scalar_t *h_PHIx,
                 scalar_t *h_PHIy,
                 scalar_t *h_PHIz,
                 scalar_t *h_PSI ,
                 const scalar_t *h_V,
                 const index_t  *h_cageF,
                 const scalar_t *h_cageV,
                 const scalar_t *h_cageN,
                 const index_t nv,
                 const index_t ncf,
                 const index_t ncv,
                 const scalar_t *h_qp,
                 const scalar_t *h_qw,
                 const index_t nq);

#ifdef SOMIG_WITH_CUDA
  // reduce PHIxyz to PHI
  void somig_post_gpu(scalar_t *d_PHI,
                      const scalar_t *d_PHIx,
                      const scalar_t *d_PHIy,
                      const scalar_t *d_PHIz,
                      const index_t  *d_cageF,
                      const index_t nv,
                      const index_t ncf);
#endif // SOMIG_WITH_CUDA

  void somig_post_cpu(scalar_t *h_PHI,
                      const scalar_t *h_PHIx,
                      const scalar_t *h_PHIy,
                      const scalar_t *h_PHIz,
                      const index_t  *h_cageF,
                      const index_t nv,
                      const index_t ncf);

void mvc_compute(scalar_t *_PHI,
               const scalar_t *_V,
               const index_t  *_cageF,
               const scalar_t *_cageV,               
               const index_t nv,
               const index_t ncf,
               const index_t ncv);
               
void green_compute(scalar_t *_phix,
                scalar_t *_phiy,
                scalar_t *_phiz,
                scalar_t *_psi,
                const scalar_t *_V,
                const index_t  *_cageF,
                const scalar_t *_cageV,
                const scalar_t *_cageN,
                const index_t nv,
                const index_t ncf,
                const index_t ncv,
                const scalar_t *_qp,
                const scalar_t *_qw,
                const index_t nq);

void green_post(scalar_t *d_phi,
                    const scalar_t *d_phix,
                    const scalar_t *d_phiy,
                    const scalar_t *d_phiz,
                    const index_t  *d_cageF,
                    const index_t nv,
                    const index_t ncf,
                    const index_t ncv);

void somig_compute(const scalar_t nu,
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
                const index_t nq
                );

void somig_post(scalar_t *d_PHI,
                const scalar_t *d_PHIx,
                const scalar_t *d_PHIy,
                const scalar_t *d_PHIz,
                const index_t  *d_cageF,
                const index_t nv,
                const index_t ncf);
}

class cage_precomputer
{
 public:
 ~cage_precomputer() {
#ifdef SOMIG_WITH_CUDA
  // Clean local device variables
   
  // mesh and points
  cudaFree(d_cageF_);
  cudaFree(d_cageV_);
  cudaFree(d_cageN_);
  cudaFree(d_V_);

  // quadratures
  cudaFree(d_qp_);
  cudaFree(d_qw_);

  // somigliana
  cudaFree(d_PHIx_); 
  cudaFree(d_PHIy_); 
  cudaFree(d_PHIz_); 
  cudaFree(d_PSI_);
  cudaFree(d_PHI_);
#endif // SOMIG_WITH_CUDA

  // Clean local host variables
  // mesh and points
  delete[] h_cageF_;
  delete[] h_cageV_;
  delete[] h_cageN_;
  delete[] h_V_;

  // quadratures
  delete[] h_qp_;
  delete[] h_qw_;

  // somigliana
  delete[] h_PHIx_; 
  delete[] h_PHIy_; 
  delete[] h_PHIz_; 
  delete[] h_PSI_;
  delete[] h_PHI_;
  }  
  
  cage_precomputer(const index_t ncf,
                        const index_t ncv,
                        const index_t nv,
                        const index_t  *h_cageF,
                        const scalar_t *h_cageV,
                        const scalar_t *h_cageN,
                        const scalar_t *h_V)
                        : ncf_(ncf), ncv_(ncv), nv_(nv)
      {
#ifdef SOMIG_WITH_CUDA
    devID = findCudaDevice(0, NULL);

    //if (devID < 0) {
    //  spdlog::error("No CUDA Capable devices found");
    //  exit(EXIT_SUCCESS);
    //} else {
    //  spdlog::info("device ID={}", devID);
    //}

    // Use GPU if a valid ID is found
    if (devID >= 0) {

      // cage and V
      cudaMalloc((void**)&d_cageF_, 3*ncf*sizeof(index_t));
      cudaMalloc((void**)&d_cageV_, 3*ncv*sizeof(scalar_t));
      cudaMalloc((void**)&d_cageN_, 3*ncf*sizeof(scalar_t));
      cudaMalloc((void**)&d_V_,     3*nv *sizeof(scalar_t));

      // copy mesh and V
      cudaMemcpy(d_cageF_, h_cageF, 3*ncf_*sizeof(index_t),  cudaMemcpyHostToDevice);    
      cudaMemcpy(d_cageV_, h_cageV, 3*ncv_*sizeof(scalar_t), cudaMemcpyHostToDevice);
      cudaMemcpy(d_cageN_, h_cageN, 3*ncf_*sizeof(scalar_t), cudaMemcpyHostToDevice);
      cudaMemcpy(d_V_,     h_V,     3*nv_ *sizeof(scalar_t), cudaMemcpyHostToDevice);    

      // basis: somigliana
      cudaMalloc((void**)&d_PHIx_, 9*ncf*nv*sizeof(scalar_t)); 
      cudaMalloc((void**)&d_PHIy_, 9*ncf*nv*sizeof(scalar_t)); 
      cudaMalloc((void**)&d_PHIz_, 9*ncf*nv*sizeof(scalar_t)); 
      cudaMalloc((void**)&d_PSI_,  9*ncf*nv*sizeof(scalar_t));
      cudaMalloc((void**)&d_PHI_,  9*ncv*nv*sizeof(scalar_t));
      return;
    }
#endif // SOMIG_WITH_CUDA
    
    // else: Execute on CPU
    // cage and V
    h_cageF_ = new index_t[3*ncf_];
    h_cageV_ = new scalar_t[3*ncv_];
    h_cageN_ = new scalar_t[3*ncf_];
    h_V_ = new scalar_t[3*nv_];

    // copy mesh and V
    std::memcpy(h_cageF_, h_cageF, 3*ncf_*sizeof(index_t));    
    std::memcpy(h_cageV_, h_cageV, 3*ncv_*sizeof(scalar_t));
    std::memcpy(h_cageN_, h_cageN, 3*ncf_*sizeof(scalar_t));
    std::memcpy(h_V_,     h_V,     3*nv_ *sizeof(scalar_t));    

    // basis: somigliana
    h_PHIx_ = new scalar_t[9*ncf_*nv_];
    h_PHIy_ = new scalar_t[9*ncf_*nv_];
    h_PHIz_ = new scalar_t[9*ncf_*nv_];

    h_PSI_ = new scalar_t[9*ncf_*nv_];
    h_PHI_ = new scalar_t[9*ncf_*nv_];
  }

  void copy_quadrature_to_device(const index_t  num,
                                 const scalar_t *h_qp,
                                 const scalar_t *h_qw) {
    nq_ = num;
#ifdef SOMIG_WITH_CUDA
    if (devID >= 0) {
      cudaMalloc((void**)&d_qp_, 2*num*sizeof(scalar_t));
      cudaMalloc((void**)&d_qw_,   num*sizeof(scalar_t));
      
      cudaMemcpy(d_qp_, h_qp, 2*num*sizeof(scalar_t), cudaMemcpyHostToDevice);
      cudaMemcpy(d_qw_, h_qw,   num*sizeof(scalar_t), cudaMemcpyHostToDevice);
      return;
    }
#endif // SOMIG_WITH_CUDA

    // else: Execute on CPU
    h_qp_ = new scalar_t[2 * num];
    h_qw_ = new scalar_t[num];
    
    std::memcpy(h_qp_, h_qp, 2*num*sizeof(scalar_t));
    std::memcpy(h_qw_, h_qw,   num*sizeof(scalar_t));
    }

  void precompute_somig(const scalar_t nu,
                            scalar_t *h_PHI,
                            scalar_t *h_PSI) {
#ifdef SOMIG_WITH_CUDA
    if (devID >= 0) {
      cudaMemset(d_PHI_,  0, 9*ncv_*nv_*sizeof(scalar_t));
      cudaMemset(d_PSI_,  0, 9*ncf_*nv_*sizeof(scalar_t));

      // intermediate ones
      cudaMemset(d_PHIx_, 0, 9*ncf_*nv_*sizeof(scalar_t));
      cudaMemset(d_PHIy_, 0, 9*ncf_*nv_*sizeof(scalar_t));
      cudaMemset(d_PHIz_, 0, 9*ncf_*nv_*sizeof(scalar_t));
      somig_gpu(nu, d_PHIx_, d_PHIy_, d_PHIz_, d_PSI_, d_V_, d_cageF_, d_cageV_, d_cageN_, nv_, ncf_, ncv_, d_qp_, d_qw_, nq_);
      somig_post_gpu(d_PHI_, d_PHIx_, d_PHIy_, d_PHIz_, d_cageF_, nv_, ncf_);

      cudaMemcpy(h_PSI, d_PSI_, 9*ncf_*nv_*sizeof(scalar_t), cudaMemcpyDeviceToHost);
      cudaMemcpy(h_PHI, d_PHI_, 9*ncv_*nv_*sizeof(scalar_t), cudaMemcpyDeviceToHost);
      return;
    }
#endif // SOMIG_WITH_CUDA

    // else: Execute on CPU
    std::memset(h_PHI_,  0, 9*ncv_*nv_*sizeof(scalar_t));
    std::memset(h_PSI_,  0, 9*ncf_*nv_*sizeof(scalar_t));

    // intermediate ones
    std::memset(h_PHIx_, 0, 9*ncf_*nv_*sizeof(scalar_t));
    std::memset(h_PHIy_, 0, 9*ncf_*nv_*sizeof(scalar_t));
    std::memset(h_PHIz_, 0, 9*ncf_*nv_*sizeof(scalar_t));
    
    somig_cpu(nu, h_PHIx_, h_PHIy_, h_PHIz_, h_PSI_, h_V_, h_cageF_, h_cageV_, h_cageN_, nv_, ncf_, ncv_, h_qp_, h_qw_, nq_);
    somig_post_cpu(h_PHI_, h_PHIx_, h_PHIy_, h_PHIz_, h_cageF_, nv_, ncf_);

    std::memcpy(h_PSI, h_PSI_, 9*ncf_*nv_*sizeof(scalar_t));
    std::memcpy(h_PHI, h_PHI_, 9*ncv_*nv_*sizeof(scalar_t));

  }

 private:
  index_t devID = -1; // Only use Cuda if a valid device is found
  const index_t ncv_, nv_, ncf_;

  // cage
  scalar_t *h_cageV_, *h_cageN_;
  index_t  *h_cageF_;

  scalar_t *d_cageV_, *d_cageN_;
  index_t  *d_cageF_;

  // mesh points
  scalar_t *h_V_;
  scalar_t *d_V_;

  // basis SOMIGLIANA
  scalar_t *h_PHIx_, *h_PHIy_, *h_PHIz_;
  scalar_t *h_PSI_, *h_PHI_;

  scalar_t *d_PHIx_, *d_PHIy_, *d_PHIz_;
  scalar_t *d_PSI_, *d_PHI_;

  // quadratures points and weights
  index_t nq_;

  scalar_t *h_qp_, *h_qw_;
  scalar_t *d_qp_, *d_qw_;
};

#endif
