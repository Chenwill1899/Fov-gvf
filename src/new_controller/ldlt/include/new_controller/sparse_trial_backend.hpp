#pragma once
#include <vector>
#include <string>
#include <memory>
#include <cstdint>
namespace ego4p0 {
enum class SparseTrialMode {EigenLDLT,EigenLLT,Cholmod,Amg,IC0,SGS};
const char *sparse_trial_name(SparseTrialMode mode);
struct SparseTrialStats {
  int64_t validation_ns=0,conversion_ns=0,import_ns=0,analysis_ns=0,factor_ns=0,solve_ns=0,verify_ns=0,total_ns=0;
  uint32_t iterations=0,unknowns=0,nonzeros=0;uint64_t factor_nonzeros=0;
  bool symbolic_reused=false,numeric_reused=false,ok=false;
  double residual=0;std::string error;
};
// Matrix is the grounded Interior principal FV matrix, full symmetric CSR.
// RHS is deliberately excluded from factor identity. Numerical and symbolic
// cache keys are exact comparisons, not an unchecked hash or node count.
class SparseTrialBackend {
 public:
  explicit SparseTrialBackend(SparseTrialMode mode=SparseTrialMode::EigenLDLT);
  ~SparseTrialBackend();
  SparseTrialBackend(const SparseTrialBackend&)=delete;
  SparseTrialBackend &operator=(const SparseTrialBackend&)=delete;
  void reset();
  bool solve(const std::vector<int>&ptr,const std::vector<int>&col,
             const std::vector<double>&val,const std::vector<double>&rhs,
             std::vector<double>&x,double tolerance=1e-7);
  const SparseTrialStats &stats()const;
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
