#pragma once
// Experimental CPU wrapper around unmodified official AMGCL 1.4.9.
// Compile without -fopenmp: the builtin backend then executes serial loops.
#ifdef _OPENMP
#error "This offline AMGCL comparison must be compiled without OpenMP"
#endif
#include <amgcl/adapter/crs_tuple.hpp>
#include <amgcl/amg.hpp>
#include <amgcl/backend/builtin.hpp>
#include <amgcl/coarsening/smoothed_aggregation.hpp>
#include <amgcl/make_solver.hpp>
#include <amgcl/relaxation/spai0.hpp>
#include <amgcl/solver/cg.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ego4p0 {
class AmgBackend {
 public:
  using Backend=amgcl::backend::builtin<double>;
  using Solver=amgcl::make_solver<
      amgcl::amg<Backend,amgcl::coarsening::smoothed_aggregation,amgcl::relaxation::spai0>,
      amgcl::solver::cg<Backend>>;
  // This adapter accepts the project's Dirichlet-eliminated finite-volume
  // Stieltjes matrix. It checks sufficient SPD conditions, not arbitrary SPD
  // matrices with positive off-diagonals. No matrix regularization is applied.
  bool setup(const std::vector<int>&ptr,const std::vector<int>&col,const std::vector<double>&val) {
    const auto begin=now_ns();clear();
    try {
      if(ptr.size()<2||ptr.front()!=0||col.size()!=val.size()||ptr.back()<0||size_t(ptr.back())!=val.size())throw std::invalid_argument("invalid CSR dimensions");
      const size_t n=ptr.size()-1;
      diagonal_.resize(n);std::vector<double> deficit(n),scale(n);std::vector<uint8_t> visited(n);std::vector<int> queue(n);
      const double roundoff=64*std::numeric_limits<double>::epsilon();
      for(size_t i=0;i<n;++i) {
        if(ptr[i]<0||ptr[i]>ptr[i+1]||size_t(ptr[i+1])>val.size())throw std::invalid_argument("invalid CSR row offsets");
        bool has_diagonal=false;double off_diagonal=0.;int previous=-1;
        for(int e=ptr[i];e<ptr[i+1];++e) {
          const int j=col[e];const double value=val[e];
          if(j<0||size_t(j)>=n||j<=previous||!std::isfinite(value))throw std::invalid_argument("CSR must be finite, sorted, and duplicate-free");
          previous=j;
          if(size_t(j)==i){diagonal_[i]=value;has_diagonal=true;}
          else {if(value>0)throw std::invalid_argument("finite-volume matrix has positive off-diagonal");off_diagonal-=value;}
        }
        if(!has_diagonal||!(diagonal_[i]>0))throw std::invalid_argument("matrix has missing/nonpositive diagonal");
        scale[i]=std::max(diagonal_[i],off_diagonal);deficit[i]=diagonal_[i]-off_diagonal;
        if(deficit[i]<-roundoff*scale[i])throw std::invalid_argument("finite-volume matrix is not weakly diagonally dominant");
      }
      for(size_t i=0;i<n;++i)for(int e=ptr[i];e<ptr[i+1];++e) {
        const int j=col[e];if(size_t(j)==i)continue;
        const auto begin_row=col.begin()+ptr[j],end_row=col.begin()+ptr[j+1];const auto reverse=std::lower_bound(begin_row,end_row,int(i));
        if(reverse==end_row||*reverse!=int(i)||std::abs(val[size_t(reverse-col.begin())]-val[e])>roundoff*std::max(std::abs(val[e]),std::abs(val[size_t(reverse-col.begin())])))throw std::invalid_argument("matrix is not symmetric");
      }
      for(size_t start=0;start<n;++start)if(!visited[start]) {
        size_t head=0,tail=1;queue[0]=int(start);visited[start]=1;bool anchored=false;
        while(head<tail) {
          const int i=queue[head++];anchored|=deficit[i]>roundoff*scale[i];
          for(int e=ptr[i];e<ptr[i+1];++e){const int j=col[e];if(j!=i&&val[e]<0&&!visited[j]){visited[j]=1;queue[tail++]=j;}}
        }
        if(!anchored)throw std::invalid_argument("unanchored/pure-Neumann component is not SPD");
      }
      ptr_=ptr;col_=col;val_=val;minimum_diagonal_=*std::min_element(diagonal_.begin(),diagonal_.end());
      Solver::params prm;
      prm.precond.coarse_enough=64;prm.precond.direct_coarse=true;
      prm.precond.npre=1;prm.precond.npost=1;prm.precond.ncycle=1;prm.precond.pre_cycles=1;
      prm.precond.allow_rebuild=false;
      prm.precond.coarsening.aggr.eps_strong=.08f;prm.precond.coarsening.aggr.block_size=1;
      prm.precond.coarsening.relax=1.f;prm.precond.coarsening.estimate_spectral_radius=false;prm.precond.coarsening.power_iters=0;
      prm.solver.tol=1e-11;prm.solver.maxiter=1600;prm.solver.ns_search=false;prm.solver.verbose=false;
      solver_=std::make_unique<Solver>(std::tie(n,ptr_,col_,val_),prm);
      setup_ns_=now_ns()-begin;return true;
    }catch(const std::exception &e){solver_.reset();error_=e.what();setup_ns_=now_ns()-begin;return false;}
  }
  bool solve(const std::vector<double>&rhs,std::vector<double>&x,double requested_diag_tolerance=1e-7) {
    const auto begin=now_ns();iterations_=0;reported_residual_=true_diag_residual_=std::numeric_limits<double>::infinity();solve_ns_=residual_ns_=0;error_.clear();
    try {
      if(!solver_||rhs.size()!=diagonal_.size()||x.size()!=rhs.size()||!std::isfinite(requested_diag_tolerance)||requested_diag_tolerance<=0)throw std::invalid_argument("invalid AMG solve input or no prepared hierarchy");
      double rhs_norm=0.;for(size_t i=0;i<rhs.size();++i){if(!std::isfinite(rhs[i])||!std::isfinite(x[i]))throw std::invalid_argument("nonfinite RHS or initial guess");rhs_norm=std::hypot(rhs_norm,rhs[i]);}
      // ||r||_2 <= eps*min(diag) is sufficient for max_i |r_i|/diag_i <= eps.
      // Keep margin for recursive-vs-true residual roundoff; true residual below
      // is the final acceptance condition. This stricter l2 solve may oversolve.
      const double absolute_target=.25*requested_diag_tolerance*minimum_diagonal_;
      solver_->solver().prm.tol=rhs_norm>0?std::min(1e-11,absolute_target/rhs_norm):1e-11;
      solver_->solver().prm.abstol=std::min(absolute_target,std::numeric_limits<double>::min());
      size_t iterations=0;std::tie(iterations,reported_residual_)=(*solver_)(rhs,x);iterations_=uint32_t(iterations);
      const auto residual_begin=now_ns();true_diag_residual_=0.;
      for(size_t i=0;i<rhs.size();++i){double ax=0.;for(int e=ptr_[i];e<ptr_[i+1];++e)ax+=val_[e]*x[col_[e]];const double value=std::abs(rhs[i]-ax)/diagonal_[i];if(!std::isfinite(value)||!std::isfinite(x[i]))throw std::runtime_error("nonfinite AMG result");true_diag_residual_=std::max(true_diag_residual_,value);}
      residual_ns_=now_ns()-residual_begin;solve_ns_=now_ns()-begin;
      if(!std::isfinite(reported_residual_)||true_diag_residual_>requested_diag_tolerance){error_="AMG true diagonal-normalized residual did not meet requested tolerance";return false;}
      return true;
    }catch(const std::exception &e){error_=e.what();solve_ns_=now_ns()-begin;return false;}
  }
  void clear(){solver_.reset();ptr_.clear();col_.clear();val_.clear();diagonal_.clear();minimum_diagonal_=0.;iterations_=0;setup_ns_=solve_ns_=residual_ns_=0;reported_residual_=true_diag_residual_=std::numeric_limits<double>::infinity();error_.clear();}
  uint32_t iterations()const{return iterations_;}
  double reported_residual()const{return reported_residual_;}
  double true_diag_residual()const{return true_diag_residual_;}
  int64_t setup_ns()const{return setup_ns_;}
  int64_t solve_ns()const{return solve_ns_;}
  int64_t residual_ns()const{return residual_ns_;}
  size_t bytes()const{return solver_?solver_->bytes():0;}
  std::string hierarchy_description()const{if(!solver_)return {};std::ostringstream out;out<<solver_->precond();return out.str();}
  const std::string &error()const{return error_;}
 private:
  static int64_t now_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  std::unique_ptr<Solver> solver_;
  std::vector<int> ptr_,col_;std::vector<double> val_,diagonal_;
  double minimum_diagonal_=0.,reported_residual_=std::numeric_limits<double>::infinity(),true_diag_residual_=std::numeric_limits<double>::infinity();
  uint32_t iterations_=0;int64_t setup_ns_=0,solve_ns_=0,residual_ns_=0;std::string error_;
};
}
