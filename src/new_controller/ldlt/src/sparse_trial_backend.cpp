#include "new_controller/sparse_trial_backend.hpp"
#include "new_controller/harmonic_preconditioner.hpp"
#include "new_controller/amg_backend.hpp"
#include <Eigen/SparseCholesky>
#include <cholmod.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>
namespace ego4p0 {
const char *sparse_trial_name(SparseTrialMode m){switch(m){case SparseTrialMode::EigenLDLT:return "eigen_ldlt";case SparseTrialMode::EigenLLT:return "eigen_llt";case SparseTrialMode::Cholmod:return "cholmod";case SparseTrialMode::Amg:return "amgcl";case SparseTrialMode::IC0:return "ic0";case SparseTrialMode::SGS:return "sgs";}return "invalid";}
struct SparseTrialBackend::Impl {
  using Matrix=Eigen::SparseMatrix<double,Eigen::ColMajor,int>;
  SparseTrialMode mode;SparseTrialStats stats;
  std::vector<int> ptr,col;std::vector<double>val,diag;
  Matrix matrix;Eigen::SimplicialLDLT<Matrix,Eigen::Lower,Eigen::AMDOrdering<int>> ldlt;
  Eigen::SimplicialLLT<Matrix,Eigen::Lower,Eigen::AMDOrdering<int>> llt;
  cholmod_common common{};cholmod_factor *factor=nullptr;
  std::vector<int> cp,ci;std::vector<double>cv;
  AmgBackend amg;AdaptiveCoarsePreconditioner preconditioner;AdaptiveCoarseField graph;
  std::vector<double>r,z,p,ap;bool valid=false,symbolic=false;int n=0;
  explicit Impl(SparseTrialMode m):mode(m){cholmod_start(&common);common.nmethods=1;common.method[0].ordering=CHOLMOD_AMD;common.postorder=1;common.supernodal=CHOLMOD_AUTO;common.print=0;}
  ~Impl(){if(factor)cholmod_free_factor(&factor,&common);cholmod_finish(&common);}
  void reset(){valid=symbolic=false;ptr.clear();col.clear();val.clear();if(factor)cholmod_free_factor(&factor,&common);amg.clear();preconditioner.invalidate();}
  bool fail(const std::string &s){stats.error=s;valid=false;stats.ok=false;return false;}
  bool validate(const std::vector<int>&rp,const std::vector<int>&cc,const std::vector<double>&vv,const std::vector<double>&b){
    n=static_cast<int>(b.size());if(n<=0||rp.size()!=size_t(n+1)||rp[0]||rp[n]!=int(cc.size())||cc.size()!=vv.size())return fail("invalid CSR shape");
    diag.assign(n,0);std::vector<uint8_t>seen(n);std::vector<int>queue;queue.reserve(n);std::vector<uint8_t>ground(n);
    for(int i=0;i<n;++i){if(rp[i]>rp[i+1]||rp[i]<0||rp[i+1]>int(cc.size())||!std::isfinite(b[i]))return fail("invalid row/RHS");double off=0.;int previous=-1;
      for(int e=rp[i];e<rp[i+1];++e){int j=cc[e];double w=vv[e];if(j<0||j>=n||j<=previous||!std::isfinite(w))return fail("CSR must be sorted unique finite");previous=j;
        if(j==i){if(!(w>0))return fail("nonpositive diagonal");diag[i]=w;}else{if(!(w<0))return fail("not an FV M-matrix");off-=w;}
      }
      if(!(diag[i]>0)||diag[i]+1e-11*std::max(1.,diag[i])<off)return fail("not diagonally dominant");
      ground[i]=diag[i]-off>1e-12*std::max(1.,diag[i]);
    }
    for(int i=0;i<n;++i)for(int e=rp[i];e<rp[i+1];++e){const int j=cc[e];if(i==j)continue;const auto it=std::lower_bound(cc.begin()+rp[j],cc.begin()+rp[j+1],i);if(it==cc.begin()+rp[j+1]||*it!=i||std::abs(vv[e]-vv[it-cc.begin()])>1e-12*std::max(1.,std::abs(vv[e])))return fail("asymmetric FV matrix");}
    for(int seed=0;seed<n;++seed)if(!seen[seed]){queue.clear();queue.push_back(seed);seen[seed]=1;bool anchored=false;for(size_t k=0;k<queue.size();++k){int i=queue[k];anchored|=ground[i];for(int e=rp[i];e<rp[i+1];++e){int j=cc[e];if(j!=i&&!seen[j]){seen[j]=1;queue.push_back(j);}}}if(!anchored)return fail("ungrounded component");}
    return true;
  }
  void import_matrix(const std::vector<int>&rp,const std::vector<int>&cc,const std::vector<double>&vv){
    if(mode==SparseTrialMode::EigenLDLT||mode==SparseTrialMode::EigenLLT) {
      Eigen::Map<const Eigen::SparseMatrix<double,Eigen::RowMajor,int>> mapped(n,n,int(vv.size()),rp.data(),cc.data(),vv.data());matrix=mapped;
    }
    if(mode!=SparseTrialMode::Cholmod)return;
    // CHOLMOD accepts symmetric CSC with a single stored triangle. Build a
    // lower CSC explicitly; no CSR-as-CSC reinterpretation or A*A^T.
    cp.assign(n+1,0);for(int i=0;i<n;++i)for(int e=rp[i];e<rp[i+1];++e)if(i>=cc[e])++cp[cc[e]+1];for(int i=0;i<n;++i)cp[i+1]+=cp[i];
    ci.resize(cp[n]);cv.resize(cp[n]);auto cursor=cp;for(int i=0;i<n;++i)for(int e=rp[i];e<rp[i+1];++e)if(i>=cc[e]){int k=cursor[cc[e]]++;ci[k]=i;cv[k]=vv[e];}
  }
  cholmod_sparse view(){cholmod_sparse a{};a.nrow=a.ncol=n;a.nzmax=cv.size();a.p=cp.data();a.i=ci.data();a.x=cv.data();a.stype=-1;a.itype=CHOLMOD_INT;a.xtype=CHOLMOD_REAL;a.dtype=CHOLMOD_DOUBLE;a.sorted=a.packed=1;return a;}
  bool setup_pc(const std::vector<int>&rp,const std::vector<int>&cc,const std::vector<double>&vv){
    graph.allocate(n);graph.neighbor.resize(std::max(graph.neighbor.size(),vv.size()));graph.weight.resize(std::max(graph.weight.size(),vv.size()));graph.nodes=n;graph.edges=0;graph.offset[0]=0;
    for(int i=0;i<n;++i){graph.boundary[i]=uint8_t(FieldBoundary::Interior);for(int e=rp[i];e<rp[i+1];++e)if(cc[e]!=i){graph.neighbor[graph.edges]=cc[e];graph.weight[graph.edges++]=-vv[e];}graph.offset[i+1]=graph.edges;}
    preconditioner.configure(n,static_cast<uint32_t>(vv.size()));preconditioner.set_mode(mode==SparseTrialMode::IC0?CoarsePreconditioner::IC0:CoarsePreconditioner::SymmetricGaussSeidel);
    r.resize(n);z.resize(n);p.resize(n);ap.resize(n);return preconditioner.prepare(graph,diag);
  }
  bool pcg(const std::vector<double>&b,std::vector<double>&x,double tolerance){
    auto multiply=[&](const std::vector<double>&a,std::vector<double>&out){for(int i=0;i<n;++i){double sum=0.;for(int e=ptr[i];e<ptr[i+1];++e)sum+=val[e]*a[col[e]];out[i]=sum;}};
    auto dot=[&](const auto&a,const auto&bb){return std::inner_product(a.begin(),a.end(),bb.begin(),0.);};
    multiply(x,ap);double residual=0.;for(int i=0;i<n;++i){r[i]=b[i]-ap[i];residual=std::max(residual,std::abs(r[i])/diag[i]);}if(residual<=tolerance)return true;
    if(!preconditioner.apply(r,z))return fail("preconditioner apply failed");
    p=z;double rz=dot(r,z);
    for(uint32_t it=0;it<1600;++it){multiply(p,ap);const double denom=dot(p,ap);if(!(denom>0)||!(rz>0)||!std::isfinite(denom)||!std::isfinite(rz))return fail("PCG breakdown");double alpha=rz/denom;residual=0.;for(int i=0;i<n;++i){x[i]+=alpha*p[i];r[i]-=alpha*ap[i];residual=std::max(residual,std::abs(r[i])/diag[i]);}stats.iterations=it+1;if(residual<=tolerance)return true;if(!preconditioner.apply(r,z))return fail("preconditioner apply failed");double next=dot(r,z);if(!(next>0)||!std::isfinite(next))return fail("PCG breakdown");double beta=next/rz;rz=next;for(int i=0;i<n;++i)p[i]=z[i]+beta*p[i];}return fail("PCG iteration cap");
  }
  bool solve(const std::vector<int>&rp,const std::vector<int>&cc,const std::vector<double>&vv,const std::vector<double>&b,std::vector<double>&x,double tolerance){
    stats={};const auto begin=field_now_ns();auto done=[&](bool ok){stats.ok=ok;stats.total_ns=field_now_ns()-begin;return ok;};
    if(!(tolerance>0)||!std::isfinite(tolerance))return done(fail("invalid tolerance"));
    const auto imported=field_now_ns();
    const bool validated=validate(rp,cc,vv,b);
    stats.validation_ns=field_now_ns()-imported;
    if(!validated)return done(false);
    if(x.size()!=b.size())x.assign(b.size(),.5);
    const bool same_pattern=symbolic&&rp==ptr&&cc==col;const bool same_numeric=valid&&same_pattern&&vv==val;stats.symbolic_reused=same_pattern&&(same_numeric||mode==SparseTrialMode::EigenLDLT||mode==SparseTrialMode::EigenLLT||mode==SparseTrialMode::Cholmod);stats.numeric_reused=same_numeric;stats.unknowns=n;stats.nonzeros=vv.size();
    const auto converted=field_now_ns();
    if(!same_numeric)import_matrix(rp,cc,vv);
    stats.conversion_ns=field_now_ns()-converted;
    stats.import_ns=field_now_ns()-imported;
    if(!same_pattern){const auto a=field_now_ns();switch(mode){case SparseTrialMode::EigenLDLT:ldlt.analyzePattern(matrix);break;case SparseTrialMode::EigenLLT:llt.analyzePattern(matrix);break;case SparseTrialMode::Cholmod:{if(factor)cholmod_free_factor(&factor,&common);common.status=CHOLMOD_OK;auto view_a=view();factor=cholmod_analyze(&view_a,&common);if(!factor||common.status<0)return done(fail("CHOLMOD analysis failed"));break;}default:break;}stats.analysis_ns=field_now_ns()-a;}
    if(!same_numeric){const auto f=field_now_ns();bool ok=true;
      switch(mode){case SparseTrialMode::EigenLDLT:ldlt.factorize(matrix);ok=ldlt.info()==Eigen::Success&&(ldlt.vectorD().array()>0).all();break;
        case SparseTrialMode::EigenLLT:llt.factorize(matrix);ok=llt.info()==Eigen::Success;break;
        case SparseTrialMode::Cholmod:{common.status=CHOLMOD_OK;auto view_a=view();ok=cholmod_factorize(&view_a,factor,&common)&&common.status==CHOLMOD_OK&&factor->minor==size_t(n);break;}
        case SparseTrialMode::Amg:ok=amg.setup(rp,cc,vv);break;
        case SparseTrialMode::IC0:case SparseTrialMode::SGS:ok=setup_pc(rp,cc,vv);break;
      }
      stats.factor_ns=field_now_ns()-f;if(!ok){symbolic=false;return done(fail("factor/setup failed"));}
    }
    ptr=rp;col=cc;val=vv;valid=symbolic=true;const auto solve_begin=field_now_ns();bool ok=true;
    if(mode==SparseTrialMode::EigenLDLT||mode==SparseTrialMode::EigenLLT){Eigen::Map<const Eigen::VectorXd>rhs(b.data(),n);Eigen::Map<Eigen::VectorXd>output(x.data(),n);if(mode==SparseTrialMode::EigenLDLT){output=ldlt.solve(rhs);ok=ldlt.info()==Eigen::Success;}else{output=llt.solve(rhs);ok=llt.info()==Eigen::Success;}}
    else if(mode==SparseTrialMode::Cholmod){cholmod_dense rhs{};rhs.nrow=n;rhs.ncol=1;rhs.nzmax=n;rhs.d=n;rhs.x=const_cast<double*>(b.data());rhs.xtype=CHOLMOD_REAL;rhs.dtype=CHOLMOD_DOUBLE;common.status=CHOLMOD_OK;auto output=cholmod_solve(CHOLMOD_A,factor,&rhs,&common);ok=output&&common.status==CHOLMOD_OK;if(output){std::copy_n(static_cast<double*>(output->x),n,x.begin());cholmod_free_dense(&output,&common);}}
    else if(mode==SparseTrialMode::Amg){ok=amg.solve(b,x,tolerance);stats.iterations=amg.iterations();}
    else ok=pcg(b,x,tolerance);
    stats.solve_ns=field_now_ns()-solve_begin;if(!ok)return done(fail("linear solve failed"));
    const auto verify=field_now_ns();double residual=0.;for(int i=0;i<n;++i){double ax=0.;for(int e=rp[i];e<rp[i+1];++e)ax+=vv[e]*x[cc[e]];double v=std::abs(b[i]-ax)/diag[i];if(!std::isfinite(x[i])||!std::isfinite(v))return done(fail("nonfinite solution"));residual=std::max(residual,v);}stats.residual=residual;stats.verify_ns=field_now_ns()-verify;
    if(mode==SparseTrialMode::EigenLDLT)stats.factor_nonzeros=ldlt.matrixL().nestedExpression().nonZeros()+n;
    else if(mode==SparseTrialMode::EigenLLT)stats.factor_nonzeros=llt.matrixL().nestedExpression().nonZeros();
    else if(mode==SparseTrialMode::Cholmod)stats.factor_nonzeros=factor->is_super?factor->xsize:static_cast<uint64_t>(common.lnz);
    if(residual>tolerance)return done(fail("true diagonal residual above requested tolerance"));
    return done(true);
  }
};
SparseTrialBackend::SparseTrialBackend(SparseTrialMode m):impl_(new Impl(m)){}
SparseTrialBackend::~SparseTrialBackend()=default;
void SparseTrialBackend::reset(){impl_->reset();}
bool SparseTrialBackend::solve(const std::vector<int>&p,const std::vector<int>&c,const std::vector<double>&v,const std::vector<double>&b,std::vector<double>&x,double t){return impl_->solve(p,c,v,b,x,t);}
const SparseTrialStats &SparseTrialBackend::stats()const{return impl_->stats;}
}
