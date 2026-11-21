#include "robot_driver/controllers/cpg_command_guard.hpp"
#include <cassert>
#include <iostream>
#include <random>
#include <limits>
int main(){
 std::mt19937 gen(9411);std::uniform_real_distribution<double> pos(-3,3),vel(-35,35),gain(0,60),kd(0,5),ff(-40,40);
 int limited=0;
 for(int i=0;i<100000;++i){
  cpg_guard::Motor m{pos(gen),vel(gen),gain(gen),kd(gen),ff(gen)};
  double q=pos(gen),dq=vel(gen),raw=cpg_guard::effort(m,q,dq);
  double factor=cpg_guard::limit(m,q,dq),after=cpg_guard::effort(m,q,dq);
  double bound=std::min(18.,(raw*dq>=0?20.2:23.4)*std::clamp((30-std::abs(dq))/16.5,0.,1.));
  assert(std::abs(after)<=bound+1e-10);assert(std::abs(after-raw*factor)<1e-10);
  assert(factor>=0&&factor<=1);limited+=factor<1;
 }
 cpg_guard::Motor bad{0,0,25,.5,0};bool threw=false;
 try {cpg_guard::limit(bad,std::numeric_limits<double>::quiet_NaN(),0);}catch(...){threw=true;}assert(threw);
 assert(!cpg_guard::targetRange(0,2));assert(cpg_guard::targetRange(-1.5,2));
 std::cout<<"100000 sample-based PD+FF envelope checks passed; limited="<<limited<<"\n";
}
