// #include <iostream>
// #include <cstring>
struct LimbCmd_t {
  // float pos[3], Kp[3], Kd[3];
  // probs add tau
  float tau[3], pos[3];
};
LimbCmd_t limbCmd[2];
float copy_data[] = {0,0,0,0,0,0,0,0,0,0,0,0};
int main() {
  float data[] = {1,1,1,2,2,2,3,3,3,4,4,4};
  // std::memcpy(copy_data, data, 6*sizeof(float));
  // std::cout << sizeof(LimbCmd_t)<< "\n";
  // std::memset(limbCmd, 0, 24*4);
  std::memset(limbCmd, 0, 2*24);
  std::memcpy(limbCmd, data, 2*24);
    for (int i = 0; i < 12; i = i+1 ){
     std::cout << copy_data[i]<<"\n";
  }

  std::memcpy(copy_data,limbCmd,2*2*24);
      for (int i = 0; i < 12; i = i+1 ){
     std::cout << copy_data[i]<<"\n";
  }
  // std::memcpy(limbCmd, data, sizeof(LimbCmd_t));
  // for (int i = 0; i < 3; i = i+1 ){
  //    std::cout << limbCmd[0].tau[i]<<"\n";
  //   std::cout << limbCmd[0].pos[i]<<"\n";
  // }
  //   for (int i = 0; i < 3; i = i+1 ){
  //    std::cout << limbCmd[1].tau[i]<<"\n";
  //   std::cout << limbCmd[1].pos[i]<<"\n";
  // }
  // Structs get stored in this way struct[0].0 then struct[0].1 then struct[0].2 struct[1].0 then struct[1].1 then struct[1].2
  // for (int i = 0; i < 6; i = i+1 ){
  //   std::cout << copy_data[i]<<"\n";
  // }
  // std::cout << "Hello World!\n";
  return 0;
}