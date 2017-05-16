#include <iostream>
#include <vector>
#include <stdlib.h>
using namespace std;

std::vector<float> box;

bool collision_box(std::vector<float> spot)
{
  box.clear();
  box.push_back(1.9);
  box.push_back(4.1);
  box.push_back(-1.1);
  box.push_back(1.1);
  if(spot[0]<box[0]||spot[0]>box[1])
  {
    return false;
  }
  else if(spot[1]<box[2]||spot[1]>box[3])
  {
        return false;
  }
  return true;
}

bool collision_narrow(std::vector<float> spot)
{
    box.clear();
    box.push_back(-11.035);
    box.push_back(4.115);
    box.push_back(2.65);
    box.push_back(4.85);
    box.push_back(5.1);
    box.push_back(8.97);
    box.push_back(2.665);
    box.push_back(4.795);
    if((spot[0]>box[0] && spot[0]<box[1])&&(spot[1]>box[2] && spot[1]<box[3]))
        return true;
    if((spot[0]>box[4] && spot[0]<box[5])&&(spot[1]>box[6] && spot[1]<box[7]))
        return true;
    return false;
}

// int main()
// {
//     std::vector<float> a;
//     a.push_back(2.36);
//     a.push_back(2.85);
//     cout<<collision_narrow(a);
//     return 0;
// }
