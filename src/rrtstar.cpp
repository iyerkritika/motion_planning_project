
#include<iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <algorithm>
#include "obstacle_one_box.cpp"

#define  PI 3.14;
const float goalb = 0.3;
const float step= 0.1 ;
const float hood=0.13;
const float step_cost=0.1;
using namespace std;

// calculating euclidean distance
float eucl(std::vector<float> a,std::vector<float> b)
{
  float eu=0;
  for (unsigned int i=0;i<a.size();i++)
    eu+=pow((a[i]-b[i]),2);
  return pow(eu,0.5);
}

//class definitions
class RRTNode
{
  std::vector<float> config;
  RRTNode *parent;
  float cost;

  public:
    // constructor with only conf
    RRTNode(std::vector<float> v,RRTNode* par=NULL,float c=0)
    {
      config=v;
      parent=par;
      cost=c;
    }

    // getting parent
    RRTNode* get_parent()
    {
      return parent;
    }

    void set_parent(RRTNode* par)
    {
        parent=par;
    }

    // getting conf
    std::vector<float> get_conf()
    {
      return config;
    }

    float get_cost()
    {
      return cost;
    }

    // operator overloading to compare 2 objects
    bool operator==(RRTNode obj)
    {
      std::vector<float> obje=obj.get_conf();
      for(unsigned int i=0;i<config.size();i++)
        if(config[i]!=obje[i])
          return false;
      return true;
    }

    // displaying a node
    void disp()
    {
      for(unsigned int i=0;i<config.size();i++)
        cout<<config[i]<<" ";
    }

    void clearing()
    {
      config.clear();
      parent=NULL;
      cost=0;
    }
};

//class of nodes
class NodeTree
{
  std::vector<RRTNode*> list;

  public:

    // constuctor to make a branch
    NodeTree(std::vector<RRTNode*> nodes)
    {
      list=nodes;
    }
    NodeTree()
    {
      list.clear();
    }
    // adding nodes to list
    void addnode(RRTNode* n)
    {
      list.push_back(n);
    }
    void number_nodes()
    {
      cout<<"the number of nodes are "<<list.size()<<"\n";
    }

    // nearest neighbor function
    RRTNode* near_neigh(std::vector<float> rc)
    {
      RRTNode *a;
      NodeTree neighbor;
      //defining first node as nearest to set bar to compare
      float dist,min=eucl(list[0]->get_conf(),rc);
      a=list[0];
      for (unsigned int i=1;i<list.size();i++)
      {
        dist=eucl(list[i]->get_conf(),rc);
        if(dist<min)
        {
        //setting nearer node as nearest
         min=dist;
         a=list[i];
        }
      }
      return a;
    }

    // displaying list from goal to start
    std::vector<std::vector<float> > get_path()
    {
      RRTNode* a;
      std::vector<std::vector<float> > path;
      if(list.empty())
      {
        cout<<"empty path";
        return path;
      }
      a=list[list.size()-1];
      while(a->get_parent()!=NULL)
      {
        path.push_back(a->get_conf());
        a=a->get_parent();
      }
      path.push_back(a->get_conf());
      return path;
    }

    RRTNode* parent_best(std::vector<float> node_config)
    {
      std::vector<RRTNode*> hoods;
      float least_cost;
      unsigned int i,index;
      for (i=0;i<list.size();i++)
        if (eucl(node_config,list[i]->get_conf())<=hood)
          hoods.push_back(list[i]);

      least_cost=hoods[0]->get_cost();
      index=0;
      for(i=1;i<hoods.size();i++)
      {
        if(hoods[i]->get_cost()<least_cost)
        {
          least_cost=hoods[i]->get_cost();
          index=i;
        }
      }
      return hoods[index];
    }
};

//finidng the direction of the vector pointing to new node and finding node withing a step
std::vector<float> direction(std::vector<float> nn, std::vector<float> rc,float s)
{
  std::vector<float> v;
  float x;
  x=s/eucl(rc,nn);
  for(unsigned int i=0;i<nn.size();i++)
    v.push_back(nn[i]+((rc[i]-nn[i])*x));
  return v;
}

// random sampling
std::vector<float> random(std::vector<float> low,std::vector<float> high)
{
  std::vector<float> pt;
  for(unsigned int i=0;i<low.size();i++)
    pt.push_back((((float)rand()/RAND_MAX)*(high[i]-low[i]))+low[i]);
  return pt;
}

//smoothing algorithm
// std::vector<std::vector<float> > smooth(std::vector<std::vector<float> > path,float step,OpenRAVE::EnvironmentBasePtr env,RobotBasePtr pr2)
// {
//   std::vector<float> node1,node2,dir;
//   std::vector<std::vector<float> > inter;
//   unsigned int index1,index2;
//   bool flag;
//   // float smooth_node_len=0,smooth_path_len=0;
//   for(int i=0;i<200;i++)
//   {
    //printing for each itration
    // smooth_path_len=0;
    // for(unsigned int j=0;j<(path.size()-1);j++)
    // {
    //   smooth_node_len=0;
    //   for(unsigned int k=0;k<=path[j].size();k++)
    //   {
    //     smooth_node_len+=pow((path[j][k]-path[j+1][k]),2);
    //   }
    //   smooth_path_len+=pow(smooth_node_len,0.5);
    // }
    // cout<<"path length of smoothened path after "<<i<<"iterations is "<<smooth_path_len<<"\n";
    //clearing before next itration
    // node1.clear();
    // node2.clear();
    // dir.clear();
    // inter.clear();
    // flag=false;
    // do
    // {
      // generating 2 random nodes
    //   index1=rand()%(path.size());
    //   index2=rand()%(path.size());
    // }
    // while(index1==index2);
    // // making sure lowest node is first
    // if(index2<index1)
    //   swap(index1,index2);
    // node1=path[index1];
    // inter.push_back(node1);
    // node2=path[index2];
    // dir=node1;
    // // path between two nodes shortest without collision
    // do
    // {
    //   if(eucl(dir,node2)<=step)
    //   {
    //     // pr2->SetActiveDOFValues(node2);
    //     // if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
    //     // {
    //     //   flag=false;
    //     //   break;
    //     // }
    //     // else
    //     // {
    //       inter.push_back(node2);
    //       flag=true;
    //       break;
    //     // }
    //   }
    //   else
    //   {
    //     dir=direction(dir,node2,step);
    //     // pr2->SetActiveDOFValues(dir);
    // //     // if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
    // //     // {
    //     //   flag=false;
    //     //   break;
    //     // }
    //     // else
    //       inter.push_back(dir);
    //       flag=true;
    //   }
//     // }
//     while(flag!=false);
//     // if no collision delete intermediate nodes inset new tree
//     if(flag!=false)
//     {
//       path.erase(path.begin()+index1,path.begin()+index2);
//       path.insert(path.begin()+index1,inter.begin(),inter.end());
//     }
//   }
//   return path;
// }
// RRT planner
std::vector<std::vector<float> > RRTpath(std::vector<float> start,std::vector<float>goal)
{
  // initializations
  cout<<"inside rrt planner\n";
  time_t time_s,time_e;
  time_s=time(NULL);
  RRTNode *node,*inter,*st,*par;
  st=new RRTNode(start,NULL,0);
  NodeTree tree;
  tree.addnode(st);
  cout<<"added start node\n";
  //  tree.number_nodes();
  std::vector<std::vector<float> > path;
  std::vector<float> samp,dir, low(3,-5),high;
  high.push_back(15);
  high.push_back(15);
  high.push_back(2*M_PI);
  unsigned int i,j;
  bool goalr;

  do
  {
    goalr=false;
    if((float)rand()/RAND_MAX<goalb) // to check if sample is goal
    {
      samp=goal; // setting sample as goal
      goalr= true;
    }
    else
    {
      samp=random(low,high); // chosing random sample
    }
    node=tree.near_neigh(samp);
    if (eucl(samp,node->get_conf())<=step)
    {
      cout<<"close\n";
      if(collision_box(samp)==true)
        continue;
      if (goalr)
        cout<<"goal !!!!!!!!!\n";
      else
        goalr=false;
      par=tree.parent_best(samp);
      inter=new RRTNode(samp,par,par->get_cost()+step_cost);
    }
    else
    {
      cout<<"extending node\n";
      goalr=false;
      dir=direction(node->get_conf(),samp,step);
      if(collision_box(dir)==true)
        continue;
      par=tree.parent_best(dir);
      inter=new RRTNode(dir,par,par->get_cost()+step_cost);
    }
    tree.addnode(inter);
    cout<<"rewiring\n";
    if(!(node==par))
    {
        node->set_parent(inter);
    }
    cout<<"rewired\n";
//    tree.number_nodes();
  }
  while(!goalr);
  time_e=time(NULL);
  cout<<"goal bias is \t"<<goalb<<"\n";
  cout<<"seconds = "<<(time_e-time_s)<<"\n";
  //printing number of nodes sampled
  tree.number_nodes();
  //getting path
  path=tree.get_path();
  //reversing path
  std::reverse(path.begin(),path.end());
  cout<<"path size"<<path.size()<<"\n";
  for(i=0;i<path.size();i++)
  {
    for (j=0;j<path[i].size();j++)
      cout<<path[i][j]<<" ";
    cout<<"\n";
  }
  return path;
}
//int main()
//{
//  std::vector<float> begining(3,0),end(3,20);
//  RRTpath(begining,end);
//  return 0;
//}
