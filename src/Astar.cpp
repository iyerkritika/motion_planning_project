
#include <iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <algorithm>
#include <math.h>
#include "obstacle_one_box.cpp"

//#define PI 3.14
using namespace std;
std::vector<std::vector<float> > visited;
const float stepx=0.1, stepy=0.1, steptheta=M_PI/4;

class Astar_Node
{
  std::vector<float> config;
  Astar_Node* parent;
  float cost;

  public:

    Astar_Node(std::vector<float> v, Astar_Node* p=NULL, float c=0)
    {

      config=v;
      parent=p;
      cost=c;
    }

    bool operator==(Astar_Node* obj)
    {
      std::vector<float> obje=obj->get_config();
      for(unsigned int i=0;i<config.size();i++)
        if(config[i]!=obje[i])
          return true;
    }

    void set_parent(Astar_Node* p,float co)
    {
      parent=p;
      cost=co;
    }

    Astar_Node* get_parent()
    {
      return parent;
    }

    std::vector<float> get_config()
    {
      return config;
    }

    float get_cost()
    {
      return cost;
    }

    bool goalcheck(std::vector<float> gol)
    {
      float checkx,checky;
      checkx=pow((config[0]-gol[0]),2);
      checky=pow((config[1]-gol[1]),2);
      if((checkx+checky)<=(pow(stepx,2)+pow(stepy,2)))
        return true;
      return false;
    }

    void disp()
    {
      for(unsigned int i=0;i<config.size();i++)
        cout<<config[i]<<"\t";
      cout<<"\n";
    }
};


class node_list
{
  std::vector<Astar_Node*> nodes;
  std::vector<float> priority;

  public:

  void put_node(float pri,Astar_Node* node)
  {
    nodes.push_back(node);
    priority.push_back(pri);
  }

  Astar_Node* get_node()
  {
    float least=priority[0];
    int index=0;
    for(unsigned int i=1;i<priority.size();i++)
    {
      if(priority[i]<=least)
      {
        least=priority[i];
        index=i;
      }
    }
    return nodes[index];
  }

  void del_node(Astar_Node *node)
  {
    for (unsigned int i=0;i<nodes.size();i++)
    {
      if(nodes[i]==node)
      {
        nodes.erase(nodes.begin()+i);
        priority.erase(priority.begin()+i);
        break;
      }
    }
  }

  bool Empty()
  {
    if(nodes.empty())
      return true;
    return false;
  }

  Astar_Node* search(std::vector<float> vect)
  {
    for (unsigned int i=0;i<nodes.size();i++)
      if(nodes[i]->get_config()==vect)
        return nodes[i];
  }

};

bool srch(std::vector<float> con)
{
  bool flag;
  for(unsigned int i=0;i<visited.size();i++)
  {
    flag=true;
    for(unsigned int j=0;j<visited[i].size();j++)
    {
      if(visited[i][j]!=con[j])
      {
        flag=false;
        break;
      }
    }
  }
  return flag;
}

float heuristic(std::vector<float> a, std::vector<float> b)
{
  return sqrt(pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2));
}

 std::vector<std::vector<float> > neigh_conn(std::vector<float> x)
{
  std::vector<std::vector<float> > neigh;
  unsigned int i,j,k;
  float a;
  std::vector<float> temp;
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
    {
      for(k=0;k<3;k++)
      {
        if(i==1 && j==1 && k==1)
          continue;
        temp.clear();
        temp.push_back(x[0]+(i-1)*stepx);
        temp.push_back(x[1]+(j-1)*stepy);
        a=remainder(x[2]+(k-1)*steptheta,2*M_PI);
        if(a<0)
          a=a+(2*M_PI);
        temp.push_back(a);
        if(collision_narrow(temp))
        {
            cout<<"collision\n";
            continue;
        }
          neigh.push_back(temp);
      }
    }
  }
  return neigh;
}

Astar_Node* Astar(std::vector<float> beg, std::vector<float> goal)
{
  time_t time_s,time_e;
  time_s=time(NULL);
  float new_cost,evalfn;
  node_list open;
  Astar_Node *current_node,*start,*gol,*node_new,*temp;
  std::vector<std::vector<float> > neighbors;
  start = new Astar_Node(beg);
//  cout<<"pushed start in list\n";
  open.put_node(0,start);
  while(!open.Empty())
  {
    neighbors.clear();
    current_node=open.get_node();
//    cout<<"got node to explore\n";
//    current_node->disp();
    open.del_node(current_node);
    visited.push_back(current_node->get_config());
    if(current_node->goalcheck(goal))
    {
      node_new=new Astar_Node(current_node->get_config(),current_node->get_parent(),current_node->get_cost()+0.05);
      cout<<"yaay goal\n";
      gol=new Astar_Node(goal,node_new,node_new->get_cost()+0.05);
      time_e=time(NULL);
      cout<<"time taken to compute = "<<time_e-time_s<<"\n";
      cout<<"total number of nodes explored = "<<visited.size()+1<<"\n";
      return gol;
    }
    cout<<"neighbors\n";
    neighbors=neigh_conn(current_node->get_config());
    for(unsigned int i=0;i<neighbors.size();i++)
    {
      new_cost=current_node->get_cost()+0.05;
      if(!srch(neighbors[i]))
      {
//        cout<<"new node\n";
        node_new = new Astar_Node(neighbors[i],current_node,new_cost);
        evalfn=new_cost+heuristic(neighbors[i],goal);
        open.put_node(evalfn,node_new);
        continue;
      }
      temp=open.search(neighbors[i]);
      if(new_cost<temp->get_cost())
      {
        temp->set_parent(current_node,new_cost);
        evalfn=new_cost+heuristic(temp->get_config(),goal);
        open.put_node(evalfn,temp);
      }
    }
  }
}

std::vector<std::vector<float> > path(Astar_Node* goal)
{
  std::vector<std::vector<float> > pat;
  Astar_Node *temp;
  pat.push_back(goal->get_config());
  temp=goal->get_parent();
  while(temp!=NULL)
  {
    pat.push_back(temp->get_config());
    temp=temp->get_parent();
  }

  std::reverse(pat.begin(),pat.end());
  for(unsigned int i=0;i<pat.size();i++)
    cout<<pat[i][0]<<" "<<pat[i][1]<<" "<<pat[i][2]<<"\n";
  return pat;
}

//int main()
//{
//  std::vector<float> begining(3,0),end(3,2.4);
//  Astar_Node *g;
//  g=Astar(begining,end);
//  cout<<"the path to be followed is \n ";
//  path(g);
//  return 0;
//}
