
#include <iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <algorithm>

#define PI 3.14
using namespace std;

const float stepx=0.1, stepy=0.1, steptheta=0.1;

class Astar_Node
{
  std::vector<float> points;
  Astar_Node* parent;
  float cost;

  public:

    Astar_Node(std::vector<float> v, Astar_Node* p=NULL, float c=0)
    {

      points=v;
      parent=p;
      cost=c;
    }

    bool operator==(Astar_Node obje)
    {
      std::vector<float> obj=obje.get_config();
      for(unsigned int i=0;i<points.size();i++)
      {
        if(points[i]!=obj[i])
        // if(points!=obj)
          return false;
      }
      return true;
    }

    void set_parent(Astar_Node* p)
    {
      parent=p;
    }

    Astar_Node* get_parent()
    {
      return parent;
    }

    std::vector<float> get_config()
    {
      return points;
    }

    float get_cost()
    {
      return cost;
    }

    bool goalcheck(Astar_Node goal)
    {
      std::vector<float> gol,curr;
      float checkx,checky,checkz;
      gol=goal.get_config();
      checkx=pow((points[0]-gol[0]),2);
      checky=pow((points[1]-gol[1]),2);
      checkz=pow((points[2]-gol[2]),2);
      if((checkx+checky+checkz)<=(pow(stepx,2)+pow(stepy,2)+pow(steptheta,2)))
        return true;
      return false;
    }

    void disp()
    {
      for(unsigned int i=0;i<points.size();i++)
        cout<<points[i]<<"\t";
      cout<<"\n";
    }
};


class node_list
{
  std::vector<Astar_Node*> nodes;
  std::vector<float> priority;

  public:

  void put(float pri,Astar_Node* node)
  {
    nodes.push_back(node);
    priority.push_back(pri);
  }

  Astar_Node* get()
  {
    float least=priority[0];
    int index=0;
    for(unsigned int i=1;i<priority.size();i++)
    {
      if(priority[i]<least)
      {
        least=priority[i];
        index=i;
      }
    }
    return nodes[index];
    // ];
  }

  void del(Astar_Node *node)
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

bool srch(std::vector<std::vector<float> > v,std::vector<float> con)
{
  bool flag;
  for(unsigned int i=0;i<v.size();i++)
  {
    flag=true;
    for(unsigned int j=0;j<v[i].size();j++)
    {
      if(v[i][j]!=con[j])
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
  neigh.clear();
  // std::vector<Astar_Node> b;
  unsigned int i,j,k;
  std::vector<float> temp;
  for(i=0;i<3;i+=2)
  {
    for(j=0;j<3;j+=2)
    {
      for(k=0;k<3;k+=2)
      {
        temp.clear();
        temp.push_back(x[0]+(i-1)*stepx);
        temp.push_back(x[1]+(j-1)*stepy);
        temp.push_back(x[2]+(k-1)*steptheta);
        neigh.push_back(temp);
      }
    }
  }
  return neigh;
//collision check and drwing path.
}

Astar_Node* Astar(std::vector<float> beg, std::vector<float> goal)
{
  time_t time_s,time_e;
  time_s=time(NULL);
  float new_cost,current_cost,evalfn;
  node_list nodes;
  Astar_Node *current_node,*start,*gol,*node_new,*temp;
  std::vector<std::vector<float> > neighbors,visited;
  start = new Astar_Node(beg);
  nodes.put(0,start);
  while(!nodes.Empty())
  {
    neighbors.clear();
    current_node=nodes.get();
    nodes.del(current_node);
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
    neighbors=neigh_conn(current_node->get_config());
    for(unsigned int i=0;i<neighbors.size();i++)
    {
      new_cost=current_node->get_cost()+0.05;
      if(!srch(visited,neighbors[i]))
      {
        node_new = new Astar_Node(neighbors[i],current_node,new_cost);
        evalfn=new_cost+heuristic(neighbors[i],goal);
        nodes.put(evalfn,node_new);
        continue;
      }
      temp=nodes.search(neighbors[i]);
      if(new_cost<temp->get_cost())
      {
        temp->set_parent(current_node);
        evalfn=new_cost+heuristic(temp->get_config(),goal);
        nodes.put(evalfn,temp);
      }
    }
  }
}

std::vector<Astar_Node*> path(Astar_Node* goal)
{
  std::vector<Astar_Node*> pat;
  Astar_Node *temp;
  pat.push_back(goal);
  temp=goal->get_parent();
  while(temp!=NULL)
  {
    pat.push_back(temp);
    temp=temp->get_parent();
  }

  std::reverse(pat.begin(),pat.end());
  return pat;
  // for(unsigned int i=0;i<pat.size();i++)
  //   pat[i]->disp();
}
// int main()
// {
//   std::vector<float> begining(3,0),end(3,2.4);
//   Astar_Node *g;
//   g=Astar(begining,end);
//
//   path(g);
//   return 0;
// }
