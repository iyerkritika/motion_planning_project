
#include <iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <algorithm>
#include "obstacle_one_box.cpp"

#define PI 3.14
using namespace std;

std::vector<std::vector<float> > visited;
const float stepx=0.1, stepy=0.1, steptheta=0.1;
int epsilon=1;

class ARAstar_Node
{
  std::vector<float> config;
  ARAstar_Node* parent;
  float cost;

  public:
    ARAstar_Node(std::vector<float> v,ARAstar_Node* par=NULL,float c=0)
    {
      config=v;
      parent=par;
      cost=c;
    }

    ARAstar_Node* get_parent()
    {
      return parent;
    }

    void set_parent(ARAstar_Node* par,float co)
    {
      parent=par;
      cost=co;
    }

    bool operator==(ARAstar_Node* obj)
    {
      std::vector<float> obje=obj->get_config();
      for(unsigned int i=0;i<config.size();i++)
        if(config[i]!=obje[i])
          return true;
      return false;
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

};

class node_list
{
  std::vector<ARAstar_Node*> nodes;
  std::vector<float> priority;

  public:

    void put_node(float pri,ARAstar_Node* node)
    {
      nodes.push_back(node);
      priority.push_back(pri);
    }

    void add_nodes(node_list other)
    {
      std::vector<ARAstar_Node*> list;
      std::vector<float> pri;
      list=other.get_nodes();
      pri=other.get_priorities();
      nodes.insert(nodes.end(),list.begin(),list.end());
      priority.insert(priority.end(),pri.begin(),pri.end());
    }

    ARAstar_Node* get_node()
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

    void del_node(ARAstar_Node* node)
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

    ARAstar_Node* search(std::vector<float> vect)
    {
      for (unsigned int i=0;i<nodes.size();i++)
        if(nodes[i]->get_config()==vect)
          return nodes[i];
        return NULL;
    }

    void clearing()
    {
      nodes.clear();
      priority.clear();
    }

    std::vector<ARAstar_Node*> get_nodes()
    {
      return nodes;
    }

    std::vector<float> get_priorities()
    {
      return priority;
    }

    void number_nodes()
    {
      cout<<"number of nodes"<<nodes.size()<<"\n";
      cout<<"number of pri"<<priority.size()<<"\n";
    }
};

node_list open,incons,closed;

float heuristic(std::vector<float> a, std::vector<float> b)
{
  return sqrt(pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2));
}

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
       if(collision_narrow(temp)==true)
           continue;
        a=remainder(x[2]+(k-1)*steptheta,2*M_PI);
        if(a<0)
          a=a+(2*M_PI);
       temp.push_back(x[2]+(k-1)*steptheta);
       neigh.push_back(temp);
     }
   }
 }
 return neigh;
}

ARAstar_Node* improve_path(ARAstar_Node* goal_node)
{
  std::vector<std::vector<float> > neighbors;
  ARAstar_Node *current_node,*node_new,*temp;
  float new_cost,evalfn;
  while(1)
  {
    neighbors.clear();
    current_node=open.get_node();
    open.del_node(current_node);
    visited.push_back(current_node->get_config());
    closed.put_node(0,current_node);
    new_cost=current_node->get_cost()+0.05;
    if(current_node->goalcheck(goal_node->get_config()))
    {
      goal_node->set_parent(current_node,new_cost);
      return goal_node;
    }
    neighbors=neigh_conn(current_node->get_config());
    for (unsigned int i=0;i<neighbors.size();i++)
    {
      evalfn=new_cost+epsilon*heuristic(neighbors[i],goal_node->get_config());
      if(!srch(neighbors[i]))
      {
        node_new=new ARAstar_Node(neighbors[i],current_node,new_cost);
        open.put_node(evalfn,node_new);
      }
      else
      {
        temp=closed.search(neighbors[i]);
        if(new_cost<temp->get_cost())
        {
          temp->set_parent(current_node,new_cost);
          incons.put_node(evalfn,temp);
        }
      }
    }
  }
}

std::vector<std::vector<float> > path(ARAstar_Node* goal_node)
{
  std::vector<std::vector<float> > pat;
  ARAstar_Node *temp;
  pat.push_back(goal_node->get_config());
  temp=goal_node->get_parent();
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

std::vector<std::vector<float> > ARAstar(std::vector<float> beg, std::vector<float> goal)
{
  cout<<"im here\n";
  time_t time_s,time_e;
  time_s=time(NULL);
  float evalfn;
  ARAstar_Node *start,*gol;
  std::vector<std::vector<float> > ara_path;
  start = new ARAstar_Node(beg);
  open.put_node(0,start);
  gol=new ARAstar_Node(goal);
  cout<<"came here\n";
  gol=improve_path(gol);
  cout<<"ooo\n";
  ara_path=path(gol);
  return ara_path;
  cout<<"next\n";
  open.number_nodes();
  while(epsilon>1)
  {
    cout<<"hi\n";
    epsilon--;
    open.add_nodes(incons);
    open.number_nodes();
    incons.clearing();
    closed.clearing();
    gol=improve_path(gol);
    ara_path=path(gol);
    cout<<"next\n";
  }
}

 int main()
 {
  std::vector<float> begining(3,0),end;
  end.push_back(5);
  end.push_back(7);
  end.push_back(3);
  ARAstar(begining,end);
  return 0;
 }
