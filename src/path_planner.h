#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <limits>
#include <cmath>
#include <vector>
#include <memory>
#include "SDL.h"

class Node{
 public:
   Node(int x, int y): pos{x, y}{}
   ~Node(){}
   SDL_Point pos; 
   float h_value = std::numeric_limits<float>::max(); 
   float g_value = 0.0; 
   //bool visited = false; 
   //std::vector<Node> neighbors; 
   Node *parent = nullptr; 
};

class Path_Planner{
  public:
    Path_Planner(SDL_Point &start, SDL_Point &goal, int w, int h)
        : start_{start}, goal_{goal}, grid_width_(w), grid_height_(h){
        //std::unique_ptr<Node> current_node = std::make_unique<Node>(start); 
        current_node = new Node(start.x, start.y); 
    }
    ~Path_Planner(){
        if(current_node != nullptr){
            delete current_node; 
        }
        for(auto &node: open_nodes){
            if(node != nullptr){
                delete node; 
            }
        }
    }

    bool CheckCellValid(int x, int y); 
    float Heuristic(Node *node); 
    void ExpandNeighbors(); 
    void NextNode(); 
    void ConstructFinalPath(); 
    std::vector<SDL_Point> AStarSearch(); 

  private:
    Node *current_node; 
    SDL_Point start_;  
    SDL_Point goal_;  
    int grid_width_; 
    int grid_height_; 

    std::vector<SDL_Point> delta_nodes{        {0,-1},
                                       {-1, 0},       {1, 0},
                                               {0, 1}};

    std::vector<Node *> open_nodes; 
    std::vector<SDL_Point> path_to_goal_; 

}; 

#endif
