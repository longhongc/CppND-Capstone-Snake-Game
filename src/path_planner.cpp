#include "path_planner.h"
#include <iostream>
#include <algorithm>
#define debug(x) std::cout << (#x) << ": " << x << std::endl; 

bool Path_Planner::CheckCellValid(int x, int y){
    for(auto &node: open_nodes){
        if(node->pos.x == x && node->pos.y == y){
            return false; 
        }
    }
    return true; 
}

float Path_Planner::Heuristic(Node *node){
    return std::sqrt(std::pow(node->pos.x - goal_.x, 2) + std::pow(node->pos.y - goal_.y, 2)); 
}

void Path_Planner::ExpandNeighbors(){
    for(auto &delta: delta_nodes){
        //int x = fmod(current_node->pos.x + delta.x + grid_width_, grid_width_); 
        //int y = fmod(current_node->pos.y + delta.y + grid_height_, grid_height_); 
        int x = current_node->pos.x + delta.x; 
        int y = current_node->pos.y + delta.y; 
        if(CheckCellValid(x, y)){
            Node *neighbor = new Node(x, y); 
            neighbor->g_value = current_node->g_value + 1;  
            neighbor->h_value = Heuristic(neighbor); 
            neighbor->parent = current_node; 
            open_nodes.push_back(neighbor); 
        }
    }
}

bool Compare(Node *node1, Node *node2){
    float f1 = node1->g_value + node1->h_value; 
    float f2 = node2->g_value + node2->h_value; 
    return f1 < f2;  
}

void Path_Planner::NextNode(){
    std::sort(open_nodes.begin(), open_nodes.end(), Compare); 
    current_node = *open_nodes.begin(); 
    open_nodes.erase(open_nodes.begin()); 
}

void Path_Planner::ConstructFinalPath(){

    while(current_node != nullptr){
        path_to_goal_.insert(path_to_goal_.begin(), current_node->pos);   
        current_node = current_node->parent; 
    }
    path_to_goal_.erase(path_to_goal_.begin()); 

}

std::vector<SDL_Point> Path_Planner::AStarSearch(){

    //current_node->visited = true; 
    open_nodes.push_back(current_node); 

    while(!open_nodes.empty()){
        // Find valid neighbors
        ExpandNeighbors(); 
        // Pop the neighbor with least cost from open list
        NextNode(); 
        if (current_node->pos.x == goal_.x && current_node->pos.y == goal_.y){
            break; 
        }
    }

    ConstructFinalPath(); 

    return path_to_goal_; 
}




