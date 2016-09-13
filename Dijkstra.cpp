//
//  main.cpp
//  Djikstra
//
//  Created by James Vaughan Craster on 10/07/2016.
//  Copyright (c) 2016 James Vaughan Craster. All rights reserved.
//

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
//functions to calculate the connections between nodes using vector equations to establish line of sight between them, currently unused:
/*float calculateLambda(sf::Vector2f lightCenter, sf::Vector2f secondLightVertex, sf::Vector2f firstVertex, sf::Vector2f secondVertex){
 
 float lambda = ((lightCenter.y - firstVertex.y)*(secondVertex.x - firstVertex.x) - (lightCenter.x - firstVertex.x)*(secondVertex.y - firstVertex.y))/((secondLightVertex.x - lightCenter.x)*(secondVertex.y - firstVertex.y) - (secondLightVertex.y - lightCenter.y)*(secondVertex.x - firstVertex.x));
 
 return lambda;
 
 }
 
float calculateMew(sf::Vector2f lightCenter, sf::Vector2f secondLightVertex, sf::Vector2f firstVertex, sf::Vector2f secondVertex, float lambda){
 float mew;
 
 if(std::abs(secondVertex.x - firstVertex.x) < 0.01){
 mew = (lightCenter.y + lambda *(secondLightVertex.y - lightCenter.y) - firstVertex.y)/(secondVertex.y - firstVertex.y);
 
 }else{
 mew = 
 (lightCenter.x + lambda*(secondLightVertex.x - lightCenter.x) - firstVertex.x)/(secondVertex.x - firstVertex.x);
 
 
 }
 return  mew;
 
 }*/

bool testNodeConnection(sf::Vector2f lightCenter, sf::Vector2f secondLightVertex, sf::Vector2f firstVertex, sf::Vector2f secondVertex)
{
    float lambda;
    lambda = ((lightCenter.y - firstVertex.y)*(secondVertex.x - firstVertex.x) - (lightCenter.x - firstVertex.x)*(secondVertex.y - firstVertex.y))/((secondLightVertex.x - lightCenter.x)*(secondVertex.y - firstVertex.y) - (secondLightVertex.y - lightCenter.y)*(secondVertex.x - firstVertex.x));
    
    if(lambda > 0 && lambda < 1){
        float mew;
        
        if(std::abs(secondVertex.x - firstVertex.x) < 0.01){
            mew = (lightCenter.y + lambda *(secondLightVertex.y - lightCenter.y) - firstVertex.y)/(secondVertex.y - firstVertex.y);
            
        }else{
            mew =
            (lightCenter.x + lambda*(secondLightVertex.x - lightCenter.x) - firstVertex.x)/(secondVertex.x - firstVertex.x);
            
            
        }
        
        if(mew > 0 && mew < 1){
            return 0;
        }
        
        
    }
    
    return 1;
}



class Node{
public:
    std::vector<Node*> nodeVector;
    sf::Vector2f position;
    float distance = INFINITY;
    int state = 0;
    bool backwardsPath = 0;
    
    Node(sf::Vector2f passPosition){
        position = passPosition;
        
    }
};




void addTemporaryNode(std::vector <Node*> &nodeMap, std::vector<Node*> &temporaryNodes){
    
    for(int x = 0; x < nodeMap[nodeMap.size()-1]->nodeVector.size(); x++){
        if(nodeMap[nodeMap.size()-1]->nodeVector[x]->state == 0)
        {
        
            temporaryNodes.push_back(nodeMap[nodeMap.size()-1]->nodeVector[x]);
            temporaryNodes[temporaryNodes.size()-1]->state = 1;
        
        }
        
        float magnitude;
        
        
        /*if(temporaryNodes[temporaryNodes.size()-1]->distance > pow(pow(nodeMap[nodeMap.size()-1]->position.x - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.x, 2) + pow(nodeMap[nodeMap.size()-1]->position.y - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.y, 2), 0.5) +  + nodeMap[nodeMap.size() -1]->distance)
            {
            
                temporaryNodes[temporaryNodes.size()-1]->distance = pow(pow(nodeMap[nodeMap.size()-1]->position.x - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.x, 2) + pow(nodeMap[nodeMap.size()-1]->position.y - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.y, 2), 0.5) + nodeMap[nodeMap.size() -1]->distance;
            
            }*/
        magnitude = pow(pow(nodeMap[nodeMap.size()-1]->position.x - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.x, 2) + pow(nodeMap[nodeMap.size()-1]->position.y - nodeMap[nodeMap.size()-1]->nodeVector[x]->position.y, 2), 0.5) + nodeMap[nodeMap.size() -1]->distance;
        
        if(nodeMap[nodeMap.size()-1]->nodeVector[x]->distance > magnitude)
        {
            
            nodeMap[nodeMap.size()-1]->nodeVector[x]->distance = magnitude;
            
        }
        
        
        
        
    }
    
}

//let heuristic return 0 for Dijkstra's algorithm
float heuristic(sf::Vector2f goal, sf::Vector2f current){
    return pow(pow(goal.x - current.x, 2) + pow(goal.y - current.y,2), 0.5);
    
}

void addPermanentNode(std::vector<Node*> & temporaryNodes, std::vector<Node*> & permanentNodes, sf::Vector2f goal, float (*h)(sf::Vector2f, sf::Vector2f)){
    float distance = INFINITY;
    
    float heuristicValue;
    int storedIndex = 0;
    for(int x = 0; x < temporaryNodes.size(); x++){
        
        heuristicValue = h(goal, temporaryNodes[x]->position);
        
        
        if(temporaryNodes[x]->distance + heuristicValue < distance)
        {
            
           distance = temporaryNodes[x]->distance + heuristicValue;
            
            storedIndex = x;
            
        }
        
        
    }
    temporaryNodes[storedIndex]->state = 2;
    permanentNodes.push_back(temporaryNodes[storedIndex]);
    temporaryNodes.erase(temporaryNodes.begin() + storedIndex);
    
}

void buildConnectionMap(std::vector<Node*> nodeMap, std::vector<sf::VertexArray> edgeVector, float spacing){
    
    bool breakTheLoopTwice = 0;
    
    for(int a = 0; a < nodeMap.size(); a++){
        for(int i = 0; i < nodeMap.size(); i++){
            
            
            breakTheLoopTwice = 0;
            if(a != i){
                if(pow(pow(nodeMap[a]->position.x - nodeMap[i]->position.x,2) +  pow(nodeMap[a]->position.y - nodeMap[i]->position.y,2),0.5) < spacing){
                for (int x = 0; x < edgeVector.size(); x++)
                {
                    if(!(testNodeConnection(nodeMap[a]->position, nodeMap[i]->position, edgeVector[x][0].position, edgeVector[x][1].position))){
                        breakTheLoopTwice = 1;
                        break;
                        
                    }
                       
                }
                if(breakTheLoopTwice == 1){
                    continue;
                }
                
                nodeMap[a]->nodeVector.push_back(nodeMap[i]);
                }
                
                
            }
        }
        
    }
    
    
}

void AStar(int start, int end, std::vector <Node*> &nodeMap, std::vector<Node*>&backwardsPath, float (*h)(sf::Vector2f, sf::Vector2f), sf::RenderWindow * window){
    
    std::vector<Node*>  permanentNodes;
    std::vector<Node*>  temporaryNodes;
    
    
    for (int a = 0; a < nodeMap.size(); a++) {
        nodeMap[a]->state = 0;
        nodeMap[a]->distance = INFINITY;
        nodeMap[a]->backwardsPath = 0;
    }
    
    backwardsPath.clear();
    
    nodeMap[start]->distance = 0;
    nodeMap[start]->state = 2;
    
    permanentNodes.push_back(nodeMap[start]);
    
   
    
   for(int x = 0; x < nodeMap[start]->nodeVector.size(); x++){
        temporaryNodes.push_back(nodeMap[start]->nodeVector[x]);
        
        temporaryNodes[temporaryNodes.size()-1]->distance = pow(pow(nodeMap[start]->position.x - nodeMap[start]->nodeVector[x]->position.x, 2) + pow(nodeMap[start]->position.y - nodeMap[start]->nodeVector[x]->position.y, 2), 0.5);
        
        
        
        temporaryNodes[temporaryNodes.size()-1]->state = 1;
        
        
    }
    
    while(nodeMap[end]->state != 2){
        
        addPermanentNode(temporaryNodes, permanentNodes, nodeMap[end]->position, h);
        addTemporaryNode(permanentNodes, temporaryNodes);
        if(temporaryNodes.size() == 0){
            
            return;
           
        }
        
    }
    float magnitude;
    sf::CircleShape circle(5);
    circle.setOrigin(5, 5);
    circle.setFillColor(sf::Color::Magenta);
    for(int b = 0; b < permanentNodes.size(); b++){
        circle.setPosition(permanentNodes[b]->position);
        if(permanentNodes[b]->state != 2){
            std::cout << "aaaah";
        }
    }
    
    circle.setFillColor(sf::Color::Red);
    for(int b = 0; b < temporaryNodes.size(); b++){
        circle.setPosition(temporaryNodes[b]->position);
       
        }



   
    
    backwardsPath.push_back(nodeMap[end]);
    
    
    
    for(int x = 0; x < backwardsPath.size(); x++){
    for(int a = 0; a < backwardsPath[x]->nodeVector.size(); a++){
        if(backwardsPath[x]->nodeVector[a]->state == 2){
            if(backwardsPath[x]->nodeVector[a]->backwardsPath == 0){
            
            
            
            magnitude = pow(
                            pow(backwardsPath[x]->position.x - backwardsPath[x]->nodeVector[a]->position.x,2) + pow(backwardsPath[x]->position.y - backwardsPath[x]->nodeVector[a]->position.y, 2),
                            0.5);
          
            if(std::abs(magnitude - ((backwardsPath[x]->distance - backwardsPath[x]->nodeVector[a]->distance))) < 0.01){
                backwardsPath.push_back(backwardsPath[x]->nodeVector[a]);
                backwardsPath[x]->nodeVector[a]->backwardsPath = 1;
                
                 if(backwardsPath[x]->nodeVector[a]->position == nodeMap[start]->position){
                     backwardsPath.push_back(backwardsPath[x]->nodeVector[a]);
                 }
                break;
                
                
            }
            }
        }
    }
}
    return backwardsPath;
    
}

bool equatePoint(sf::Vector2f a, sf::Vector2f b, float epsilon = 1){
    if(std::abs(a.x - b.x) < epsilon){
        if(std::abs(a.y - b.y) < epsilon){
            return 1;
        }
    }
    return  0;
}





int main(int, char const**)
{
    // Create the main window
    sf::RenderWindow window(sf::VideoMode(1400, 900), "SFML window");
    
    
    std::vector<Node*> nodeMap;
 
    std::vector<sf::VertexArray> edgeVector;

    std::string map ="NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOONOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOOOOOOOONOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN";
    
    float width = 100;
    float height = 10;
    float row;
    float column;
    int highlighted = 0;
    float sideLength = 10;
    float nodeDistance = sideLength;
    
    
    
    for(float x = 0; x < map.length(); x++){
        row = floor(x/width) * nodeDistance;
        column = (x/width - floor(x/width)) * width * nodeDistance;
        
        if(map[x] == 'N'){
            nodeMap.push_back(new Node(sf::Vector2f(column+10, row+10)));
            
        }
        
        
    }
    sf::Vector2f nodePosition;
    
    for(int a = 0; a < nodeMap.size(); a++){
        for(int b = 0; b < nodeMap.size(); b++){
            if(a != b){
                nodePosition = nodeMap[a]->position;
                if(equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(0, nodeDistance)) ||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(0, -nodeDistance))||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(nodeDistance, 0)) ||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(-nodeDistance, 0))||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(nodeDistance, nodeDistance))||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(-nodeDistance, -nodeDistance))||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(-nodeDistance, nodeDistance))||
                   equatePoint(nodeMap[a]->position,nodeMap[b]->position + sf::Vector2f(nodeDistance, -nodeDistance))){
                    nodeMap[a]->nodeVector.push_back(nodeMap[b]);
                    
                }
                
                
            }
        }
    }
    
    
    
    sf::CircleShape circle(5);
    circle.setFillColor(sf::Color::Blue);
    circle.setOrigin(5,5);
    sf::RectangleShape rect;
    rect.setSize(sf::Vector2f(50,50));
    rect.setFillColor(sf::Color::Green);
    
   
    int start = 50;
    int end = nodeMap.size()-1;
    std::vector<Node*> permanentNodes;
    std::vector<Node*> temporaryNodes;
    std::vector<Node*> backwardsPath;
    
    AStar(start, end, nodeMap, backwardsPath, heuristic, & window);
    
    rect.setPosition(backwardsPath[0]->position);
    rect.setOrigin(sf::Vector2f(25,25));
    int state = 0;
    
    float magnitude;
    
    window.setFramerateLimit(60);
    
    sf::VertexArray edge(sf::LinesStrip);
    edge.resize(2);
    edge[0].color = sf::Color::Yellow;
    edge[1].color = sf::Color::Yellow;
    
    sf::Clock calcClock;
    

    // Start the game loop
    while (window.isOpen())
    {
        // Process events
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window: exit
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // Escape pressed: exit
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
            
            if (event.type == sf::Event::MouseButtonPressed) {
                end += 30;
                if(end >= nodeMap.size()){
                    end = 0;
                }
                highlighted += 1;
                 AStar(start, end, nodeMap, backwardsPath, heuristic, &window);
                rect.setPosition(backwardsPath[0]->position);
                state = 0;
                
            }
            
        }
        
        if(highlighted >= backwardsPath.size()-1){
            highlighted = 0;
        }
        
        
        window.clear();
        circle.setFillColor(sf::Color::Blue);
        for (int a = 0; a < nodeMap.size(); a++) {
            circle.setPosition(nodeMap[a]->position);
             window.draw(circle);
        }
       
        
        
        if(equatePoint(backwardsPath[state]->position, rect.getPosition())){
            state -= 1;
        }
        if(state < 0){
            state = backwardsPath.size()-1;
        }
        
        if(calcClock.getElapsedTime().asSeconds() >= 0.5 || state == 0){
            calcClock.restart();
            backwardsPath.clear();
            for(int a = 0; a < nodeMap.size(); a++){
                if(equatePoint(nodeMap[a]->position, sf::Vector2f(sf::Mouse::getPosition(window)), 8.0)){
                    end = a;
                    break;
                }
            }
            for(int a = 0; a < nodeMap.size(); a++){
                if(equatePoint(rect.getPosition(), nodeMap[a]->position, 8.0)){
                    start = a;
                    break;
                }
            }
            
            
            AStar(start, end, nodeMap, backwardsPath, heuristic, &window);
            state = backwardsPath.size()-2;
            
        }
        if(backwardsPath.size() > 0 && state < backwardsPath.size()){
        magnitude = pow(pow(rect.getPosition().x - backwardsPath[state]->position.x,2) + pow(rect.getPosition().y - backwardsPath[state]->position.y,2),0.5);
        rect.move((backwardsPath[state]->position.x - rect.getPosition().x)/magnitude, (backwardsPath[state]->position.y - rect.getPosition().y)/magnitude);
        circle.setFillColor(sf::Color::Yellow);
        }
        for(int a = 0; a < backwardsPath.size(); a++){
            circle.setPosition(backwardsPath[a]->position.x, backwardsPath[a]->position.y);
            window.draw(circle);
        }
        
        edge.clear();
        edge.resize(2);
        if(backwardsPath.size() != 0){
        for(int a = 0; a < backwardsPath.size()-1; a++ ){
            edge[0].position = backwardsPath[a]->position;
            edge[1].position = backwardsPath[a+1]->position;
            window.draw(edge);
        }
        }
        
        
        for(int a = 0; a < edgeVector.size(); a++){
            window.draw(edgeVector[a]);
            
        }
        if(end != 0){
        for (int a = 0; a < nodeMap[end-1]->nodeVector.size(); a++) {
            circle.setPosition(nodeMap[end-1]->nodeVector[a]->position);
        }
        }
        
        window.draw(rect);
        
        //update window
        window.display();
    }

    return EXIT_SUCCESS;
}
