//
// Created by Thomas on 15.05.18.
//
//*********************************************************************************************************************
// A* is an informed search algorithm
// The A* algorithm always examines the nodes first that are likely to lead quickly to the target.
// To determine the most promising node, a value f(x) is assigned to each known node x.
// This value specifies how long the path from start to destination is in the best case using
// the node under consideration. The node with the lowest f-value is examined next.
//*********************************************************************************************************************

#pragma once

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <vector>
#include "Graph.h"
#include "Triangle.h"
#include "ExplorationAgenda.h"


class CAStar {
private:
    CGraph* graph; //Adjacency list of the Graph
    const std::vector<CTriangle>* p3DMesh; //3D Point representation of the Vertices
    std::vector<int>* pOutBuffer;
    std::vector<CTriangle::SPoint3D>* pRoute3DPoints;
    ExplorationAgenda* explorationAgenda;

    // helper functions:
    const CTriangle::SPoint3D* get3DPoint(int vertexID) const;
    float HScore(glm::vec3 fPos, glm::vec3 fPosTarget);

    //alternative Heurisitc:
    //float HEuclidean(glm::vec3 fPos, glm::vec3 fPosTarget);

public:
    //public members for statistic purposes:
    int expansedVertixes;
    int steps;

    CAStar(CGraph* graph,const std::vector<CTriangle>* p3DMesh);
    ~CAStar();
    //The A* algorithm:
    float FindPath(int vertexIDStart, int vertexIDTarget, const unsigned int nOutBufferSize);

    //get functions for the OpenGL programm:
    //get route in vertexIDs
    std::vector<int>* getRoute();
    //get route in 3D Points for OpenGL
    const std::vector<CTriangle::SPoint3D>* getRoute3DPoints() const;
    const std::vector<CTriangle::SPoint3D>* getExpansed3DPoints() const;
};



