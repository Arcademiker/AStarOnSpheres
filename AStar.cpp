//
// Created by Thomas on 15.05.18.
//

#include "AStar.h"

CAStar::CAStar(CGraph *graph,const std::vector<CTriangle>* p3DMesh) {
    //Adjacency list of the Graph
    this->graph = graph;
    //3D Point representation of the Vertices
    this->p3DMesh = p3DMesh;
    //stores the VertexIDs that are on the Path
    this->pOutBuffer = new std::vector<int>;
    //stores the 3DPoints that are on the Path
    this->pRoute3DPoints = new std::vector<CTriangle::SPoint3D>;

    // ExplorationAgenda is a Priority Queue with key value pairs. The pair with the smallest key is on top.
    // key is the float value for the f(n) score of A*.
    // the value of the Priority Queue is the VertexID.
    // why priority queue, why not multimap?
    //  the std-priority_queue is faster than the std-multimap, because:
    //   The typical underlying implementation of a multimap is a balanced red/black tree.
    //   Repeated element removals from one of the extreme ends of a multimap has a good chance of skewing the tree,
    //   requiring frequent rebalancing of the entire tree. This is going to be an expensive operation.
    this->explorationAgenda = new ExplorationAgenda();
}

CAStar::~CAStar() {
    delete this->pOutBuffer;
    delete this->pRoute3DPoints;
    delete this->explorationAgenda;
}


/* A* algorithm pseudo code:
 *
 * take vertex from the Priority Queue with best (lowest) FScore
 *   explore all adjacent vertices
 *     check djacent vertices if already visited
 *       put GScore+edge weight in visit list
 *       remember parent node (for path extraction)
 *       calc HScore = Manhattan distance current node to target
 *       calc FScore = HScore + GScore+1
 *       put FScore as Key in Priority Queue
 *  repeat (break if target node is reached)
 *       follow the parent nodes back to the start
 *  return path length or -1 if no such path exists */
float CAStar::FindPath(int vertexIDStart, int vertexIDTarget, const unsigned int nOutBufferSize) {

    // exist a path variable
    bool bPath = false;

    // path length in steps (ignoring edge weights)
    float nPathLength = 0;

    // clear old pOutBuffer from last pathfinding (pOutBuffer stores the vertexIDs of the taken route)
    delete this->pOutBuffer;
    this->pOutBuffer = new std::vector<int>(nOutBufferSize,-1);

    glm::vec3 fPos = this->get3DPoint(vertexIDStart)->fPos;
    glm::vec3 fPosTarget = this->get3DPoint(vertexIDTarget)->fPos;

    // ExplorationAgenda is a Priority Queue with key value pairs.
    //  The pair with the smallest key on top marks the next Vertex which should be evaluated by the Pathfinding Algorithm.
    delete this->explorationAgenda;
    this->explorationAgenda = new ExplorationAgenda();
    // explore start vertex (which means calculating f(n) score for this vertex): f(n) = h(n)+g(n)
    // g(n) := dijsktra distance from start to current vertex. for the start vertex = 0
    // h(n) := manhattan distance form target to current vertex ignoring all possible walls on the way. h(n) the heuristic for this A*
    // explorationAgenda->Add(HScore(fPos,fPosTarget)*this->normEdgeLength,vertexIDStart);
    explorationAgenda->Add(HScore(fPos,fPosTarget),vertexIDStart);

    // pGScore = g(n) ; key = vertexID , value = GScore
    // why hash map, why not a vector?
    //  a vector has a slightly faster access to its elements in many cases, however some maps can be very big to do pathfinding on,
    //  especially in modern games. The big advantage of unordered maps (hashmaps) is, they just keep track of the visited elements.
    //  If A* has to operate just in a small corner of a big map, the whole map (in case of a vector) has to be set to 0 at first
    //  to initialise the g(n) score values and would consume storage that is never needed the whole time the algorithm is in use.
    auto pGScore = new std::unordered_map<int,float>();
    // iterator to work with unordered map (hash map)
    std::pair<std::unordered_map<int,float>::iterator, bool> itVisited;
    pGScore->insert(std::make_pair(vertexIDStart,0));

    // parent Tree to find the shortest path from target back to the start (key = vertex, value = parent)
    auto pParentNode = new std::unordered_map<int,int>();
    // define starting point of the parent tree (parenting itself)
    pParentNode->insert({vertexIDStart,vertexIDStart});

    // direction up = 0, left = 1, down = 2, right = 3
    std::vector<int> nDir;

    // current visited vertex
    int vertexID = vertexIDStart;

    // distance in steps (ignoring edge weights)
    this->steps = 0;
    this->expansedVertixes = 0;

    // main loop of the A* algorithm
    while(steps < nOutBufferSize && !explorationAgenda->IsEmpty()) {

        // set the current vertex (node) to the vertex with the highest exploration priority (lowest f(n) Score)
        vertexID = explorationAgenda->VisitTop();

        expansedVertixes++; //for static purposes

        // total path length with respect to edge weights
        nPathLength = (*pGScore)[vertexID];

        // abort if target vertex is reached
        if (vertexIDTarget==vertexID) {
            // track way back to the start and write VertexIDs into pOutBuffer and 3Dpoints into pRoute3DPoints
            // count steps to start
            while((*pParentNode)[vertexID] != vertexID) {
                vertexID = (*pParentNode)[vertexID];
                steps++;
            }
            vertexID = vertexIDTarget;
            // create list of 3Dpoints for opengl draw routine
            delete this->pRoute3DPoints;
            this->pRoute3DPoints = new std::vector<CTriangle::SPoint3D>(steps+1);
            // find the shortest way back from the target to the start
            for(int i=steps; i>0; i--) {
                (*pOutBuffer)[i-1] = vertexID;
                (*pRoute3DPoints)[i] = *this->get3DPoint(vertexID);
                vertexID = (*pParentNode)[vertexID];
            }
            // insert start Point in 3D Point List
            (*pRoute3DPoints)[0] = *this->get3DPoint(vertexID);
            bPath = true;
            break;
        }


        // calculate the adjacent vertices of the current vertex
        for (auto u : this->graph->getAdjacent(vertexID)) {
            nDir.push_back(u.first);
        }

        // try to visit each direction
        // (6 directions in most cases. There are 12 Vertices on a tesselated icosahedron which have got 5 connecting edges)
        for(auto d: nDir) {
            // Check outer rim of the map && Check for Walls
            // if insertion is successful "itVisited.second" in the if-statement becomes true
            itVisited = pGScore->insert(std::make_pair(d, 0));
            // don't visit the same vertex twice
            if (itVisited.second) {
                // set the g(n) Score (without doing a second hashmap traversal) by raising the old g(n) by 1
                itVisited.first->second = (*pGScore)[vertexID] + this->graph->getEdgeWeight(this->graph->getAdjacent(vertexID).at(d));
                // keep track of the last visited vertex to find the shortest way back
                pParentNode->insert(std::make_pair(d, vertexID));
                // visit up and write FScore (f(n)=h(n)+g(n)) as key into the priority queue
                fPos = this->get3DPoint(vertexID)->fPos;
                explorationAgenda->Add(HScore(fPos, fPosTarget) + itVisited.first->second, d);
            }
        }
    }
    // end of the A* Algorithm


    delete pParentNode;
    delete pGScore;

    // return -1 when there is no way or the nOutBufferSize is too short
    if(!bPath) {
        if(nOutBufferSize<=nPathLength) {
            std::cout << std::endl << "Pathfinding was aborted, because there is no way to the target within the distance of "
                      << nOutBufferSize << " (nOutBufferSize)" << std::endl;
        }
        nPathLength = -1;
    }

    return nPathLength;
}

// This Heuristik calculates the arclength of two vectors which point to 3DPoints on the sphere.
float CAStar::HScore(glm::vec3 fPos, glm::vec3 fPosTarget) {
    //return glm::angle(fPos,fPosTarget);
    fPos = glm::normalize(fPos);
    fPosTarget = glm::normalize(fPosTarget);
    return 0.98f*acos(glm::dot(fPos, fPosTarget));

}

// alternative Heurisitcs (not in use):

/*
// Euclidean distance:
float CAStar::HEuclidean(glm::vec3 fPos, glm::vec3 fPosTarget) {
    return glm::sqrt((fPos.x-fPosTarget.x)*(fPos.x-fPosTarget.x)
           +(fPos.y-fPosTarget.y)*(fPos.y-fPosTarget.y)
           +(fPos.z-fPosTarget.z)*(fPos.z-fPosTarget.z));
}
 */

/*
// 3D Diagonal Distance 24-way method (based on 8-way method)
float CAStar::HScore(glm::vec3 fPos, glm::vec3 fPosTarget) {
    float Root2 = 1.41421f;
    float Root3 = 1.73205f;
    float deltax = glm::abs(fPos.x-fPosTarget.x);
    float deltay = glm::abs(fPos.y-fPosTarget.y);
    float deltaz = glm::abs(fPos.z-fPosTarget.z);

    float min_xy = glm::min(deltax,deltay);
    float d_min = glm::min(min_xy,deltaz);
    float max_xy = glm::max(deltax,deltay);
    float d_max = glm::max(max_xy,deltaz);
    //median of 3 elements := max(min(x,y), min(max(x,y),z));
    float d_median = glm::max(min_xy, glm::min(max_xy,deltaz));

    return Root3*d_min+Root2*(d_median-d_min)+d_max-d_median;
}
*/

// get functions for OpenGL Program:
const CTriangle::SPoint3D* CAStar::get3DPoint(int vertexID) const {
    //todo TPID explain
    std::pair<int,int> TPID = this->graph->get3DPointIDofVertexID(vertexID);
    return this->p3DMesh->at(TPID.first).GetPoints(TPID.second);
}

std::vector<int>* CAStar::getRoute() {
    return this->pOutBuffer;
}

const std::vector<CTriangle::SPoint3D> *CAStar::getRoute3DPoints() const{
    return this->pRoute3DPoints;
}

const std::vector<CTriangle::SPoint3D> *CAStar::getExpansed3DPoints() const {
    auto expansed = new std::vector<CTriangle::SPoint3D>;
    while (!explorationAgenda->IsEmpty()) {
        expansed->push_back(*this->get3DPoint(this->explorationAgenda->VisitTop()));

    }
    std::cout << std::endl;
    return expansed;
}






