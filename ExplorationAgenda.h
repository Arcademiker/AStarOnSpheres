//
// Created by Thomas on 19.04.18.
//

#pragma once

#include <queue>

/* ExplorationAgenda is a Priority Queue with key value pairs. The pair with the smallest key is on top.
 * key is the float value for the f(n) score of A*.
 * the value of the Priority Queue is the VertexID.
 * why priority queue, why not multimap?
 *  the std-priority_queue is faster than the std-multimap, because:
 *   The typical underlying implementation of a multimap is a balanced red/black tree.
 *   Repeated element removals from one of the extreme ends of a multimap has a good chance of skewing the tree,
 *   requiring frequent rebalancing of the entire tree. This is going to be an expensive operation. */
class ExplorationAgenda {
private:
    // define (Key (nK), Value (nV)) Priority Queue (PriorityQ) ordered by Key (minimal Key on top):
    typedef std::priority_queue<std::pair<float,int>,std::vector<std::pair<float,int>>,std::greater<std::pair<float,int>>> minK_PriorityQ;
    minK_PriorityQ* qPriorityQ;  //q prefix indicates queue data type
public:
    // member functions for simpler std::priority_queue handling:
    ExplorationAgenda();
    ~ExplorationAgenda();
    void Add(float nK, int nV);
    int VisitTop();
    bool IsEmpty();
};


