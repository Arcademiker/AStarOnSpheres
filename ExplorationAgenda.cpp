//
// Created by Thomas on 19.04.18.
//

#include "ExplorationAgenda.h"

ExplorationAgenda::ExplorationAgenda() {
    this->qPriorityQ = new minK_PriorityQ();
}


ExplorationAgenda::~ExplorationAgenda() {
    delete this->qPriorityQ;
}

void ExplorationAgenda::Add(float nK, int nV) {

    this->qPriorityQ->push(std::pair<float,int>(nK,nV));
}

// attention! for efficiency purposes it is not tested in VisitTop() whether the queue is empty before pop()
int ExplorationAgenda::VisitTop() {
    int nV=this->qPriorityQ->top().second;
    this->qPriorityQ->pop();
    return nV;
    //todo throw empty exception
}


bool ExplorationAgenda::IsEmpty() {
    return this->qPriorityQ->empty();
}