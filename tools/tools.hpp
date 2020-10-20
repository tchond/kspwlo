/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include "../model/graph.hpp"

using namespace std;

class Label {
public:
    NodeID node_id;
    int length;
    int lowerBound;
    Label* previous;
    
    Label(NodeID node_id, double length) {
        this->node_id = node_id;
        this->length = length;
        this->previous = NULL;
        this->lowerBound = 0;
    };
    
    Label(NodeID node_id, double length, Label* previous) {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = 0;
    };
    
    Label(NodeID node_id, double length, int lowerBound) {
        this->node_id = node_id;
        this->length = length;
        this->previous = NULL;
        this->lowerBound = lowerBound;
    };
    
    Label(NodeID node_id, double length, int lowerBound, Label* previous) {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = lowerBound;
    };
};

class MyComparator {
    bool reverse;
public:
    MyComparator(const bool& revparam=false) {
    	reverse=revparam;
    }
    bool operator() (const Label* lhs, const Label* rhs) const {
        return (lhs->length>rhs->length);
    }
};

class AstarComparator {
    bool reverse;
public:
    AstarComparator(const bool& revparam=false) {
    	reverse=revparam;
    }
    bool operator() (const Label* lhs, const Label* rhs) const     {
        return (lhs->lowerBound>rhs->lowerBound);
    }
};

typedef priority_queue<Label*,std::vector<Label*>,MyComparator> PriorityQueue;
typedef priority_queue<Label*,std::vector<Label*>,AstarComparator> PriorityQueueAS;

pair<Path,vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeID source, NodeID target);
int dijkstra_dist_del(RoadNetwork *rN, NodeID source, NodeID target);
Path astar_limited(RoadNetwork *rN, NodeID source, NodeID target, vector<int> &bounds, unordered_set<Edge, boost::hash<Edge>> &deletedEdges);

#endif