/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "graph.hpp"

RoadNetwork::RoadNetwork(const char *filename) {
    
    FILE *fp;
    NodeID lnode, rnode, tmp;
    int w;
    char c;

    fp = fopen(filename, "r");
    fscanf(fp, "%c\n", &c);
    fscanf(fp, "%u %u\n", &this->numNodes, &this->numEdges);
    this->adjListOut = vector<EdgeList>(this->numNodes);
    this->adjListInc = vector<EdgeList>(this->numNodes);
    
    while (fscanf(fp, "%u %u %d %u\n", &lnode, &rnode, &w, &tmp) != EOF) {
        this->adjListOut[lnode].insert(make_pair(rnode, w));
        this->adjListInc[rnode].insert(make_pair(lnode, w));
    }
    fclose(fp);
}

int RoadNetwork::getEdgeWeight(NodeID lnode, NodeID rnode) {
    return this->adjListOut[lnode][rnode];
}

RoadNetwork::~RoadNetwork() {
    this->adjListOut.clear();
   	this->adjListInc.clear();
}

bool operator==(const Edge& le, const Edge& re) {
    return (le.first == re.first && le.second == re.second) || (le.second == re.first && le.first == re.second);
}

bool Path::containsEdge(Edge e) {
    bool res = false;
    
	for(unsigned int i=0;i<this->nodes.size()-1;i++) {
		if(this->nodes[i] == e.first && this->nodes[i+1] == e.second) {
			res = true;
			break;
		}
	}
	
    return res;
}

double Path::overlap_ratio(RoadNetwork *rN, Path &path2) {
	double sharedLength = 0;
	
	for(unsigned int i=0;i<path2.nodes.size()-1;i++) {
		Edge e = make_pair(path2.nodes[i],path2.nodes[i+1]);
		if(this->containsEdge(e))
			sharedLength += rN->getEdgeWeight(path2.nodes[i],path2.nodes[i+1]);
	}

    return sharedLength/path2.length;
}

bool operator==(const Path& lp, const Path& rp) {
	if(lp.length != rp.length || lp.nodes.size() != rp.nodes.size())
		return false;
	for(int i=0;i<lp.nodes.size();i++) {
		if(lp.nodes[i] != rp.nodes[i])
			return false;
	}
    return true;
}