/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#ifndef KSPWLO_HPP
#define KSPWLO_HPP

#include <iostream>
#include <vector>
#include <unordered_map>

#include "../model/graph.hpp"
#include "../tools/tools.hpp"

using namespace std;

// Class for the Label which contains an overlap vector
class OlLabel : public Label {
public:
	//float ratio;
	vector<double> overlapList;
    unsigned int overlapForK;
	
	OlLabel(NodeID node_id, int length, vector<double> &overlapList, int overlapForK) : Label(node_id,length) {
        this->overlapList = overlapList;
        this->overlapForK = overlapForK;
        this->previous = NULL;
    };
	
	OlLabel(NodeID node_id, int length, vector<double> &overlapList, int overlapForK, OlLabel* previous) : Label(node_id,length,previous) {
        this->overlapList = overlapList;
        this->overlapForK = overlapForK;
    };
    
    OlLabel(NodeID node_id, int length, int fDist, vector<double> &overlapList, int overlapForK) : Label(node_id,length,fDist) {
        this->overlapList = overlapList;
        this->overlapForK = overlapForK;
        this->previous = NULL;
    };
	
	OlLabel(NodeID node_id, int length, int fDist, vector<double> &overlapList, int overlapForK, OlLabel* previous) : Label(node_id,length,fDist,previous) {
        this->overlapList = overlapList;
        this->overlapForK = overlapForK;
    };
    
};

/*
 *	SkylineContainer class stores the skyline in each node.
 * 	Responsible for executing the domination check.
 */

class SkylineContainer {
	public:
		unordered_map<int,vector<OlLabel*>> container; // Consider replacing the unordered_map with a vector
		void insert(OlLabel*);
		bool contains(int);
		vector<OlLabel*> get(int);
		bool dominates(OlLabel*);
		unsigned long contentsSize();
};

class AstarComparator2 {
    bool reverse;
public:
    AstarComparator2(const bool& revparam=false) {
    	reverse=revparam;
    }
    bool operator() (const OlLabel* lhs, const OlLabel* rhs) const     {
        if(lhs->lowerBound > rhs->lowerBound)
        	return true;
        else if(lhs->lowerBound < rhs->lowerBound)
        	return false;
    	else {
    		float minLhs = 1;
    		for(unsigned int i=0;i<lhs->overlapList.size();i++) {
    			if(lhs->overlapList[i] < minLhs)
    				minLhs = lhs->overlapList[i];
    		}
    		
    		float minRhs = 1;
    		for(unsigned int i=0;i<rhs->overlapList.size();i++) {
    			if(rhs->overlapList[i] < minRhs)
    				minRhs = rhs->overlapList[i];
    		}
    		
    		return minLhs > minRhs;
    	}
    }
};

typedef priority_queue<OlLabel*,std::vector<OlLabel*>,AstarComparator2> PriorityQueueAS2;

// Declarations of exact algorithms
vector<Path> onepass(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);
vector<Path> multipass(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);

// Declarations of performance-oriented heuristic algorithms
vector<Path> svp_plus(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);
vector<Path> onepass_plus(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);
vector<Path> esx(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);

// Declarations of completeness-oriented heuristic algorithms
pair<vector<Path>,double> esx_complete(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);
pair<vector<Path>,double> svp_plus_complete(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta);

// Supporting function for the completeness-oriented heuristic algorithms
pair<vector<Path>,double> completeness_function(RoadNetwork *rN, vector<Path> inputPaths, unsigned int k, double theta);
#endif
