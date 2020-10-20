/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "tools.hpp"

/*
 *
 *	dijkstra_path_and_bounds(RoadNetwork, NodeID, NodeID, vector<int>, unordered_set<Edge,boost::hash<Edge>>)
 *	-----
 *	This algorithm is essentially an all-to-one Dijkstra and returns at the same
 *	time the shortest path from the source to the target AND the distances of
 *	all nodes to the target. This algorithm is used way to compute the shortest
 *	path along with exact lower bounds for OnePas, MultiPass and OnePass+.
 *
 */

pair<Path,vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeID source, NodeID target) {
	unsigned int count = 0;
    PriorityQueue queue;
    Path resPath;
    int newLength = 0;
    EdgeList::iterator iterAdj;
    vector<int> distances(rN->numNodes, INT_MAX);
    vector<bool> visited(rN->numNodes);
    Label *targetLabel=NULL;
    distances[target]=0;
    vector<Label*> allCreatedLabels;
    Label* srcLabel = new Label(target, newLength);
    queue.push(srcLabel);
    allCreatedLabels.push_back(srcLabel);
    
    while (!queue.empty()) {
        Label* curLabel = queue.top();
        queue.pop();
        
        if (visited[curLabel->node_id])
            continue;
        
        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;
        
        if (curLabel->node_id == source) { // Destination has been found
        	targetLabel = curLabel;
            resPath.length = curLabel->length;
            while(targetLabel != NULL) {
    			resPath.nodes.push_back(targetLabel->node_id);
    			targetLabel = targetLabel->previous;
    		}
        }
        
        if(++count == rN->numNodes)
        	break;
        
        else { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInc[curLabel->node_id].begin(); iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++) {
                newLength = curLabel->length + iterAdj->second;
                Label* newPrevious = curLabel;
                if (distances[iterAdj->first] > newLength) {
                	Label* label = new Label(iterAdj->first, newLength, newPrevious);
                	allCreatedLabels.push_back(label);
                    queue.push(label);
                }
            }
        }
    }
    for(unsigned int i=0;i<allCreatedLabels.size();i++)
    	delete allCreatedLabels[i];
    
    return make_pair(resPath, distances);
}

int dijkstra_dist_del(RoadNetwork *rN, NodeID source, NodeID target) {
    PriorityQueue queue;
    int newLength = 0;
    int resDist = -1;
    EdgeList::iterator iterAdj;
    vector<int> distances(rN->numNodes, INT_MAX);
    vector<bool> visited(rN->numNodes);
    Label *targetLabel=NULL;
    distances[target]=0;
    vector<Label*> allCreatedLabels;
    Label* srcLabel = new Label(target, newLength);
    queue.push(srcLabel);
    allCreatedLabels.push_back(srcLabel);
    
    while (!queue.empty()) {
        Label* curLabel = queue.top();
        queue.pop();
        
        if (visited[curLabel->node_id])
            continue;
        
        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;
        
        if (curLabel->node_id == source) { // Destination has been found
        	targetLabel = curLabel;
            resDist = curLabel->length;
            break;
        }
        
        else { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInc[curLabel->node_id].begin(); iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++) {
                if((source == curLabel->node_id && target == iterAdj->first) || (target == curLabel->node_id && source == iterAdj->first))
                	continue;
                newLength = curLabel->length + iterAdj->second;
                Label* newPrevious = curLabel;
                if (distances[iterAdj->first] > newLength) {
                	Label* label = new Label(iterAdj->first, newLength, newPrevious);
                	allCreatedLabels.push_back(label);
                    queue.push(label);
                }
            }
        }
    }
    for(unsigned int i=0;i<allCreatedLabels.size();i++)
    	delete allCreatedLabels[i];
    
    return resDist;
}