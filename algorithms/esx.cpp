/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

#include "kspwlo.hpp"

typedef priority_queue<pair<int,Edge>> PathEdges;

struct PathComparator {
    inline bool operator() (const Path &p1, const Path &p2)     {
        return (p1.length < p2.length);
    }
} pComp2;

int compute_priority(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges);
int compute_priority_maxpt(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges);
int compute_priority_minpt(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges);
int compute_priority_maxw(RoadNetwork *rN, Edge &e);
int compute_priority_minw(RoadNetwork *rN, Edge &e);
int compute_priority_maxstr(RoadNetwork *rN, Edge &e);
int compute_priority_minstr(RoadNetwork *rN, Edge &e);

/*
 *
 *	Implementation of the ESX algorithm
 *
 */

vector<Path> esx(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta) {
    
	vector<Path> resPaths;
	EdgeList::iterator iterAdj;
	
	pair<Path,vector<int>> resDijkstra= dijkstra_path_and_bounds(rN,source,target);
	resPaths.push_back(resDijkstra.first);
	
	if(k==1)
		return resPaths;

	vector<PathEdges> pathEdges(k);
	
	unordered_set<Edge,boost::hash<Edge>> untouchableEdges;
	unordered_set<Edge,boost::hash<Edge>> deletedEdges;

	for(unsigned int j=0;j<resPaths[0].nodes.size()-1;j++) {
		Edge e(resPaths[0].nodes[j],resPaths[0].nodes[j+1]);
		pathEdges[0].push(make_pair(compute_priority(rN,e,resDijkstra.second,deletedEdges),e));
	}
	
	vector<double> overlaps(k,0);
	overlaps[0] = 1;	
	
	
	bool possible = true;
	
	//cout << "Starting loop" << endl;
	
	while(resPaths.size()<k && possible) {
	
		NodeID maxOlIdx = 0;
		double olRatio = 1;
		//cout << "Starting inner loop" << endl;
		while(olRatio > theta) {
			
			double tempOlRatio = 0;
			for(unsigned int i=0;i<resPaths.size();i++) {
				if(overlaps[i] > tempOlRatio) {
					tempOlRatio = overlaps[i];
					maxOlIdx = i;
				}
			}
			olRatio = tempOlRatio;
			//cout << "Overlap ratio checked" << endl;
			// Checking is finding a result is feasible			
			bool peCheck = true;
			for(unsigned int m=0;m<overlaps.size();m++) {
				if(overlaps[m] > 0) {
					peCheck = false;
					break;
				}
			}
			if(peCheck) {
				possible = false;
				break;
			}
			//cout << "Feasible result checked" << endl;
			
			Edge e = pathEdges[maxOlIdx].top().second;
			if(untouchableEdges.find(e) != untouchableEdges.end()) {
				pathEdges[maxOlIdx].pop();
				if(pathEdges[maxOlIdx].size() == 0) 
					overlaps[maxOlIdx] = 0;
				continue;
			}
			else {
				deletedEdges.insert(e);
			}
			
			//cout << "Untouchable checked" << endl;	
			
			Path newP = astar_limited(rN,source,target,resDijkstra.second,deletedEdges);
			//cout << "A-start done" << endl;	
			
			if(newP.nodes.size() == 0) { // If astar_limited did not find a path
				unsigned int size = deletedEdges.size();
				deletedEdges.erase(e);
				assert(deletedEdges.size()+1 == size);
				untouchableEdges.insert(e);
				continue;
			}
			//cout << "Path was found " << newP.nodes.size() << endl;	
			// in cases there are no more edges to remove we set overlap to zero to avoid choosing from the same path again.
			// a path the overlap of which is zero can never be chosen to remove its edges.
			pathEdges[maxOlIdx].pop();
			if(pathEdges[maxOlIdx].size() == 0) 
				overlaps[maxOlIdx] = 0;
			else
				overlaps[maxOlIdx] = newP.overlap_ratio(rN,resPaths[maxOlIdx]);
			//cout << "Overlap updated" << endl;	
			// Checking if the resulting path is valid
			bool check = false; // true if it violates theta
			for(unsigned int j=0;j<resPaths.size();j++) {
				if(newP.overlap_ratio(rN,resPaths[j]) > theta) {
					check = true;					
					break;
				}
			}
			if(!check) {
				resPaths.push_back(newP);
				overlaps[resPaths.size()-1] = 1;
				for(unsigned int j=0;j<resPaths.back().nodes.size()-1;j++) {
					Edge e(resPaths.back().nodes[j],resPaths.back().nodes[j+1]);
					double sim = compute_priority(rN,e,resDijkstra.second,deletedEdges);
					pathEdges[resPaths.size()-1].push(make_pair(sim,e));
				}
				break;
			}
		}
		//cout << "Ending inner loop" << endl;
	}	
	//cout << "Ending loop" << endl;
	return resPaths;
}

/*
 *
 *	Implementation of the ESX-C algorithm
 *
 */

pair<vector<Path>,double> esx_complete(RoadNetwork *rN, NodeID source, NodeID target, unsigned int k, double theta) {
    
	vector<Path> candidatePaths;
	vector<Path> resPaths;
	set<double> myThetas;
	
	EdgeList::iterator iterAdj;
	
	pair<Path,vector<int>> resDijkstra= dijkstra_path_and_bounds(rN,source,target);
	resPaths.push_back(resDijkstra.first);
	candidatePaths.push_back(resDijkstra.first);
	double globalMin = 1;
	if(k==1)
		return make_pair(resPaths,theta);
	else {

		vector<PathEdges> pathEdges(k);
	
		unordered_set<Edge,boost::hash<Edge>> untouchableEdges;
		unordered_set<Edge,boost::hash<Edge>> deletedEdges;
	
		for(unsigned int j=0;j<resPaths[0].nodes.size()-1;j++) {
			Edge e(resPaths[0].nodes[j],resPaths[0].nodes[j+1]);
			pathEdges[0].push(make_pair(compute_priority(rN,e,resDijkstra.second,deletedEdges),e));
		}
	
		vector<double> overlaps(k,0);
		overlaps[0] = 1;	
	
		bool possible = true;
	
		//cout << "Starting loop" << endl;
		
		while(resPaths.size()<k && possible) {
	
			NodeID maxOlIdx = 0;
			double olRatio = 1;
			//cout << "Starting inner loop" << endl;
			while(olRatio > theta) {
			
				double tempOlRatio = 0;
				for(unsigned int i=0;i<resPaths.size();i++) {
					if(overlaps[i] > tempOlRatio) {
						tempOlRatio = overlaps[i];
						maxOlIdx = i;
					}
				}
				olRatio = tempOlRatio;
				//cout << "Overlap ratio checked" << endl;
				// Checking is finding a result is feasible			
				bool peCheck = true;
				for(unsigned int m=0;m<overlaps.size();m++) {
					if(overlaps[m] > 0) {
						peCheck = false;
						break;
					}
				}
				if(peCheck) {
					possible = false;
					break;
				}
			
				Edge e = pathEdges[maxOlIdx].top().second;
				if(untouchableEdges.find(e) != untouchableEdges.end()) {
					pathEdges[maxOlIdx].pop();
					if(pathEdges[maxOlIdx].size() == 0) 
						overlaps[maxOlIdx] = 0;
					continue;
				}
				else {
					deletedEdges.insert(e);
				}
						
				Path newP = astar_limited(rN,source,target,resDijkstra.second,deletedEdges);
			
				if(newP.nodes.size() == 0) { // If astar_limited did not find a path
					unsigned int size = deletedEdges.size();
					deletedEdges.erase(e);
					assert(deletedEdges.size()+1 == size);
					untouchableEdges.insert(e);
					continue;
				}
				
				candidatePaths.push_back(newP);
							
				//cout << "Path was found " << newP.nodes.size() << endl;	
				// in cases there are no more edges to remove we set overlap to zero to avoid choosing from the same path again.
				// a path the overlap of which is zero can never be chosen to remove its edges.
				pathEdges[maxOlIdx].pop();
				if(pathEdges[maxOlIdx].size() == 0) 
					overlaps[maxOlIdx] = 0;
				else
					overlaps[maxOlIdx] = newP.overlap_ratio(rN,resPaths[maxOlIdx]);
				//cout << "Overlap updated" << endl;
				
				double localMax = -1;
				bool check = true;
				for(unsigned int j=0;j<resPaths.size();j++) {
					double currentTheta = newP.overlap_ratio(rN,resPaths[j]);
					if(currentTheta > theta) {
						check = false;					
					}
					if(currentTheta >= localMax)
						localMax = currentTheta;
				}
				if(check) {
					resPaths.push_back(newP);
					if(resPaths.size() == k)
						break;
					overlaps[resPaths.size()-1] = 1;
					for(unsigned int j=0;j<resPaths.back().nodes.size()-1;j++) {
						Edge e(resPaths.back().nodes[j],resPaths.back().nodes[j+1]);
						double sim = compute_priority(rN,e,resDijkstra.second,deletedEdges);
						pathEdges[resPaths.size()-1].push(make_pair(sim,e));
					}
					break;
				}
				else {
					//cout << "localMax = " << localMax << endl;
					if(localMax < globalMin) {
						globalMin = localMax;
					}
				}
			}
			//cout << "Ending inner loop" << endl;
		}	
		//cout << "Ending loop" << endl;
	}
	
	pair<vector<Path>,double> resPair = make_pair(resPaths,theta);
	if(resPaths.size() < k) {
		sort(candidatePaths.begin(),candidatePaths.end(),pComp2);
		candidatePaths.erase( unique( candidatePaths.begin(), candidatePaths.end() ), candidatePaths.end() );
		if(candidatePaths.size() < k) {
			vector<Path> ksp = onepass(rN,source,target,k,0.999); // Can be replaced with a better k-shortest path algorithm.
			for(unsigned int i=1;i<ksp.size();i++) {
				candidatePaths.push_back(ksp[i]);
			}
			sort(candidatePaths.begin(),candidatePaths.end(),pComp2);
			candidatePaths.erase( unique( candidatePaths.begin(), candidatePaths.end() ), candidatePaths.end() );
			return completeness_function(rN,candidatePaths,k,theta);
		}
		else {
			return completeness_function(rN,candidatePaths,k,globalMin);
		}
	}	
	return make_pair(resPaths,theta);
}

/*
	Let Sources be all the nodes which are the starting points of all incoming edges to the source of the given edge.
	Let Target be all the nodes which are the ending points of all outgoing edges from the target of the given edge.
	This function returns the number of shortest paths from some source to some target that contain the given edge. 
*/

int compute_paths_through(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges) {
	int strength2 = 0;
	EdgeList::iterator iterAdj;
	vector<NodeID> sources, targets;
	for (iterAdj = rN->adjListInc[e.first].begin(); iterAdj != rN->adjListInc[e.first].end(); iterAdj++) {
		if(iterAdj->first != e.second)
			sources.push_back(iterAdj->first);
	}
	for (iterAdj = rN->adjListOut[e.second].begin(); iterAdj != rN->adjListOut[e.second].end(); iterAdj++) {
		if(iterAdj->first != e.first)	
			targets.push_back(iterAdj->first);
	}
	for(unsigned int m=0;m<sources.size();m++) {
		for(unsigned int n=0;n<targets.size();n++) {
			Path tempP = astar_limited(rN,sources[m],targets[n],bounds,deletedEdges);
			if(tempP.nodes.size() > 0 && tempP.containsEdge(e))
				strength2++;
			}
	}
	
	return strength2;
}

/*
	This is the function that computes the priority of a given edge.
*/

int compute_priority(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges) {
	return compute_priority_minw(rN,e); // Paths through
}

/*
	The following functions implement the different strategies for identifying the next edge to be removed.
*/

int compute_priority_maxpt(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges) {
	int strength2 = 0;
	EdgeList::iterator iterAdj;
	vector<NodeID> sources, targets;
	for (iterAdj = rN->adjListInc[e.first].begin(); iterAdj != rN->adjListInc[e.first].end(); iterAdj++) {
		if(iterAdj->first != e.second)
			sources.push_back(iterAdj->first);
	}
	for (iterAdj = rN->adjListOut[e.second].begin(); iterAdj != rN->adjListOut[e.second].end(); iterAdj++) {
		if(iterAdj->first != e.first)	
			targets.push_back(iterAdj->first);
	}
	for(unsigned int m=0;m<sources.size();m++) {
		for(unsigned int n=0;n<targets.size();n++) {
			Path tempP = astar_limited(rN,sources[m],targets[n],bounds,deletedEdges);
			if(tempP.nodes.size() > 0 && tempP.containsEdge(e))
				strength2++;
			}
	}
	
	return strength2;
}

int compute_priority_minpt(RoadNetwork *rN, Edge &e, vector<int> &bounds, unordered_set<Edge,boost::hash<Edge>> &deletedEdges) {
	return INT_MAX-compute_priority_maxpt(rN,e,bounds,deletedEdges);
}

int compute_priority_maxw(RoadNetwork *rN, Edge &e) {
	return rN->getEdgeWeight(e.first,e.second); // Paths through
}

int compute_priority_minw(RoadNetwork *rN, Edge &e) {
	return INT_MAX-rN->getEdgeWeight(e.first,e.second); // Paths through
}

int compute_priority_maxstr(RoadNetwork *rN, Edge &e) {
	int dist = dijkstra_dist_del(rN,e.first,e.second);
	int weight = rN->getEdgeWeight(e.first,e.second);
	return abs(dist-weight);
}

int compute_priority_minstr(RoadNetwork *rN, Edge &e) {
	return INT_MAX-compute_priority_maxstr(rN,e); // Paths through
}
