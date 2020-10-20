#include "kspwlo.hpp"

pair<vector<Path>,double> completeness_function(RoadNetwork *rN, vector<Path> inputPaths, unsigned int k, double theta) {
	vector<Path> resPaths;
	vector<int> resPathsIndices;
	set<double> globalMins;
	
	unordered_map<int,vector<double>> similarityMatrix;
	similarityMatrix.insert(make_pair(0,vector<double>(inputPaths.size(),-1)));
	double globalMin = theta;
	int tpIndex = 0;
	while(resPaths.size() < k && globalMin < 1) {
		double iterationTheta = globalMin;
		
		vector<Path> tempVector;
		tempVector.insert(tempVector.end(),resPaths.begin(),resPaths.end());
		vector<int> tempVectorIndices;
		tempVectorIndices.insert(tempVectorIndices.end(),resPathsIndices.begin(),resPathsIndices.end());
		
		resPaths.clear();
		resPathsIndices.clear();
		
		for(int i =0;i<tempVector.size();i++) {
			if(tempVectorIndices[i] < tpIndex) {
				resPaths.push_back(tempVector[i]);
				resPathsIndices.push_back(tempVectorIndices[i]);
			}
			else {
				break;
			}
		}
		
		globalMin = 1;
			
		for(unsigned int i=0;i<inputPaths.size();i++) {
			double localMax = -1;
			bool check = true;
			for(unsigned int k=0;k<resPaths.size();k++) {

				if(similarityMatrix[resPathsIndices[k]][i] == -1) {
					similarityMatrix[resPathsIndices[k]][i] = inputPaths[i].overlap_ratio(rN,resPaths[k]);
				}
				double currentTheta = similarityMatrix[resPathsIndices[k]][i];
				if(currentTheta > iterationTheta)
					check = false;					
				if(currentTheta > localMax)
					localMax = currentTheta;
			}
			if(resPaths.size() > 0 && resPaths.back().length > inputPaths[i].length) {
				if(localMax < globalMin) {
					globalMin = localMax;
					tpIndex = i;
				}
				continue;
			}
			
			if(check) {
				resPaths.push_back(inputPaths[i]);
				resPathsIndices.push_back(i);
				
				similarityMatrix.insert(make_pair(i,vector<double>(inputPaths.size(),-1)));
				
				if(resPaths.size() == k) {
					return make_pair(resPaths,iterationTheta);
				}
			}
			else {
				if(localMax < globalMin) {
					globalMin = localMax;
					tpIndex = i;
				}
			}	
    	}    	
    	globalMin = (int)((globalMin * 100000.0)+1)/100000.0; // globalMin is the next theta.
    }
	return make_pair(resPaths,globalMin);
}