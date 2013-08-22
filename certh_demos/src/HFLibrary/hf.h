#pragma once

#include <vector>
#include <time.h>
#include <cv.h>
#include <fstream>

using namespace std;

struct vec{
	double x,y;
};



struct trainImage{
	int fpos, w, h;
	double xgrasp, ygrasp;
	bool symmetric;	
};

typedef vector< vector<trainImage> > TrainSet;

class HF{

public:
	
	HF( int ntrees, int max_depth, int min_samples, int num_tests ) : ntrees(ntrees), max_depth(max_depth), min_samples(min_samples), num_tests(num_tests){
		vTrees.resize(ntrees);
		tmpbuffer = new float[640*480];
		srand(time(NULL));			
	}

	HF( string folder_name ) {
		loadForestFromFolder(folder_name);
		tmpbuffer = new float[640*480];
		srand(time(NULL));						
	}

	~HF(){
		delete tmpbuffer;
		//Detele trees

	}

	void train(string s);	
	int houghDetect(cv::Mat& dImg, cv::Mat& hImg, cv::Rect& hrect);		

private:		
	
	struct Test{
		vec u,v;
		int threshold;
	};
	
	struct treeNode{
		bool leaf;
		Test test;
		treeNode *left, *right;
		vector< vec > graspPList;		
		int sumW;
		int depth;
		double pfg;
	};

    void crop_init(vector<vector<int> >& inImg, trainImage* tImg, int xgrasp, int ygrasp);
	void getTrainSet(TrainSet& trainSet, string s);
	void growNode(treeNode *node, TrainSet& trainSet, int depth, double pnratio);
	bool optimizeTest(Test& bestTest, TrainSet& bestSetA, TrainSet& bestSetB, TrainSet& trainSet);
    void splitSet(TrainSet& SetA, TrainSet& SetB, Test& test, TrainSet& trainSet, vector<vector<float> > storedValues, int& samplesA, int& samplesB);
	double measureSplit(TrainSet& SetA, TrainSet& SetB, int measure_mode);
	void makeLeaf(treeNode *node, TrainSet& trainSet, double pnratio);	
	void saveTreeNode(treeNode *n, ofstream& f);
	void loadForestFromFolder(string folder_name);
	void loadNodeFromFile(treeNode *n, ifstream& f);
	void deleteTreeNode(treeNode *n);
	void GetBoundingBox(cv::Mat& mat, cv::Rect& rect);
	
	treeNode* getLeaf(cv::Mat& img, treeNode* n);

	void trainTrees(vector<int>& treesNo, TrainSet& trainSet);

	void kmeans(vector<vec>& orig, vector<vec>& centers, vector<int>& weights, int ncenters);

	vector<treeNode*> vTrees;
	vector< vector<trainImage*> > trainImages;
	int ntrees, max_depth, min_samples, num_tests;		
	ifstream trainFile;
	float *tmpbuffer;
};
