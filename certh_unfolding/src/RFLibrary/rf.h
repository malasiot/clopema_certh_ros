#pragma once

#include <vector>
#include <time.h>
#include <cv.h>
#include <fstream>



using namespace std;

class RF{

public:
	
    struct vec{
        double x,y;
    };

    struct trainImage{
        int fpos, w, h;
        bool symmetric;
    };

    typedef vector< vector<trainImage> > TrainSet;

	RF( int ntrees, int max_depth, int min_samples, int num_tests ) : ntrees(ntrees), max_depth(max_depth), min_samples(min_samples), num_tests(num_tests){
		vTrees.resize(ntrees);
		tmpbuffer = new float[640*480];
		srand(time(NULL));		
	}

	RF( string folder_name ) {
		loadForestFromFolder(folder_name);	
		tmpbuffer = new float[640*480];
		srand(time(NULL));		
	}

	~RF(){
		delete tmpbuffer;
		//Detele trees

	}

    //void train(string s);
	int RFDetect(cv::Mat& cvImg, vector<double>& res_tab);

	int getNLabels(){
		return nlabels;
	}


private:		
	
	struct Test{
		vec u,v;
		int threshold;
	};
	
	struct treeNode{
		bool leaf;
		Test test;
		treeNode *left, *right;				
		int depth;
		vector<double> pfg;
	};

    void crop_init( vector < vector < int > > & inImg, trainImage* tImg, int xgrasp, int ygrasp);
	void getTrainSet(TrainSet& trainSet, string s);
	void growNode(treeNode *node, TrainSet& trainSet, int depth);
	bool optimizeTest(Test& bestTest, TrainSet& bestSetA, TrainSet& bestSetB, TrainSet& trainSet, int depth);
	bool generateTest(Test& test, TrainSet& trainSet, int thres_iter);
    void splitSet( TrainSet& SetA, TrainSet& SetB, Test& test, TrainSet& trainSet, vector < vector <float> > & storedValues, int& samplesA, int& samplesB);
	double measureSplit(TrainSet& SetA, TrainSet& SetB);
	void makeLeaf(treeNode *node, TrainSet& trainSet);	
	void saveTreeNode(treeNode *n, ofstream& f);
	void loadForestFromFolder(string folder_name);
	void loadNodeFromFile(treeNode *n, ifstream& f);
	void deleteTreeNode(treeNode *n);
	void GetBoundingBox(cv::Mat& mat, cv::Rect& rect);

	treeNode* getLeaf(cv::Mat& img, treeNode* n);

    //void trainTrees(vector<int>& treesNo, TrainSet& trainSet);

	vector<treeNode*> vTrees;
	vector< vector<trainImage*> > trainImages;
	int ntrees, nlabels, max_depth, min_samples, num_tests;				
    //vector<ifstream> trainFiles;
	float *tmpbuffer;
};
