#include "hf.h"
#include <time.h>
#include <iostream>
#include <fstream>
#include <highgui.h>
#include <vector>
#include <math.h>

//#include <boost/thread.hpp>

using namespace std;


///////////////----------------------------Training functions---------------------------------/////////////////

void HF::getTrainSet(TrainSet& trainSet, string s){				
			
    trainFile.open((s + ".dps").c_str(), ios::in | ios::binary);
	trainSet.resize(2);		//2 labels
    ifstream configFile((s + ".txt").c_str());
		
	int fpos, w, h, label;
	double xgrasp, ygrasp;
	while(configFile >> fpos >> w >> h >> xgrasp >> ygrasp){
		label = (xgrasp==-1)? 0:1;	//non visible class->0, visible class->1
		trainImage ti1,ti2;
		ti1.fpos = fpos;
		ti1.w = w;
		ti1.h = h;
		ti1.xgrasp = xgrasp;
		ti1.ygrasp = ygrasp;
		ti1.symmetric = false;
		trainSet[label].push_back(ti1);
		ti2.fpos = fpos;
		ti2.w = w;
		ti2.h = h;
		ti2.xgrasp = (label==1)? 1-xgrasp:xgrasp;
		ti2.ygrasp = ygrasp;
		ti2.symmetric = true;
		trainSet[label].push_back(ti2);
	}					
	
	configFile.close();
}

void HF::splitSet(TrainSet& SetA, TrainSet& SetB, Test& test, TrainSet& trainSet, vector<vector<float> > storedValues, int& samplesA, int& samplesB){
	
	SetA.resize(trainSet.size());
	SetB.resize(trainSet.size());
	samplesA = 0;
	samplesB = 0;
    for(int i=0; i<trainSet.size(); ++i){
		SetA[i].resize(0);
		SetB[i].resize(0);
		for(int j=0; j<trainSet[i].size(); ++j){					
			float testval = storedValues[i][j];
			if(testval<=(float)test.threshold){
				SetA[i].push_back(trainSet[i][j]);
				samplesA++;
			}
			else{
				SetB[i].push_back(trainSet[i][j]);	
				samplesB++;
			}
		}
	}
}



double HF::measureSplit(TrainSet& SetA, TrainSet& SetB, int measure_mode){	
	
	if( measure_mode == 0 ){				

		double sizeA = 0;
        for(int i=0; i<SetA.size(); ++i)
			sizeA += (double) SetA[i].size();

		double sizeB = 0;
		for(int i=0; i<SetB.size(); ++i)
			sizeB += (double) SetB[i].size();
		
		double entropyA = 0;
		for(int i=0; i<SetA.size(); ++i){
			double p = (double)SetA[i].size() / sizeA;
			if(p>0) entropyA += p*log(p);
		}		

		double entropyB = 0;
		for(int i=0; i<SetB.size(); ++i){
			double p = (double)SetB[i].size() / sizeB;
			if(p>0) entropyB += p*log(p);
		}

		return (sizeA*entropyA+sizeB*entropyB)/(sizeA+sizeB); 

	}else{		
		
		double meanAx = 0;
		double meanAy = 0;		
		double samplesA = SetA[1].size();
		for(int j=0; j<SetA[1].size(); ++j){
			meanAx += SetA[1][j].xgrasp;
			meanAy += SetA[1][j].ygrasp;
		}
		meanAx /= samplesA;
		meanAy /= samplesA;

		double distA = 0;
		for(int j=0; j<SetA[1].size(); ++j){
			double x = SetA[1][j].xgrasp - meanAx; x*=100;
			double y = SetA[1][j].ygrasp - meanAy; y*=100;
			distA += x*x + y*y;
		}
		
		double meanBx = 0;
		double meanBy = 0;		
		double samplesB = SetB[1].size();
		for(int j=0; j<SetB[1].size(); ++j){
			meanBx += SetB[1][j].xgrasp;
			meanBy += SetB[1][j].ygrasp;
		}
		meanBx /= samplesB;
		meanBy /= samplesB;

		double distB = 0;		
		for(int j=0; j<SetB[1].size(); ++j){
			double x = SetB[1][j].xgrasp - meanBx; x*=100;
			double y = SetB[1][j].ygrasp - meanBy; y*=100;
			distB += x*x + y*y;
		}

		return -(distA+distB) / (samplesA+samplesB);
	}
}

bool HF::optimizeTest(Test& bestTest, TrainSet& bestSetA, TrainSet& bestSetB, TrainSet& trainSet){	

	vector<Test> tests(num_tests); 
	int measure_mode;
	vector<float> min_val(num_tests, 1000000), max_val(num_tests, -1000000);
	bool found = false;
	double bestDist = -DBL_MAX;
	
    vector<vector<vector<float> > > storedValues;
	storedValues.resize(num_tests);
	for(int i=0; i<num_tests; ++i){
		storedValues[i].resize(trainSet.size());
		for(int j=0; j<trainSet.size(); ++j)
			storedValues[i][j].resize(trainSet[j].size());
	}		
	
	//0 -> classification, 1 -> regression		
	if(trainSet[0].size() == 0)	//if all images have visible g.p., make only regression
		measure_mode = 1;
	else
		measure_mode = rand() % 2;


	for(int iteration = 0; iteration < num_tests; iteration++){
		
		//Get random positions
		tests[iteration].u.x = (float)rand()/(float)RAND_MAX;
		tests[iteration].u.y = (float)rand()/(float)RAND_MAX;
		tests[iteration].v.x = (float)rand()/(float)RAND_MAX;
		tests[iteration].v.y = (float)rand()/(float)RAND_MAX;			
	
	}
		
	for(int label=0; label<trainSet.size(); ++label){
		for(int i=0; i<trainSet[label].size(); ++i){
			trainImage ti = trainSet[label][i];
			trainFile.seekg(ti.fpos);
			trainFile.read((char*)tmpbuffer, ti.w*ti.h*sizeof(float));
			for(int iteration=0; iteration<num_tests; iteration++){
				int x1 = int( (tests[iteration].u.x * (ti.w-1)) );
				int y1 = int( (tests[iteration].u.y * (ti.h-1)) );
				int x2 = int( (tests[iteration].v.x * (ti.w-1)) );
				int y2 = int( (tests[iteration].v.y * (ti.h-1)) );
				float val1, val2;
				if(!ti.symmetric){
					val1 = tmpbuffer[y1*ti.w + x1]; 					
					val2 = tmpbuffer[y2*ti.w + x2];					
				}else{
					val1 = tmpbuffer[y1*ti.w + (ti.w-1-x1)]; 
					val2 = tmpbuffer[y2*ti.w + (ti.w-1-x2)];
				}
				if(val1 == 0) val1 = 100000;
				if(val2 == 0) val2 = 100000;
				storedValues[iteration][label][i] = val1 - val2;
				if( (val1!=100000) && (val2!=100000) && (min_val[iteration]>val1-val2) )
					min_val[iteration] = val1-val2;
				if( (val1!=100000) && (val2!=100000) && (max_val[iteration]<val1-val2) )
					max_val[iteration] = val1-val2;
			}
		}
	}	


	for(int iteration=0; iteration<num_tests; ++iteration){
		
		int thres = (int)min_val[iteration];
		int i=0;
		while( (i<3) || (thres<=(int)max_val[iteration]) ){
			if(i==0){
				tests[iteration].threshold = 50000;
				i++;
			}
			else if(i==1){
				tests[iteration].threshold = -50000;
				i++;
			}
			else if(i==2){
				tests[iteration].threshold = 0;
				i++;
			}
			else{			
				tests[iteration].threshold = thres;
				thres += 10;
			}				
			
			TrainSet SetA, SetB;
			int samplesA, samplesB;
			splitSet(SetA, SetB, tests[iteration], trainSet, storedValues[iteration], samplesA, samplesB);						

			if( (samplesA > 0) && (samplesB > 0) ){
		
				double m = measureSplit(SetA, SetB, measure_mode);
				if(m>bestDist){

					found = true;
					bestDist = m;
					bestTest = tests[iteration];
					bestSetA = SetA;
					bestSetB = SetB;

				}
			}		
		}
	}	
	return found;
}

void HF::makeLeaf(treeNode *node, TrainSet& trainSet, double pnratio){  //pnratio should normalize not equal training samples from different classes.
	node->leaf = true;
	double samples = 0;
	
	for(int i=0; i<trainSet.size(); ++i)
		samples += trainSet[i].size();		
	

	node->pfg = (double)trainSet[1].size() / ( (double)trainSet[0].size()*pnratio + (double)trainSet[1].size() );

	if(trainSet[1].size()==0)
		node->graspPList.resize(0);
	else{
		node->graspPList.resize(trainSet[1].size());		
		for(int j=0; j<trainSet[1].size(); ++j){			
			vec v;
			v.x = trainSet[1][j].xgrasp;
			v.y = trainSet[1][j].ygrasp;			
			node->graspPList[j] = v;			
		}
	}
}

void HF::growNode(treeNode *node, TrainSet& trainSet, int depth, double pnratio){	
	
	if( depth<max_depth ){
		
		node->leaf = false;
		TrainSet SetA, SetB;
		Test test;
		node->depth = depth;
		
		if( optimizeTest(test, SetA, SetB, trainSet) ){
	
			//cout << "Set A:" << endl << "Size: " << SetA[0].size() + SetA[1].size() << endl << "Size 0: " << SetA[0].size() << endl << "Size 1: " << SetA[1].size() << endl;
			//cout << "Set B:" << endl << "Size: " << SetB[0].size() + SetB[1].size() << endl << "Size 0: " << SetB[0].size() << endl << "Size 1: " << SetB[1].size() << endl;

			node->test = test;
			node->left = new treeNode;
			node->right = new treeNode;
			
			int samples = 0;
			for(int i=0; i<SetA.size(); ++i)
				samples += SetA[i].size();
			
			if( (SetA[1].size() > 0) && (samples > min_samples)	)
				growNode(node->left, SetA, depth+1, pnratio);
			else
				makeLeaf(node->left, SetA, pnratio);		
			
			samples = 0;
			for(int i=0; i<SetB.size(); ++i)
				samples += SetB[i].size();
			
			if( (SetB[1].size() > 0) && (samples > min_samples)	)
				growNode(node->right, SetB, depth+1, pnratio);
			else
				makeLeaf(node->right, SetB, pnratio);		

		}else
			makeLeaf(node, trainSet, pnratio);
	}else
		makeLeaf(node, trainSet, pnratio);
}


void HF::trainTrees(vector<int>& treesNo, TrainSet& trainSet){		

	int firstTree;
	cout << "Start tree No: ";
	cin >> firstTree;

	for(int t=0; t<treesNo.size(); ++t){							

		treeNode *tree = new treeNode;

		double pnratio = (double)trainSet[1].size() / (double)trainSet[0].size();		

		cout << "Tree: " << t+firstTree << " ";
		int t0 = time(NULL);
		growNode(tree, trainSet, 0, pnratio);
		int t1 = time(NULL);
		cout << t1-t0 << "sec";	
		
		
		stringstream s;
		s << "tree" << treesNo[t]+firstTree << ".dat";
		ofstream f(s.str().c_str(), ios::out | ios::binary);
		saveTreeNode(tree, f);
		cout << " -> saved" << endl;
		
		deleteTreeNode(tree);
		delete tree;	

		f.close();
	}	

}


void HF::train(string s){					

	TrainSet trainSet;
	getTrainSet(trainSet, s);
	
	vector<int> trees;
	trees.resize(ntrees);
	for(int i=0; i<ntrees; ++i)
		trees[i] = i;
	
	trainTrees(trees, trainSet);

	trainFile.close();
	
	//ofstream of("forest.txt");
	//of << ntrees << " " << nlabels << endl;
	//of.close();


	/*
	vector< vector<int> > trees(5);
	
	for(int i=0; i<5; ++i)
		trees[i].resize(0);

	for(int t=0; t<ntrees; ++t){																				
		trees[t%5].push_back(t);
	}	
	
	boost::thread thread1(boost::bind(&HF::trainTrees, this, trees[1]));
	

	if(ntrees>0)
		thread1.join();
	
	if(ntrees>1)
		thread2.join();
	if(ntrees>2)
		thread3.join();
	if(ntrees>3)
		thread4.join();
	if(ntrees>4)
		thread5.join();	
		*/	
}

////////////////////////////////////////////////////




////////////////---------------------------------Detection functions--------------------------------------//////////////////////

HF::treeNode* HF::getLeaf(cv::Mat& img, treeNode* n){

	if(n->leaf)
		return n;
	else{
		Test test = n->test;
		
		int d1=0;
		int d2=0;				

		int x1 = int( (test.u.x * (img.cols-1)) );
		int y1 = int( (test.u.y * (img.rows-1)) );
		int x2 = int( (test.v.x * (img.cols-1)) );
		int y2 = int( (test.v.y * (img.rows-1)) );
						
		d1 = img.at<float>(y1, x1);
		if(d1 == 0) d1 = 100000;			
			
		d2 = img.at<float>(y2, x2);
		if(d2 == 0) d2 = 100000;

		int val = d1 - d2;
		
		if(val<=test.threshold)
			return getLeaf(img, n->left);
		else
			return getLeaf(img, n->right);
	}		
}


void HF::GetBoundingBox(cv::Mat& mat, cv::Rect& rect){	
	cv::Mat rowSum;
	cv::Mat colSum;
	cv::reduce(mat, rowSum, 0, CV_REDUCE_SUM, -1);
	cv::reduce(mat, colSum, 1, CV_REDUCE_SUM, -1);
	
	int w1 = rowSum.at<float>(0);
	int k=0;
	while((w1==0)&&k<rowSum.cols)
		w1 = rowSum.at<float>(++k);
	w1 = k;

	
	k = rowSum.cols-1;
	int w2 = rowSum.at<float>(k);
	while((w2==0)&&k>w1)
		w2 = rowSum.at<float>(--k);
	w2 = k+1;

	int h1 = colSum.at<float>(0);
	k=0;
	while((h1==0)&&k<colSum.rows)
		h1 = colSum.at<float>(++k);
	h1 = k;

	k = colSum.rows-1;
	int h2 = colSum.at<float>(k);
	while((h2==0)&&k>h1)
		h2 = colSum.at<float>(--k);
	h2 = k+1;

	rect.x = w1;
	rect.y = h1;
	rect.width = w2-w1;
	rect.height = h2-h1;
}


int HF::houghDetect(cv::Mat& dImg, cv::Mat& hImg, cv::Rect& hrect){									

	cv::Rect rect;
	GetBoundingBox(dImg, rect);
	cv::Mat matIn = dImg(rect);		
	cv::Mat matOut;
	cv::bilateralFilter(matIn, matOut, -1, 13, 8, 4);
	for(int i=0; i<matOut.rows; ++i)
		for(int j=0; j<matOut.cols; ++j)
			if(matOut.at<float>(i, j)<300)
				matOut.at<float>(i, j) = 0;

	vector<treeNode*> leafs(ntrees);		
	double result=0;
	for(int i=0; i<ntrees; ++i){
		leafs[i] = getLeaf(matOut, vTrees[i]);		
		result += leafs[i]->pfg / (double)ntrees;
	}	

		
	hImg = cv::Mat::zeros(dImg.rows, dImg.cols, CV_32FC1);
	double votes_weight_sum = 0;
	double meanx=0, meany=0;
	for(int i=0; i<leafs.size(); ++i){
		for(int j=0; j<leafs[i]->graspPList.size(); ++j){
			int x = int( leafs[i]->graspPList[j].x * (double)rect.width + rect.x );
			int y = int( leafs[i]->graspPList[j].y * (double)rect.height + rect.y );
            if(matOut.at<float>(y,x) != 0){
                hImg.at<float>(y, x) += leafs[i]->pfg;
                meanx += leafs[i]->pfg * x;
                meany += leafs[i]->pfg * y;
                votes_weight_sum += leafs[i]->pfg;
            }
		}		
	}
	meanx /= votes_weight_sum;
	meany /= votes_weight_sum;

	//stdx=0;
	//stdy=0;

	for(int i=0; i<leafs.size(); ++i){
		for(int j=0; j<leafs[i]->graspPList.size(); ++j){
			int x = int( leafs[i]->graspPList[j].x * (double)rect.width + rect.x );
			int y = int( leafs[i]->graspPList[j].y * (double)rect.height + rect.y );
			//stdx += leafs[i]->pfg * pow((x - meanx), 2);
			//stdy += leafs[i]->pfg * pow((y - meany), 2);
		}
	}
	//stdx = sqrt(stdx/votes_weight_sum);
	//stdy = sqrt(stdy/votes_weight_sum);	

	cv::Mat gImg = cv::Mat(hImg);
	GaussianBlur( hImg, gImg, cv::Size( 7, 7 ), 0, 0 );

	double min,max;
	cv::Point minp, maxp;	
	cv::minMaxLoc(gImg, &min, &max, &minp, &maxp, cv::Mat());
	gImg /= max;
	hrect.x = maxp.x-10; hrect.y = maxp.y-10; hrect.width = 20; hrect.height = 20;
	//cv::TermCriteria criteria(1, 20, 0.001);
	//cv::meanShift(hImg, hrect, criteria);
		
	hImg = gImg;

	float nx = float(maxp.x-rect.x) / float(rect.width);
	float ny = float(maxp.y-rect.y) / float(rect.height);
    if(nx<0) nx = 0; if(nx>1) nx = 1;
    if(ny<0) ny = 0; if(ny>1) ny = 1;
    float xbar = floor(nx/0.125f);
	float ybar = floor(ny/0.125f);
	if(xbar > 7) xbar = 7;
	if(ybar > 7) ybar = 7;
	int cur_state = ybar * 8 + xbar;
	int cur_bar = floor(result/0.2f);
	if(cur_bar > 4) cur_bar = 4;
	int cur_obs  = cur_state * 5 + cur_bar;

	return cur_obs;													
	
}


/////////////////////////////////////////////////////////






////////////// ---------------------------------------- I/O functions --------------------------------------- ///////////////////

////k-means
void HF::kmeans(vector<vec>& orig, vector<vec>& centers, vector<int>& weights, int ncenters){
/*
	centers.resize(ncenters);
	weights.resize(ncenters);

	vector<bool> b(orig.size(), 0);	
	//get random centers
	for(int i=0; i<ncenters; ++i){
		int a = rand()%orig.size();
		while(b[a]){
			a++;
			if(a==orig.size()) a = 0;
		}
		b[a] = true;
		centers[i] = orig[a];
	}

	double maxDist = 201;
	while(maxDist>200){		
		vec z; z.x = 0; z.y = 0;
		vector<vec> new_centers(ncenters, z);
		vector<int> size_new_centers(ncenters, 0);
		for(int i=0; i<orig.size(); ++i){			
			double mind = 100000000;
			int cur_center = 0;
			for(int j=0; j<ncenters; ++j){
				double d = sqrt( pow(orig[i].x*10000 - centers[j].x*10000,2) + pow(orig[i].y*10000 - centers[j].y*10000, 2) );
				if(d < mind){
					mind = d;
					cur_center = j;
				}
			}
			new_centers[cur_center].x += orig[i].x;
			new_centers[cur_center].y += orig[i].y;
			size_new_centers[cur_center]++;
		}
		double dist = 0;
		for(int i=0; i<ncenters; ++i){
			new_centers[i].x /= (double)size_new_centers[i];
			new_centers[i].y /= (double)size_new_centers[i];

			double cdist = sqrt( pow(centers[i].x*10000 - new_centers[i].x*10000, 2) + pow(centers[i].y*10000 - new_centers[i].y*10000, 2) );
			if(cdist > dist) dist = cdist;
			centers[i] = new_centers[i];
			weights[i] = size_new_centers[i];
		}		
		maxDist = dist;
	}
*/
}


//////////// Save ////////////////
void HF::saveTreeNode(treeNode *n, ofstream& f){

	f.write((char*)&n->leaf, sizeof(bool));			
	
	if(n->leaf){
				
		f.write((char*)&n->pfg, sizeof(double));
		
		int sz = n->graspPList.size();
		f.write((char*)&sz, sizeof(int));		
		if(sz>0){
			for(int j=0; j<sz; ++j){						
				f.write((char*)&n->graspPList[j].x, sizeof(double));
				f.write((char*)&n->graspPList[j].y, sizeof(double));				
			}
		}

	}else{
		vec u = n->test.u;
		vec v = n->test.v;

		f.write((char*)&u.x, sizeof(double));
		f.write((char*)&u.y, sizeof(double));
		f.write((char*)&v.x, sizeof(double));
		f.write((char*)&v.y, sizeof(double));
		f.write((char*)&n->test.threshold, sizeof(int));
		
		saveTreeNode(n->left, f);
		saveTreeNode(n->right, f);
	}
}

void HF::deleteTreeNode(treeNode *n){

	if(n->leaf) return;

	if(!n->left->leaf){			
		n->left->graspPList.clear();
		deleteTreeNode(n->left);
	}

	delete(n->left);

	if(!n->right->leaf){
		n->right->graspPList.clear();
		deleteTreeNode(n->right);
	}

	delete(n->right);

}
///////////////////////////////////////


//////////////// Load ///////////////////
void HF::loadNodeFromFile(treeNode *n, ifstream& f){

	f.read((char*)&n->leaf, sizeof(bool));
	
	if(n->leaf){
						
		f.read((char*)&n->pfg, sizeof(double));
				
		int sz;
		f.read((char*)&sz, sizeof(int));			
		n->graspPList.resize(sz);		
		if(sz>0){
			for(int j=0; j<sz; ++j){		
				f.read((char*)&n->graspPList[j].x, sizeof(double));
				f.read((char*)&n->graspPList[j].y, sizeof(double));				
			}
		}
		
		//if(sz<=100){			
		//}else{
		//	vector<vec> orig(sz);			
		//	for(int i=0; i<sz; ++i){		
		//		f.read((char*)&orig[i].x, sizeof(double));
		//		f.read((char*)&orig[i].y, sizeof(double));
		//	}
		//	kmeans(orig, n->graspPList, n->graspPWeights, 100);
		//	n->sumW = 0;
		//	for(int i=0; i<100; ++i) 
		//		n->sumW += n->graspPWeights[i];
		//}		
		
	}else{		
		Test test;
		f.read((char*)&test.u.x, sizeof(double));
		f.read((char*)&test.u.y, sizeof(double));
		f.read((char*)&test.v.x, sizeof(double));
		f.read((char*)&test.v.y, sizeof(double));
		f.read((char*)&test.threshold, sizeof(int));
		n->test = test;	
		
		n->left = new treeNode;
		n->right = new treeNode;
		loadNodeFromFile(n->left, f);
		loadNodeFromFile(n->right, f);
	}

}

void HF::loadForestFromFolder(string folder_name){
		
    ifstream conf( (folder_name+"/"+"forest.txt").c_str());
	conf >> ntrees;
	conf.close();
	
	vTrees.resize(ntrees);
	
	for(int t=0; t<ntrees; ++t){
		stringstream s;
        s << folder_name.c_str() << "/tree" << t << ".dat";
		ifstream f(s.str().c_str(), ios::in | ios::binary);

		if(f.is_open()){														

			vTrees[t] = new treeNode();
			loadNodeFromFile(vTrees[t], f);

		}
		f.close();				
	}	

}
/////////////////////////////////////////////

