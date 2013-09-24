#include "rf.h"
#include <time.h>
#include <fstream>
#include <highgui.h>
#include <vector>


using namespace std;


///////////////---------------------------Training functions--------------------------------/////////////////

//void RF::getTrainSet(TrainSet& trainSet, string s){
	
//    ifstream fconfig(s.c_str());
//	fconfig >> nlabels;
//	trainFiles.resize(nlabels);
//	trainSet.resize(nlabels);

//	vector<ifstream> configFiles(nlabels);

//	for(int i=0; i<nlabels; ++i){
//		string s;
//		fconfig >> s;
//		trainFiles[i].open((s + ".dps").c_str(), ios::in | ios::binary);
//		configFiles[i].open((s + ".txt").c_str());
//	}

//	fconfig.close();
//	for(int l=0; l<nlabels; ++l){
//		int fpos, w, h;
//		double a,b; // xgrasp, ygrasp -> not used in rf.
//		while(configFiles[l] >> fpos >> w >> h >> a >> b){
//			trainImage ti1,ti2;
//			ti1.fpos = fpos;
//			ti1.w = w;
//			ti1.h = h;
//			ti1.symmetric = false;
//			trainSet[l].push_back(ti1);
//			ti2.fpos = fpos;
//			ti2.w = w;
//			ti2.h = h;
//			ti2.symmetric = true;
//			trainSet[l].push_back(ti2);
//		}
//	}
	
//	for(int i=0; i<nlabels; ++i)
//		configFiles[i].close();
//}

//void RF::splitSet(TrainSet& SetA, TrainSet& SetB, Test& test, TrainSet& trainSet, vector< vector < float > >& storedValues, int& samplesA, int& samplesB){
	
//	SetA.resize(trainSet.size());
//	SetB.resize(trainSet.size());
//	samplesA = 0;
//	samplesB = 0;
//	for(int i=0; i<trainSet.size(); ++i){
//		SetA[i].resize(0);
//		SetB[i].resize(0);
//		for(int j=0; j<trainSet[i].size(); ++j){
//			float testval = storedValues[i][j];
//			if(testval<=(float)test.threshold){
//				SetA[i].push_back(trainSet[i][j]);
//				samplesA++;
//			}
//			else{
//				SetB[i].push_back(trainSet[i][j]);
//				samplesB++;
//			}
//		}
//	}
//}

//double RF::measureSplit(TrainSet& SetA, TrainSet& SetB){
				
//	double sizeA = 0;
//	for(int i=0; i<SetA.size(); ++i)
//		sizeA += (double) SetA[i].size();

//	double sizeB = 0;
//	for(int i=0; i<SetB.size(); ++i)
//		sizeB += (double) SetB[i].size();
		
//	double entropyA = 0;
//	for(int i=0; i<SetA.size(); ++i){
//		double p = (double)SetA[i].size() / sizeA;
//		if(p>0) entropyA += p*log(p);
//	}

//	double entropyB = 0;
//	for(int i=0; i<SetB.size(); ++i){
//		double p = (double)SetB[i].size() / sizeB;
//		if(p>0) entropyB += p*log(p);
//	}

//	return (sizeA*entropyA+sizeB*entropyB)/(sizeA+sizeB);
//}


//bool RF::optimizeTest(Test& bestTest, TrainSet& bestSetA, TrainSet& bestSetB, TrainSet& trainSet, int depth){
	
//	vector<Test> tests(num_tests);
//	vector<float> min_val(num_tests, 1000000), max_val(num_tests, -1000000);
//	bool found = false;
	
//	double bestDist = -DBL_MAX;
//    vector<vector< vector<float> > > storedValues;
//	storedValues.resize(num_tests);
//	for(int i=0; i<num_tests; ++i){
//		storedValues[i].resize(trainSet.size());
//		for(int j=0; j<trainSet.size(); ++j)
//			storedValues[i][j].resize(trainSet[j].size());
//	}
	
//	for(int iteration = 0; iteration < num_tests; iteration++){
		
//		//Get random positions
//		tests[iteration].u.x = (float)rand()/(float)RAND_MAX;
//		tests[iteration].u.y = (float)rand()/(float)RAND_MAX;
//		tests[iteration].v.x = (float)rand()/(float)RAND_MAX;
//		tests[iteration].v.y = (float)rand()/(float)RAND_MAX;
		
//	}
	
//	for(int label=0; label<trainSet.size(); ++label){
//		for(int i=0; i<trainSet[label].size(); ++i){
//			trainImage ti = trainSet[label][i];
//			trainFiles[label].seekg(ti.fpos);
//			trainFiles[label].read((char*)tmpbuffer, ti.w*ti.h*sizeof(float));
//			for(int iteration=0; iteration<num_tests; iteration++){
//				int x1 = int( (tests[iteration].u.x * (ti.w-1)) );
//				int y1 = int( (tests[iteration].u.y * (ti.h-1)) );
//				int x2 = int( (tests[iteration].v.x * (ti.w-1)) );
//				int y2 = int( (tests[iteration].v.y * (ti.h-1)) );
//				float val1, val2;
//				if(!ti.symmetric){
//					val1 = tmpbuffer[y1*ti.w + x1];
//					val2 = tmpbuffer[y2*ti.w + x2];
//				}else{
//					val1 = tmpbuffer[y1*ti.w + (ti.w-1-x1)];
//					val2 = tmpbuffer[y2*ti.w + (ti.w-1-x2)];
//				}
//				if(val1 == 0) val1 = 100000;
//				if(val2 == 0) val2 = 100000;
//				storedValues[iteration][label][i] = val1 - val2;
//				if( (val1!=100000) && (val2!=100000) && (min_val[iteration]>val1-val2) )
//					min_val[iteration] = val1-val2;
//				if( (val1!=100000) && (val2!=100000) && (max_val[iteration]<val1-val2) )
//					max_val[iteration] = val1-val2;
//			}
//		}
//	}

//	for(int iteration=0; iteration<num_tests; ++iteration){
		
//		int thres = (int)min_val[iteration];
//		int i=0;
//		while( (i<3) || (thres<=(int)max_val[iteration]) ){
//			if(i==0){
//				tests[iteration].threshold = 50000;
//				i++;
//			}
//			else if(i==1){
//				tests[iteration].threshold = -50000;
//				i++;
//			}
//			else if(i==2){
//				tests[iteration].threshold = 0;
//				i++;
//			}
//			else{
//				tests[iteration].threshold = thres;
//				thres += 10;
//			}
			
//			TrainSet SetA, SetB;
//			int samplesA, samplesB;
//			splitSet(SetA, SetB, tests[iteration], trainSet, storedValues[iteration], samplesA, samplesB);

//			if( (samplesA > 0) && (samplesB > 0) ){
		
//				double m = measureSplit(SetA, SetB);
//				if(m>bestDist){

//					found = true;
//					bestDist = m;
//					bestTest = tests[iteration];
//					bestSetA = SetA;
//					bestSetB = SetB;

//				}
//			}
//		}
//	}
//	return found;
//}

//void RF::makeLeaf(treeNode *node, TrainSet& trainSet){
//	node->leaf = true;
//	double samples = 0;
	
//	for(int i=0; i<trainSet.size(); ++i)
//		samples += trainSet[i].size();
	

//	node->pfg.resize(trainSet.size());
//	for(int i=0; i<trainSet.size(); ++i)
//		node->pfg[i] = (double)trainSet[i].size() / samples;
	
//}

//void RF::growNode(treeNode *node, TrainSet& trainSet, int depth){

//	if( depth<max_depth ){
		
//		node->leaf = false;
//		TrainSet SetA, SetB;
//		Test test;
//		node->depth = depth;

//		if( optimizeTest(test, SetA, SetB, trainSet, depth) ){

//			//cout << "Set A:" << endl << "Size: " << SetA[0].size() + SetA[1].size() << endl << "Size 0: " << SetA[0].size() << endl << "Size 1: " << SetA[1].size() << endl;
//			//cout << "Set B:" << endl << "Size: " << SetB[0].size() + SetB[1].size() << endl << "Size 0: " << SetB[0].size() << endl << "Size 1: " << SetB[1].size() << endl;

//			node->test = test;
//			node->left = new treeNode;
//			node->right = new treeNode;
			
//			int samples = 0;
//			int nonZeroClasses = 0;
//			for(int i=0; i<SetA.size(); ++i){
//				samples += SetA[i].size();
//				if(SetA[i].size()!=0) nonZeroClasses++;
//			}
			
//			if( (samples > min_samples) && (nonZeroClasses > 1) )
//				growNode(node->left, SetA, depth+1);
//			else
//				makeLeaf(node->left, SetA);
			
//			samples = 0;
//			nonZeroClasses = 0;
//			for(int i=0; i<SetB.size(); ++i){
//				samples += SetB[i].size();
//				if(SetB[i].size()!=0) nonZeroClasses++;
//			}
			
//			if( (samples > min_samples) && (nonZeroClasses > 1) )
//				growNode(node->right, SetB, depth+1);
//			else
//				makeLeaf(node->right, SetB);

//		}else
//			makeLeaf(node, trainSet);
//	}else
//		makeLeaf(node, trainSet);
//}


//void RF::trainTrees(vector<int>& treesNo, TrainSet& trainSet){

//	int firstTree;
//	cout << "Start tree No: ";
//	cin >> firstTree;

//	for(int t=0; t<treesNo.size(); ++t){

//		treeNode *tree = new treeNode;
		
//		cout << "Tree: " << t+firstTree << " ";
//		int t0 = time(NULL);
//		growNode(tree, trainSet, 0);
//		int t1 = time(NULL);
//		cout << t1-t0 << "sec";
		
		
//		stringstream s;
//		s << "tree" << treesNo[t]+firstTree << ".dat";
//		ofstream f(s.str().c_str(), ios::out | ios::binary);
//		saveTreeNode(tree, f);
//		cout << " -> saved" << endl;
		
//		deleteTreeNode(tree);
//		delete tree;

//		f.close();
//	}

//}


//void RF::train(string s){

//	TrainSet trainSet;
//	getTrainSet(trainSet, s);
	
//	vector<int> trees;
//	trees.resize(ntrees);
//	for(int i=0; i<ntrees; ++i)
//		trees[i] = i;
	
//	trainTrees(trees, trainSet);
	
//	//ofstream of("forest.txt");
//	//of << ntrees << " " << nlabels << endl;
//	//of.close();


//	/*
//	vector< vector<int> > trees(5);
	
//	for(int i=0; i<5; ++i)
//		trees[i].resize(0);

//	for(int t=0; t<ntrees; ++t){
//		trees[t%5].push_back(t);
//	}
	
//	boost::thread thread1(boost::bind(&RF::trainTrees, this, trees[1]));
	

//	if(ntrees>0)
//		thread1.join();
	
//	if(ntrees>1)
//		thread2.join();
//	if(ntrees>2)
//		thread3.join();
//	if(ntrees>3)
//		thread4.join();
//	if(ntrees>4)
//		thread5.join();
//		*/
//}

////////////////////////////////////////////////////


////////////////----------------------------Detection functions-----------------------------------//////////////////////
/*
void RF::GetBoundingBox(cv::Mat& mat, cv::Rect& rect){	
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
*/
void RF::GetBoundingBox(cv::Mat& mat, cv::Rect& rect){

	cv::Mat temp_mat;
	mat.convertTo(temp_mat, CV_8UC1);
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	findContours( temp_mat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	vector<cv::Rect> boundRect( contours.size() );
	int maxArea = 0;
	vector<vector<cv::Point> > contours_poly( contours.size() );
	for( int i = 0; i < contours.size(); i++ ){
		approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
		if(boundRect[i].width * boundRect[i].height > maxArea){
			maxArea = boundRect[i].width * boundRect[i].height;
			rect = boundRect[i];
		}
    }
}

RF::treeNode* RF::getLeaf(cv::Mat& img, treeNode* n){

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

//Mat -> depthMap (float)
int RF::RFDetect(cv::Mat& cvImg, vector<double>& res_tab){				
	
	cv::Rect rect;
	GetBoundingBox(cvImg, rect);
	cv::Mat matIn = cvImg(rect);		
	cv::Mat matOut;
	cv::bilateralFilter(matIn, matOut, -1, 13, 8, 4);
	for(int i=0; i<matOut.rows; ++i)
		for(int j=0; j<matOut.cols; ++j)
			if(matOut.at<float>(i, j)<300)
				matOut.at<float>(i, j) = 0;


	vector<double> results(nlabels, 0);
	for(int i=0; i<ntrees; ++i){
		treeNode *leaf;
		leaf = getLeaf(matOut, vTrees[i]);
		for(int j=0; j<nlabels; ++j)
			results[j] += leaf->pfg[j] / (double)ntrees;
	}
	double max = 0;
	int k=0;
	res_tab.resize(nlabels);
	for(int i=0; i<nlabels; ++i){
		res_tab[i] = results[i];
		if(results[i] > max){
			max = results[i];
			k = i;
		}
	}
	return k;							
}





//////////////------------------------------- I/O functions ------------------------------------///////////////////


//////////// Save ////////////////
void RF::saveTreeNode(treeNode *n, ofstream& f){

	f.write((char*)&n->leaf, sizeof(bool));			
	
	if(n->leaf){
		
		for(int i=0; i<nlabels; ++i)
			f.write((char*)&n->pfg[i], sizeof(double));
		
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

void RF::deleteTreeNode(treeNode *n){

	if(!n->left->leaf)			
		deleteTreeNode(n->left);

	delete(n->left);

	if(!n->right->leaf)			
		deleteTreeNode(n->right);

	delete(n->right);

}
///////////////////////////////////////


//////////////// Load ///////////////////
void RF::loadNodeFromFile(treeNode *n, ifstream& f){

	f.read((char*)&n->leaf, sizeof(bool));
	
	if(n->leaf){
		
		n->pfg.resize(nlabels);
		for(int i=0; i<nlabels; ++i)
			f.read((char*)&n->pfg[i], sizeof(double));				
		
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

void RF::loadForestFromFolder(string folder_name){
		
    ifstream conf((folder_name+"/"+"forest.txt").c_str());
	conf >> ntrees >> nlabels;
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
