#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

struct true_pix{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
    vector<vector<int> > edges_t;
};


true_pix true_pixels(vector<vector<int> > junctions,vector<vector<int> > detailed_edges,vector<vector<int> > edges_t,int minx,int miny,int imcol){
        //cout<<imcol<< " "<<minx<<" "<<miny<<endl;
    for (int i=0;i<junctions.size();i++){
        //cout<<junctions.at(i).at(0)<<" "<<junctions.at(i).at(1)<<endl;

        int temp=junctions.at(i).at(0);
        junctions.at(i).at(0)=junctions.at(i).at(1);
        junctions.at(i).at(1)=temp;
        junctions.at(i).at(0)=imcol-junctions.at(i).at(0)-1;
        junctions.at(i).at(0)=junctions.at(i).at(0)+minx;
        junctions.at(i).at(1)=junctions.at(i).at(1)+miny;

    }



    //for (int i=0;i<edges_t.size();i++){
    //	int temp=edges_t.at(i).at(0);
    //	edges_t.at(i).at(0)=edges_t.at(i).at(1);
    //	edges_t.at(i).at(1)=temp;
    //	edges_t.at(i).at(0)=imcol-edges_t.at(i).at(0)+1;

    //	edges_t.at(i).at(0)+=minx;
    //	edges_t.at(i).at(1)+=miny;
    //	//edges_t.at(i).at(1)=imcol-edges_t.at(i).at(1)+1;

    //	temp=edges_t.at(i).at(2);
    //	edges_t.at(i).at(2)=edges_t.at(i).at(3);
    //	edges_t.at(i).at(3)=temp;
    //	//edges_t.at(i).at(2)=imcol-edges_t.at(i).at(2)+1;

    //	edges_t.at(i).at(2)+=minx;
    //	edges_t.at(i).at(3)+=miny;
    //	//edges_t.at(i).at(3)=imcol-edges_t.at(i).at(3)+1;
    //}

    for (int i=0;i<edges_t.size();i++){
        //wste oses akmes enwthhkan me alles na mhn ephreastoun
        //kai na synexisoun na exoun edges.at(i).at(0)=-90
        if (edges_t.at(i).at(0)!=-90){
            int temp=edges_t.at(i).at(0);
            edges_t.at(i).at(0)=edges_t.at(i).at(1);
            edges_t.at(i).at(1)=temp;
            edges_t.at(i).at(0)=imcol-edges_t.at(i).at(0);

            edges_t.at(i).at(0)+=minx;
            edges_t.at(i).at(1)+=miny;
            //edges_t.at(i).at(1)=imcol-edges_t.at(i).at(1)+1;

            temp=edges_t.at(i).at(2);
            edges_t.at(i).at(2)=edges_t.at(i).at(3);
            edges_t.at(i).at(3)=temp;
            edges_t.at(i).at(2)=imcol-edges_t.at(i).at(2);

            edges_t.at(i).at(2)+=minx;
            edges_t.at(i).at(3)+=miny;
            //edges_t.at(i).at(3)=imcol-edges_t.at(i).at(3)+1;
        }
    }



    int c;
    for (int i=0;i<detailed_edges.size();i=i+2){
        c=0;
        while (detailed_edges.at(i).at(c)>0 && c<detailed_edges.at(i).size()){
            int temp=detailed_edges.at(i).at(c);
            detailed_edges.at(i).at(c)=detailed_edges.at(i+1).at(c);
            detailed_edges.at(i+1).at(c)=temp;
            detailed_edges.at(i).at(c)=imcol-detailed_edges.at(i).at(c)+1;

            detailed_edges.at(i).at(c)+=minx;
            detailed_edges.at(i+1).at(c)+=miny;
            //detailed_edges.at(i+1).at(c)=imcol-detailed_edges.at(i+1).at(c)+1;
            c++;
        }
        detailed_edges.at(i).resize(c);
        detailed_edges.at(i+1).resize(c);
    }

    true_pix ret;
    ret.detailed_edges=detailed_edges;
    ret.edges_t=edges_t;
    ret.junctions=junctions;

    return ret;
}
