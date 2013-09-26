#include "common_matlab.h"
#include "common_cpp.h"




#define PI		3.14159265	

void shiftRows(cv::Mat& mat);
void shiftRows(cv::Mat& mat,int n);

void uniform_interp(double *XIs, double *YIs, int n_samp, 
					double *Xs,  double *Ys,  int n_pt)

{
	double	dx,dy,x,y,r;
	int		ii;

	// compute length from first point to all other points
	double	*seg_lens	= new double[n_pt];
	seg_lens[0]	= 0;
	for(ii=1;ii<n_pt;++ii)
	{
		dx	= Xs[ii]-Xs[ii-1];
		dy	= Ys[ii]-Ys[ii-1];
		seg_lens[ii]	= sqrt(dx*dx+dy*dy) + seg_lens[ii-1];
	}

	double	d_len	= seg_lens[n_pt-1]/(n_samp+1);
	int		i_fill	= 0;
	double	cur_len	= d_len;
	for(ii=1;ii<n_pt && i_fill<n_samp;++ii)
	{
		while(cur_len<=seg_lens[ii] && i_fill<n_samp)
		{
			// interpolate a point
			r	= (cur_len-seg_lens[ii-1]) / (seg_lens[ii]-seg_lens[ii-1]);
			x	= Xs[ii-1]+r*(Xs[ii]-Xs[ii-1]);
			y	= Ys[ii-1]+r*(Ys[ii]-Ys[ii-1]);
			XIs[i_fill] = x;
			YIs[i_fill]	= y;
			
			++i_fill;
			cur_len	+= d_len;
		}
	}

	delete	[]seg_lens;
}






void trans_inputs_id(double  *pX1, double *pY1, double *fg_mask1,
					 int n_samp, vector<cv::Point_<double> > &cntr, cv::Mat &mask)

{

	    int n_V1=(int) cntr.size();
		double *pX		= new double[n_V1];
		double *pY		= new double[n_V1];



		for (int i=0; i<n_V1;i++){
			pX[i]	=	cntr[i].x;
			pY[i]	=	cntr[i].y;
		}


		cv::Mat cntrX(n_V1,1,CV_64F);
		cv::Mat cntrY(n_V1,1,CV_64F);
		cv::Mat cntrXl(n_V1,1,CV_64F);
		cv::Mat cntrYl(n_V1,1,CV_64F);
		cv::Mat dX(n_V1,1,CV_64F);
		cv::Mat dY(n_V1,1,CV_64F);
			
		
	
		for (int i=0;i<n_V1;i++)			
			cntrX.at<double>(i,0)=pX[i];

		

		

		for (int i=0;i<n_V1;i++)
			cntrY.at<double>(i,0)=pY[i];

	

		for (int i=0;i<n_V1-1;i++)
			cntrXl.at<double>(i,0)=pX[i+1];

		cntrXl.at<double>(n_V1-1,0)=pX[0];

		for (int i=0;i<n_V1-1;i++)
			cntrYl.at<double>(i,0)=pY[i+1];

		cntrYl.at<double>(n_V1-1,0)=pY[0];
		
		dX	= cntrXl-cntrX;
		dY	= cntrYl-cntrY;


		cv::Mat angs(n_V1,1,CV_64F);
		cv::Mat angsl(n_V1,1,CV_64F);
		cv::Mat dA(n_V1,1,CV_64F);

		
		for (int i=0;i<n_V1;i++)
			angs.at<double>(i,0)=atan2(dY.at<double>(i,0),dX.at<double>(i,0));

		for (int i=0;i<n_V1-1;i++)
			angsl.at<double>(i,0)=angs.at<double>(i+1,0);

		angsl.at<double>(n_V1-1,0)=angs.at<double>(0,0);
		dA	= angsl-angs;
		double dAt;
		for (int i=0;i<n_V1;i++){
			dAt=dA.at<double>(i,0);
			if (dAt>PI)
				dA.at<double>(i,0)=dAt-2*PI;
			dAt=dA.at<double>(i,0);
			if (dAt<-PI)
				dA.at<double>(i,0)=dAt+2*PI;
		}

		cv::Scalar mm=cv::mean(dA);
		double hh=mm.val[0];
		bool bClk=(hh<0);

		if (bClk){
			cv::flip(cntrX,cntrX,0);
			cv::flip(cntrY,cntrY,0);

			for (int i=0;i<n_V1;i++)
				pX[i]=cntrX.at<double>(i,0);

		

		for (int i=0;i<n_V1;i++)
			pY[i]=cntrY.at<double>(i,0);

		}
	

	double v_min, v_max;

	int idx_min[2] = {255,255}, idx_max[2] = {255, 255};

	cv::minMaxIdx(cntrY, &v_min, &v_max, idx_min, idx_max);

	if (idx_min[0]!=0){

	
	shiftRows(cntrY,-idx_min[0]);
	shiftRows(cntrX,-idx_min[0]);
	for (int i=0;i<n_V1;i++)
				pX[i]=cntrX.at<double>(i,0);

	for (int i=0;i<n_V1;i++)
		pY[i]=cntrY.at<double>(i,0);

}
		

		uniform_interp( pX1, pY1, n_samp, pX,  pY,  n_V1);

	


		int nRow1	=	mask.rows;
		int nCol1	=	mask.cols;
	

	
		for (int i=0; i < nRow1; i++){	
			for (int j=0; j < nCol1; j++){
				double Et2=(double) mask.at<unsigned char>(i,j);
				if (Et2>0)
					Et2=1;
				MAT_SET(fg_mask1,j,i,Et2,nRow1);
			}
		}

		delete []pX;
	
		delete []pY;

}




//circular shift one row from up to down
void shiftRows(cv::Mat& mat) {

    cv::Mat temp;
    cv::Mat m;
    int k = (mat.rows-1);
    mat.row(k).copyTo(temp);
    for(; k > 0 ; k-- ) {
        m = mat.row(k);
        mat.row(k-1).copyTo(m);
    }
    m = mat.row(0);
    temp.copyTo(m);

}

//circular shift n rows from up to down if n > 0, -n rows from down to up if n < 0
void shiftRows(cv::Mat& mat,int n) {

    if( n < 0 ) {

        n = -n;
        flip(mat,mat,0);
        for(int k=0; k < n;k++) {
            shiftRows(mat);
        }
        flip(mat,mat,0);

    } else {

        for(int k=0; k < n;k++) {
            shiftRows(mat);
        }
    }

}











void compute_IDSCdescriptors_C(double *D1, double *pX1, double *pY1, int n_V1,
						  double *fg_mask1, int nRow1, int nCol1, int n_dist, int n_theta,
						  bool bTangentV,  bool bSmoothCont)
{

// Initialization
	int *X1	= new int[n_V1];
	int	*Y1	= new int[n_V1];
	for(int i=0; i<n_V1; i++) {
		X1[i]= ROUND(pX1[i]);
		Y1[i]= ROUND(pY1[i]);
	}

	

	/* build the graph */
	double *E1	= new double[3*n_V1*n_V1];	// locate memory for edge list
	int		n_E1;
	build_graph_contour(E1, n_E1,
						fg_mask1, nRow1, nCol1,
						X1, Y1, n_V1,
						bSmoothCont);

	
	
    double	*En1		= new double[3*n_E1];
	double Et1;
	for(int i=0;i<n_E1;i++){
		for(int j=0;j<3;j++){
			Et1=MAT_GET(E1,i,j,3);
			MAT_SET(En1,j,i,Et1,n_E1);
		}
	}


	

	double *dis_mat1	= new double[n_V1*n_V1];
	double *ang_mat1	= new double[n_V1*n_V1];
	bellman_ford_allpair(dis_mat1, ang_mat1,
						 pX1, pY1, n_V1,
						 En1, n_E1);

	
	


	double *Dist1	= new double[n_V1*(n_V1-1)];
	double *Ang1	= new double[n_V1*(n_V1-1)];
	sc_hist_inputs(Dist1, Ang1, pX1, pY1, dis_mat1, ang_mat1, bTangentV, n_V1, n_V1, n_V1);

	


    //int nBin=n_dist*n_theta;
   
	/*mxArray	*pMxD1	= mxCreateDoubleMatrix(nBin,n_V1,mxREAL);
    double	*D1		= mxGetPr(pMxD1);*/
	comp_sc_hist(D1, Dist1,n_V1-1, Ang1, n_V1-1, n_V1, n_dist, n_theta);


	delete []Y1;
	delete []X1;
	delete []E1;
	delete []En1;
	delete []dis_mat1;
	delete []ang_mat1;
	delete []Dist1;
	delete []Ang1;
	
}

/*
% function [E] = build_graph_contour_C(X,Y,fg_mask,bSmoothCont)
% 	Initialize the graph from contour points
%		for each p1
%			for each p2
%				if (p1,p2) inside the shape
%					add edge e(p1,p2) into E
%					set dis_mat(p1,p2)=dis_mat(p2,p1)
%					set ang_mat(p1,p2), ang_mat(p2,p1)
%					set viewable(p1,p2)=viewable(p2,p1)=1

% Author	:	Haibin Ling, hbling AT umiacs.umd.edu
%				Computer Science Department, University of Maryland, College Park
*/

/* common functions */



#define NEW_EDGE(E,n_E,p1,p2,dis)	{\
					MAT_SET(E, n_E, 0, p1, 3);\
					MAT_SET(E, n_E, 1, p2, 3);\
					MAT_SET(E, n_E, 2, dis,3);\
		}

/*-------------------------------------------------------------------------------*/

void build_graph_contour(double *E, int &n_E,
						 double *fg_mask, int nRow, int nCol,
						 int *X, int *Y, int n_V,
						 bool	bSmoothCont
						 )
{
	/*
	dis_mat		= inf*ones(n_V,n_V);	% each COLUMN is a record at a point
	ang_mat		= inf*ones(n_V,n_V);
	viewable	= zeros(n_V,n_V);
	*/
	int		x1,y1,x2,y2,dx,dy,n,k;
	int		p1,p2;
	bool	bIn	= false;
	double	deltaX,deltaY,x,y;
	double	dis;
	n_E		= 0;
	for(p1=0;p1<n_V;++p1)
	{
		x1	= X[p1]-1;
		y1	= Y[p1]-1;
		for(p2=p1+1;p2<n_V;++p2)
		{
			x2	= X[p2]-1;
			y2	= Y[p2]-1;
			dx	= x2-x1;
			dy	= y2-y1;

			// test if (p1,p2) inside the shape
			bIn	= true;
			n	= MAX(abs(dx),abs(dy));

			if(bSmoothCont && (p2==p1+1 || p2-p1==n_V-1))
			{
				bIn	= true;
				//printf("bIn\n");
			}
			else if(n>2)
			{
				deltaX	= ((double)dx)/n;
				deltaY	= ((double)dy)/n;
				x		= x1;
				y		= y1;
				for(k=0;bIn && k<n-1;k++)
				{
					x	+= deltaX;
					y	+= deltaY;
					if( MAT_GET(fg_mask,ROUND(x),ROUND(y),nRow) < .5 )
						bIn	= false;
				}
			}
			// if inside, set dis_mat, ang_mat, viewable
			if(bIn)
			{
				// adding edge
				dis		= sqrt((double)(dx*dx+dy*dy));
				NEW_EDGE(E,n_E, (double)(p1+1),(double)(p2+1),dis);				
				n_E++;
			}
		}// of p2
	}//of p1
}


/* 
	Fast way to compute all pair shortest path for 2D binary shape contour

	Haibin Ling, 08/18/2004

*/




//----------------------------------------------------------------------
void bellman_ford_allpair(double *dis_mat, double *ang_mat,
						  double *X, double *Y, int n_V,
						  double *E, int n_E)
{
	const double INF_DIS	= n_V*n_E;
	const double INF_THRE	= INF_DIS-2;
	//const double PI	= 3.14159265;
	int		n_E1,i,s,u,v,e;
	double	dx,dy,w,ang;
	bool	bstop;

	int		*Ep1 = new int[n_E];
	int		*Ep2 = new int[n_E];
	double	*Ew	 = new double[n_E];
	
	// initialize dis_mat and ang_mat
	double	*pD,*pA;
	pD	= dis_mat;
	pA	= ang_mat;
	for(u=0;u<n_V*n_V;u++) {
		*(pD++)	= INF_DIS;
		*(pA++)	= -10;
	}

	// set viewable values
	double *pu	= E;
	double *pv	= E+n_E;
	double *pw	= pv+n_E;
	for(e=0;e<n_E;e++)
	{
		u		= (int)pu[e]-1;
		v		= (int)pv[e]-1;
		w		= pw[e];
		Ep1[e]	= u;
		Ep2[e]	= v;
		Ew[e]	= w;
		
		// set dis_mat
		MAT_SET(dis_mat,u,v,w,n_V);
		MAT_SET(dis_mat,v,u,w,n_V);

		// set ang_mat
		dx	= X[v]-X[u];
		dy	= Y[v]-Y[u];
		ang	= atan2(dy,dx);
		MAT_SET(ang_mat,u,v,ang,n_V);
		MAT_SET(ang_mat,v,u,(ang>0)?(ang-PI):(ang+PI),n_V);
	}


	// shortest pathes from every start point s
	int		*U	= new int[2*n_E];
	int		*V	= new int[2*n_E];
	double	*W	= new double[2*n_E];
	for(s=0;s<n_V;s++)
	{
		pD		= dis_mat+s*n_V;
		pA		= ang_mat+s*n_V;
		pD[s]	= 0;
		//pA[s]	= -100;

		/*/ reduce the size of graph
		memcpy(U,Ep1,n_E*sizeof(int));
		memcpy(V,Ep2,n_E*sizeof(int));
		memcpy(W,Ew, n_E*sizeof(double));	
		n_E1	= n_E;/*/
		n_E1	= 0;
		for(e=0;e<n_E;e++)
		{
			u	= Ep1[e];
			v	= Ep2[e];
			w	= Ew[e];
			
			if(pD[u]>INF_THRE)	// this node is not viewable from s
			{
				U[n_E1]	= v;
				V[n_E1]	= u;
				W[n_E1]	= w;
				n_E1++;
			}

			if(pD[v]>INF_THRE)	// this node is not viewable from s
			{
				U[n_E1]	= u;
				V[n_E1]	= v;
				W[n_E1]	= w;
				n_E1++;
			}
		}


		/* Relaxation using standard bellman-ford*/
		bstop	= false;
		for(i=1; i<n_V-1 && !bstop; i++)
		{
			bstop	= 1;
			for(e=0; e<n_E1; ++e)
			{
				/* Relax for each edge */
				u	= U[e];
				v	= V[e];
				w	= W[e];
				if(pD[v]>pD[u]+w)	{
					pD[v]	= pD[u]+w;
					pA[v]	= pA[u];
					//pP[v] = u+1;
					bstop = 0;
				}

				/*
				if(pD[u]>pD[v]+w)	{
					pD[u]	= pD[v]+w;
					pA[u]	= pA[v];
					//pP[v] = u+1;
					bstop = 0;
				}/**/
			}
			/*printf("i=%d\n",i);*/
		}
	}//of start point s


	delete	[]U;
	delete	[]V;
	delete	[]W;
	delete	[]Ep1;
	delete	[]Ep2;
	delete	[]Ew;
}

 /* This is a C verion of following matlab code, Haibin Ling, 09/04/2004
if bTangent
	Xs	= [X(end); X; X(1)];
	Ys	= [Y(end); Y; Y(1)];
	gX	= gradient(Xs);
	gY	= gradient(Ys);

	thetas	= atan2(gY,gX);
	thetas	= thetas(2:end-1);
	thetas	= repmat(thetas',n_pt,1);
	
	ang_mat	= ang_mat-thetas;
	
	idx		= find(ang_mat>pi);
	ang_mat(idx)	= ang_mat(idx)-2*pi;
	idx		= find(ang_mat<-pi);
	ang_mat(idx)	= ang_mat(idx)+2*pi;
end



%-- Compute shape context  ----------------------------------------
n_pt		= size(V,1);
dists		= zeros(n_pt-1,n_pt);		% distance and orientation matrix, the i-th COLUMN contains
angles		= zeros(n_pt-1,n_pt);		% dist and angles from i-th point to all other points
id_gd		= setdiff(1:n_pt*n_pt, 1:(n_pt+1):n_pt*n_pt);
dists(:)	= dis_mat(id_gd);
angles(:)	= ang_mat(id_gd);


*/







 
void sc_hist_inputs(double* Dist, double* Ang, double* X, double* Y, double* X1, double* X2, bool bTangentV, int m, int m1, int dim)
{
	

	if (bTangentV){

		//const double PI	= 3.141592653589793;


		double* gX=new double[m1];
		double* gY=new double[m1];
		double dx=1;
		double thetas;

		gX[0] = (X[1] - X[m-1]) / (2*dx);
		
		for(int i=1;i<(m-1);i++)
			gX[i] = (X[i+1] - X[i-1]) / (2*dx); 
		gX[m-1] = (X[0] - X[m-2]) / (2*dx);


		gY[0] = (Y[1] - Y[m-1]) / (2*dx);
		
		for(int i=1;i<(m-1);i++)
			gY[i] = (Y[i+1] - Y[i-1]) / (2*dx); 
		gY[m-1] = (Y[0] - Y[m-2]) / (2*dx);
		
	
		double Xtemp;
		for(int i=0;i<m;i++){
			
			thetas=atan2(gY[i],gX[i]);
			
			
			for (int j=0;j<m;j++){
				Xtemp=MAT_GET(X2,i,j,m1);
				Xtemp-=thetas;
				if (Xtemp>PI)
					Xtemp-=2*PI;
				else if (Xtemp<-PI)
					Xtemp+=2*PI;

				MAT_SET(X2,i,j,Xtemp,m1);
			}
				
		}
	}

	double Dtemp;
	double Atemp;
	for(int i=0;i<m1;i++){
		for(int j=0;j<m1;j++){
			if (j==i)
				continue;
			else if (j<i){
				Dtemp=MAT_GET(X1,i,j,m1);
				Atemp=MAT_GET(X2,i,j,m1);
				MAT_SET(Dist,i,j,Dtemp,m1-1);
				MAT_SET(Ang,i,j,Atemp,m1-1);
			}
			else if (j>i){
				Dtemp=MAT_GET(X1,i,j,m1);
				Atemp=MAT_GET(X2,i,j,m1);
				MAT_SET(Dist,i,j-1,Dtemp,m1-1);
				MAT_SET(Ang,i,j-1,Atemp,m1-1);
			}
				
			
		}
	}
	
	
return;
}



 /* This is a C verion of following matlab code, Haibin Ling, 09/04/2004
  
% [sc_hist]	= comp_sc_hist(dists,angles);
%		
% 	Compute shape context from input distances and angles
% Inputs: 	
% 		dists	: distance and orientation matrix, the i-th COLUMN contains
% 		angles	: dist and angles from i-th point to all other points

function [sc_hist] = comp_sc_hist(dists,angles,n_dist,n_theta)

	if ~exist('n_dist')		n_dist		= 10;	end
	if ~exist('n_theta')	n_theta		= 16;	end
	n_pts		= size(dists,2);

	%- preprocessing distances
	if 1
		% using log distance
		logbase		= 1.5;
		mean_dis	= mean(dists(:));
		b			= 1;
		a			= (logbase^(.75*n_dist)-b)/mean_dis;
		
		dists		= floor(log(a*dists+b)/log(logbase));
		dists		= max(dists,1);
		dists		= min(dists,n_dist);
	else
		% using linear distance
		logbase		= 1.5;
		mean_dis	= mean(dists(:));
		delta_dis	= mean_dis/n_dist;
		dists		= ceil(dists/delta_dis);
		dists		= max(dists,1);
		dists		= min(dists,n_dist);
	end		
% 	keyboard;
	
	%- preprocessing angles
	delta_ang	= 2*pi/n_theta;
	if 0
		angles		= angles+delta_ang/2;
		idx			= find(angles<0);
		angles(idx)	= angles(idx)+2*pi;
		angles		= ceil(angles/delta_ang);
	else
		angles		= ceil((angles+pi)/delta_ang);
	end
	angles		= max(angles,1); 
	angles		= min(angles,n_theta);
	
	
	%- shape context
	sc_hist		= zeros(n_theta*n_dist, n_pts);
	sctmp		= zeros(n_dist,n_theta);
	for v=1:n_pts
		for dis=1:n_dist
			for ang=1:n_theta
				sctmp(dis,ang)	= length(find(dists(:,v)==dis & angles(:,v)==ang));
			end
		end
		sc_hist(:,v)	= sctmp(:);
		
		if 0
			figure(324);	clf; hold on;
			imagesc(sctmp); colorbar;	%colormap(gray); 
			title(i2s(v));	drawnow
			pause;
		end
	end
	
	sc_hist	= sc_hist/(n_pts-1);
	
return;



*/




void comp_sc_hist(double* sc_hist, double* X1,int m1, double* X2, int m2, int dim, int n_dist,int n_theta)
{
	
	//const double PI	= 3.141592653589793;
	int L=m1*dim;
	double mean_dis;
	double delta_ang;

	double	*pD;
	pD	= sc_hist;
	
	double sum=0;
	for(int i=0;i<L;i++)
		sum+=X1[i];
		
	mean_dis=sum/((double)L);

	//- preprocessing distances
	
		// using log distance
	
		double logbase= 1.5;
		
		
		double b			= 1.0;
		double e=.75*n_dist;
		double o			=  pow ( logbase,  e);
		double a=(o-b)/mean_dis;
		
		for(int i=0;i<L;i++){
			X1[i]		= floor(log(a*X1[i]+b)/log(logbase));
			X1[i]=MAX(X1[i],1);
			X1[i]=MIN(X1[i],n_dist);
		}

		
	
	
// keyboard;
	
	//- preprocessing angles
	delta_ang	= 2*PI/(double) n_theta;

		
	for(int i=0;i<L;i++){
			X2[i]		= ceil((X2[i]+PI)/delta_ang);
			X2[i]=MAX(X2[i],1);
			X2[i]=MIN(X2[i],n_theta);
		}
	 
	
	//- shape context
	
	
	
	double td,ta;
	
	for (int v=0; v<dim; v++){
		
		for (int ang=1;ang<=n_theta;ang++) {
			for (int dis=1; dis<=n_dist; dis++){
				int count=0;
				for (int i=0;i<m1;i++){
					td=MAT_GET(X1,v,i,m1);
					ta=MAT_GET(X2,v,i,m2);
					if (((int)td==dis) && ((int) ta==ang))
					count++;
				}
				*(pD++)	= (double)count/(double)(dim-1);
				
				
			
			}
		}
		
		
	}

	
	
return;
}






