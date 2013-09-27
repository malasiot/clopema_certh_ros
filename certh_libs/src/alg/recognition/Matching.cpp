/* 
	Dynamic Programming to solve contour matching.
	This is the matlab interface of a C++ function.

	Haibin Ling, 08/29/2004

	  [C,T]=DPMatching_C(A, thre, n_start, n_search)

		A 	- a square cost matrix,
				A(r,c) is the match cost from the r-th point in contour 1 to
				the c-th curve on countour 2
		thre- use average*thre as the occlusion
		C 	- the optimal assignment.
		T 	- the cost of the optimal assignment.
*/


/* common functions */

#include "common_matlab.h"
#include "common_cpp.h"


bool DPMatchingFixStartPoint( int *C, double &T,			// output
							  double *A, double thre,		// input
							  int	M, int N				// A is MxN matrix
							  );

bool DPMatchingMultiStart (	int *C, double &T_best,			// output
							double *A, double thre,		// input
							int	M, int N,				// A is MxN matrix
							int	n_start,				// number of start point
							int n_search);

bool DPMatchingCircular(int *C, double &T,			// output
						double *A, double thre,		// input
						int	M, int N,				// A is MxN matrix
						int n_search
						);

/* global variables, for acceleration */
double	*D 		= 0;	//new double[M*N];		// DP matrix, D[y,x] is the cost of contour 1 at x
int		*links	= 0;	//new int[M*N];			//	matching to contour 2 at y



/****************************************************************************
	
	  [C,T]=DPMatching_C(A, thre, n_start, n_search)

 ****************************************************************************/

void DPMatching( double *cvec, double *match_cost, double *A, double thre, int M, int N, int n_start, int n_search)
{
    /* Input data */

	



	/* call algorithm */
	D 				= new double[M*N];		// DP matrix, D[y,x] is the cost of contour 1 at x
	links			= new int[M*N];			//	matching to contour 2 at y
	int		*C		= new int[M];
	double	T		= 100000000;

	bool bSucc	= false;
	if(n_start==1 && n_search==1)
		bSucc	= DPMatchingFixStartPoint(C, T,	A, thre, M, N);
	else
		bSucc		= DPMatchingMultiStart(C,T, A,thre,M,N,n_start,n_search);
		//bSucc	= DPMatchingCircular(C, T,	A, thre, M, N, n_search);


    /* Output data */
    
	
	for(int ii=0;ii<M;++ii)
		cvec[ii]	= (double)C[ii]+1;

   *match_cost=T;

    /* Return */
   

	delete	[]C;
	delete	[]D;
	delete	[]links;
}



/*-------------------------------------------------------------------
	C 	- the optimal assignment.
	T 	- the cost of the optimal assignment.
	A 	- a square cost matrix,
			A(r,c) is the match cost from the r-th point in contour 1 to
			the c-th curve on countour 2
	thre- use average*thre as the occlusion
*/	
bool DPMatchingFixStartPoint( int *C, double &T,			// output
							  double *A, double thre,		// input
							  int	M, int N				// A is MxN matrix
							  )
{
	double	uplimit = 100000000;
	double	pen1	= thre;
	bool	bSucc	= true;
	int		pt1,pt2;
	double	dTmp,dTmp1,dTmp2,dTmp3;


	//mexPrintf("from FixStartPoint!\n");

//	double	*D 		= new double[M*N];		// DP matrix, D[y,x] is the cost of contour 1 at x
//	int		*links	= new int[M*N];			//	matching to contour 2 at y /*
	// the above two lines are GLOBAL now, to accelerate the code

	/*
	for(pt1=0;pt1<M;++pt1)
	{
		for(pt2=0;pt2<N;++pt2)
		{
			MAT_SET(D,	  pt2,pt1,uplimit+1,M);
			MAT_SET(links,pt2,pt1,-1,       M);
		}
	}//*/

	
	//- initialization
	//mexPrintf("pen1=%lf\n",pen1);
	dTmp	= MAT_GET(A,0,0,M);
	if(dTmp<pen1) {							// MAT_SET(D,1,1,dTmp,M);
		MAT_SET(D,0,0,dTmp,M);
		MAT_SET(links,0,0,1,M);
	}
	else {
		MAT_SET(D,0,0,pen1,M);
		MAT_SET(links,0,0,2,M);
	}

	//pt1	= 0;
	//pt2	= 0;
	//mexPrintf("\nlinks(%2d,%2d)=%d,  D(%2d,%2d)=%lf \n\n", 
	//	pt1,pt2, MAT_GET(links,pt2,pt1,M), pt1,pt2, MAT_GET(D,pt2,pt1,M));

	for(pt2=1;pt2<N;pt2++)
	{
		dTmp1	= MAT_GET(A,pt2,0,M)+pt2*pen1;
		dTmp3	= MAT_GET(D,pt2-1,0,M)+pen1;
		if(dTmp1<dTmp3) {
			MAT_SET(D,pt2,0,dTmp1,M);
			MAT_SET(links,pt2,0,1,M);
		}
		else {
			MAT_SET(D,pt2,0,dTmp3,M);
			MAT_SET(links,pt2,0,3,M);
		}

		//pt1	= 0;
		//mexPrintf("links(%2d,%2d)=%d,  D(%2d,%2d)=%lf \n", 
		//	pt1,pt2, MAT_GET(links,pt2,pt1,M), pt1,pt2, MAT_GET(D,pt2,pt1,M));
	}

	for(pt1=1;pt1<M;++pt1)
	{
		dTmp1	= MAT_GET(A,0,pt1,M)+pt1*pen1;
		dTmp2	= MAT_GET(D,0,pt1-1,M)+pen1;
		if(dTmp1<dTmp2) {
			MAT_SET(D,0,pt1,dTmp1,M);
			MAT_SET(links,0,pt1,1,M);
		}
		else {
			MAT_SET(D,0,pt1,dTmp2,M);
			MAT_SET(links,0,pt1,2,M);
		}

		//pt2	= 0;
		//mexPrintf("links(%2d,%2d)=%d,  D(%2d,%2d)=%lf \n", 
		//	pt1,pt2,MAT_GET(links,pt2,pt1,M),  pt1,pt2, MAT_GET(D,pt2,pt1,M));
	}

	//- DP looping
	for(pt1=1;pt1<M;++pt1)
	{
		for(pt2=1;pt2<N;++pt2)
		{
			dTmp1	= MAT_GET(D,pt2-1,pt1-1,M) + MAT_GET(A,pt2,pt1,M);
			dTmp2	= MAT_GET(D,pt2,pt1-1,M) + pen1;
			dTmp3	= MAT_GET(D,pt2-1,pt1,M) + pen1;
			if(dTmp1<=dTmp2 && dTmp1<=dTmp3) {
				MAT_SET(D,pt2,pt1,dTmp1,M);
				MAT_SET(links,pt2,pt1,1,M);
			}
			else if(dTmp2<=dTmp3) {
				MAT_SET(D,pt2,pt1,dTmp2,M);
				MAT_SET(links,pt2,pt1,2,M);
			}
			else {
				MAT_SET(D,pt2,pt1,dTmp3,M);
				MAT_SET(links,pt2,pt1,3,M);
			}
		}
	}


	//- Get the mapping result
	int	OCL	= 4*N;		// ocllusion using this index
	for(pt1=0;pt1<M;++pt1)
		C[pt1]	= OCL;

	T 	= MAT_GET(D,N-1,M-1,M);		//D(M,N);
	if(T < uplimit)	
	{	
		pt1 	= M-1;
		pt2 	= N-1;
		while(pt1>=0 && pt2>=0)
		{
			switch(MAT_GET(links,pt2,pt1,M))	//links(pt1,pt2)
			{
			case 1:
				C[pt1]	= pt2;
				--pt1;
				--pt2;
				break;
			case 2:
				//C[pt1]	= OCL;
				--pt1;
				break;
			case 3:
				--pt2;
				break;
			default:
				printf("links[pt1,pt2]=%d, FAILED!!",MAT_GET(links,pt2,pt1,M));
				return false;
			}

			//mexPrintf("pt1=%d, pt2=%d, link(pt1,pt2)=%d, D(pt1,pt2)=%lf \n", 
			//	pt1,pt2, MAT_GET(links,pt2,pt1,M), MAT_GET(D,pt2,pt1,M));
		}
	}
	else 
	{// terminate before final result
		printf("Terminate without computing C,  T=%lf",T);
		bSucc	= false;
	}
	
	return bSucc;
}



/*/ -----------------------------------------------------------------------------
// This code is un-optimized
bool DPMatchingMultiStart (	int *C, double &T_best,		// output
							double *A, double thre,		// input
							int	M, int N,				// A is MxN matrix
							int	n_start,
							int n_search)
{
	double *A2	= new double[M*2*N];
	bool bSucc	= false;
	memcpy(A2,A,M*N*sizeof(double));
	memcpy(A2+M*N,A,M*N*sizeof(double));

	int		*CC	= new int[M];		// !!!! can be optimized !
	double	TT;
	int		id_best;
	id_best	= -1;
	T_best	= 4*N;

	// try different start points
	int	id_start, iS, iS1, iS2, dS;
	for(iS=0;iS<n_start;++iS)
	{
		id_start	= ROUND(N*iS/(double)n_start);
		bSucc		= DPMatchingFixStartPoint(CC, TT, A2+id_start*M, thre, M, N);
		if(TT<T_best) {
			T_best 	= TT;
			id_best	= id_start;
			memcpy(C,CC,M*sizeof(int));

		}

		for(dS=1;dS<n_search;++dS)	
		{
			// forward
			iS1	= id_start+dS;
			if(iS1>N)	iS1-=N;
			bSucc = DPMatchingFixStartPoint(CC, TT, A2+iS1*M, thre, M, N);
			if(TT<T_best) {
				T_best 	= TT;
				id_best	= iS1;
				//printf("+ id_start=%d, iS1=%d, dS=%d\n", id_start,iS1,dS);
				memcpy(C,CC,M*sizeof(int));
			}
		
			// backward
			iS2	= id_start-dS;
			if(iS2<0)	iS2+=N;
			bSucc	= DPMatchingFixStartPoint(CC, TT, A2+iS2*M, thre, M, N);
			if(TT<T_best) {
				T_best 	= TT;
				id_best	= iS2;
				//printf("- id_start=%d, iS2=%d, dS=%d\n", id_start,iS2,dS);
				memcpy(C,CC,M*sizeof(int));
			}
		}
	}

	// adjust indices
	for(int ii=0;ii<M;++ii)
	{
		if(C[ii]<N)	{		// not occluded
			C[ii] += id_best;
			if(C[ii]>=N)	C[ii] -= N;
		}
	}

	delete	[]A2;
	delete	[]CC;
	return true;
}

/*-------------------------------------------------------------------*/	
bool DPMatchingMultiStart (	int *C, double &T_best,		// output
							double *A, double thre,		// input
							int	M, int N,				// A is MxN matrix
							int	n_start,
							int n_search)
{
	double *A2	= new double[M*2*N];
	bool bSucc	= false;
	memcpy(A2,A,M*N*sizeof(double));
	memcpy(A2+M*N,A,M*N*sizeof(double));

	int		*CC	= new int[M*N];
	double	TT;
	int		id_best;

	id_best		= -1;
	T_best		= 4000*N;
	n_start	= MIN(N,n_start);

	// try different start points
	int	id_start, iS, iS1, iS2, dS;
	for(iS=0;iS<n_start;++iS)
	{
		id_start	= ROUND(N*iS/(double)n_start);
		bSucc		= DPMatchingFixStartPoint(CC+id_start*M, TT, A2+id_start*M, thre, M, N);
		if(TT<T_best) {
			T_best 	= TT;
			id_best	= id_start;
		}

		for(dS=1;dS<n_search;++dS)	
		{
			// forward
			iS1	= id_start+dS;
			if(iS1>N)	iS1-=N;
			bSucc = DPMatchingFixStartPoint(CC+iS1*M, TT, A2+iS1*M, thre, M, N);
			if(TT<T_best) {
				T_best 	= TT;
				id_best	= iS1;
			}
		
			// backward
			iS2	= id_start-dS;
			if(iS2<0)	iS2+=N;
			bSucc	= DPMatchingFixStartPoint(CC+iS2*M, TT, A2+iS2*M, thre, M, N);
			if(TT<T_best) {
				T_best 	= TT;
				id_best	= iS2;
			}
		}
	}

	// adjust indices 
	if(id_best<0)	printf("\n\tId_best=%d !!!!!!!\n",id_best);

	memcpy(C,CC+id_best*M,M*sizeof(int));				
	for(int ii=0;ii<M;++ii)
	{
		if(C[ii]<N)	{		// not occluded
			C[ii] += id_best;
			if(C[ii]>=N)	C[ii] -= N;
		}
	}

	delete	[]A2;
	delete	[]CC;
	return true;
}


/* -----------------------------------------------------------------------------
		 Circular mapping
	function [C,T]=DPMatchingCircular(A, thre)
/*-------------------------------------------------------------------*/	
bool DPMatchingCircular(int *C, double &T,			// output
						double *A, double thre,		// input
						int	M, int N,				// A is MxN matrix
						int n_search)
{
	double *A2	= new double[M*2*N];
	bool bSucc	= false;
	memcpy(A2,A,M*N*sizeof(double));
	memcpy(A2+M*N,A,M*N*sizeof(double));

	int		*CC	= new int[M];
	double	TT	= 1000000;
	int		id_best	= 0;
	T	= 100000;

	// try different start points
	bSucc	= DPMatchingFixStartPoint(C, T, A2, thre, M, N);
	id_best		= 0;
	int	iS1, iS2;
	//n_search=0;
	for(iS1=1;iS1<n_search;++iS1)	
	{
		bSucc	= DPMatchingFixStartPoint(CC, TT, A2+iS1*M, thre, M, N);
		if(TT<T) {
			T 	= TT;
			id_best	= iS1;
			memcpy(C,CC,M*sizeof(int));
		}

	
		// another direction
		iS2	= N-iS1;
		bSucc	= DPMatchingFixStartPoint(CC, TT, A2+iS2*M, thre, M, N);
		if(TT<T) {
			T 	= TT;
			id_best	= iS2;
			memcpy(C,CC,M*sizeof(int));
		}//*/
	}


	// adjust indices
	for(int ii=0;ii<M;++ii)
	{
		if(C[ii]<N)	{		// not occluded
			C[ii] += id_best;
			if(C[ii]>=N)	C[ii] -= N;
		}
	}

	delete	[]A2;
	delete	[]CC;
	return true;
}


/*-------------------------------------------------------------------
	C 	- the optimal assignment.
	T 	- the cost of the optimal assignment.
	A 	- a square cost matrix.
	thre- use average*thre as the occlusion
*/	
bool DPMatchingOneContourOpen(	int *C, double &T,			// output
								double *A, double thre,		// input
								int	M, int N				// A is MxN matrix
								)
{
	double	uplimit = 1000000;
	double	pen1	= thre;
	bool	bSucc	= true;
	int	ii,jj;
	double	dTmp,dTmp1,dTmp2,dTmp3;

	double	*D 		= new double[M*N];		// DP matrix
	int		*links	= new int[M*N];

		
	//- initialization
	dTmp	= MAT_GET(A,0,0,M);
	if(dTmp<pen1) {							// MAT_SET(D,1,1,dTmp,M);
		MAT_SET(D,0,0,dTmp,M);
		MAT_SET(links,0,0,1,M);
	}
	else {
		MAT_SET(D,0,0,pen1,M);
		MAT_SET(links,0,0,2,M);
	}

	for(jj=1;jj<N;jj++)
	{
		dTmp1	= MAT_GET(A,jj,0,M)+(jj-1)*pen1;
		dTmp2	= MAT_GET(D,jj-1,0,M)+pen1;
		if(dTmp1<dTmp2) {
			MAT_SET(D,jj,0,dTmp1,M);
			MAT_SET(links,jj,0,1,M);
		}
		else {
			MAT_SET(D,jj,0,dTmp2,M);
			MAT_SET(links,jj,0,2,M);
		}
	}

	for(ii=1;ii<M;++ii)
	{
		dTmp1	= MAT_GET(A,0,ii,M)+(ii-1)*pen1;
		dTmp2	= MAT_GET(D,0,ii-1,M)+pen1;
		if(dTmp1<dTmp2) {
			MAT_SET(D,0,ii,dTmp1,M);
			MAT_SET(links,0,ii,1,M);
		}
		else {
			MAT_SET(D,0,ii,dTmp2,M);
			MAT_SET(links,0,ii,2,M);
		}
	}

	//- DP looping
	for(ii=1;ii<M;++ii)
	{
		for(jj=1;jj<N;++jj)
		{
			dTmp1	= MAT_GET(D,jj-1,ii-1,M) + MAT_GET(A,jj,ii,M);
			dTmp2	= MAT_GET(D,jj,ii-1,M) + pen1;
			dTmp3	= MAT_GET(D,jj-1,ii,M) + pen1;
			if(dTmp1<=dTmp2 && dTmp1<=dTmp3) {
				MAT_SET(D,jj,ii,dTmp1,M);
				MAT_SET(links,jj,ii,1,M);
			}
			else if(dTmp2<=dTmp3) {
				MAT_SET(D,jj,ii,dTmp2,M);
				MAT_SET(links,jj,ii,2,M);
			}
			else {
				MAT_SET(D,jj,ii,dTmp3,M);
				MAT_SET(links,jj,ii,3,M);
			}
		}
	}


	//- Get the mapping result
	int	OCL	= 4*N;		// ocllusion using this index
	for(ii=0;ii<M;++ii)
		C[ii]	= OCL;

	T 	= MAT_GET(D,N-1,M-1,M);		//D(M,N);
	if(T < uplimit)	
	{	
		ii 	= M-1;
		jj 	= N-1;
		while(ii>=0 && jj>=0)
		{
			switch(MAT_GET(links,jj,ii,M))	//links(ii,jj)
			{
			case 1:
				C[ii]	= jj;
				--ii;
				--jj;
				break;
			case 2:
				--ii;
				break;
			case 3:
				--jj;
				break;
			default:
				printf("links[ii,jj]=%d, FAILED!!",MAT_GET(links,jj,ii,M));
				delete	[]D;
				delete	[]links;
				return false;
			}
		}
	}
	else 
	{// terminate before final result
		printf("Terminate without computing C,  T=%lf",T);
		bSucc	= false;
	}
	
	delete	[]D;
	delete	[]links;
	return bSucc;
}





/*-----------------------------------------------------------------------
	 functions 



/*-----------------------------------------------------------------------
[dis,cost_mat] = dist_bw_sc_C( sc1, sc2, n_dist, n_theta, n_dis_type)
	
	sc1,sc2	: input shape context of two object, 
			  each COLUMN of sc1,sc2 is a chape contex feature at a given point.
	
	type	:  0 - using the "Hausdorff" distance as in Belongie's paper
			   1 - find the minimum distance with respect all possible global
					rotations
		  
% Compute cost matrix
sc1			= sc1/mean(sum(sc1,1));
sc2			= sc2/mean(sum(sc2,1));
costmat		= hist_cost(sc1,sc2);		% in C code to avoid memory insufficiency

% calculate shape context cost
[a1,b1]=min(costmat,[],1);
[a2,b2]=min(costmat,[],2);
sc_cost=max(mean(a1),mean(a2));
  
 ------------------------------------------------------------------------*/


void compu_dist_matrix( double *pSC1, double *pSC2, 
					    int nSamp1, int nSamp2, int nBin1, int nBin2,
						int n_dist, int n_theta, int nType,
						double *pDist, double *cost_mat
					   )
{
	

	
	double	*cost_mat_tmp;

	double	*sc1,*sc2,*pCur;
	double	*pRowDist;
	
	
	int		r,c,k,k1,k2,ia,id,ir, n_delta, N;
	double	dis,tmp,minDis,dist1,dist2,res_dis;


   



	
	


	/*------------------------------------------------
	type	:  0 - using the "Hausdorff" distance as in Belongie's paper
			   1 - find the minimum distance with respect all possible global
					rotations	/
	printf("nType=%d\n",nType);	/**/


	pRowDist = (double*)malloc(nSamp1*sizeof(double));
	if(pRowDist==0)	printf("error when malloc!");

	switch(nType)
	{
	case 1:
		/* compute the distance between the r-th point in shape 1
									and the c-th point in shape 2 */
		cost_mat_tmp	= (double*)malloc(nSamp1*nSamp2*sizeof(double));
		res_dis	= 10000;
		for(ir=0;ir<n_theta;ir+=2)
		{
			/* printf("ir=%d\n",ir);	*/
			for(r=0;r<nSamp1;r++)		pRowDist[r] = 10000;

			n_delta	= ir*n_dist;
			dist2	= 0;
			for(c=0;c<nSamp2;c++)
			{
				sc2		= pSC2+c*nBin2;
				minDis	= 10000;
				for(r=0;r<nSamp1;r++)
				{
					sc1	= pSC1+r*nBin1;
					dis	= 0;

					for(k1=0;k1<nBin1;k1++)
					{
						k2	= k1+n_delta;
						if(k2>=nBin2)	
							k2 = k2-nBin2;
						tmp	= sc1[k1]+sc2[k2];
						if(tmp>0.000001)
							dis	+= (sc1[k1]-sc2[k2])*(sc1[k1]-sc2[k2])/tmp;
					}
					
					MAT_SET(cost_mat_tmp,c,r,dis,nSamp1);
					if(dis<minDis)					minDis		= dis;
					if(dis<pRowDist[r])				pRowDist[r] = dis;
				}
				dist2	+= minDis;
			}
			dist2 /= nSamp2;

			/* compute the shape context distance */
			dist1	= 0;	
			for(r=0;r<nSamp1;r++)		dist1 += pRowDist[r];
			dist1 /= nSamp1;		

			minDis	= (dist1<dist2)? dist2:dist1;

			if(minDis<res_dis) {
				res_dis	= minDis;
				memcpy(cost_mat,cost_mat_tmp,nSamp1*nSamp2*sizeof(double));
			}
		
		}/* of ir=0..n_theta */
		free(cost_mat_tmp);
	break;
	

	case 0:
	default:
		/*
		N	= nBin1*nSamp1;
		for(r=0; r<N; ++r)
			if(pSC1[r]<0.00001)		pSC1[r]=0.00001;
		N	= nBin2*nSamp1;
		for(c=0; c<N; ++c)
			if(pSC2[c]<0.00001)		pSC2[c]=0.00001;
		//*/

		/* compute the distance between the r-th point in shape 1
									and the c-th point in shape 2 */
		for(r=0;r<nSamp1;r++)		pRowDist[r] = 10000;

		dist2	= 0;
		for(c=0;c<nSamp2;c++)
		{
			sc2		= pSC2+c*nBin2;
			minDis	= 10000;
			for(r=0;r<nSamp1;r++)
			{
				sc1	= pSC1+r*nBin1;
				dis	= 0;
				for(k=0;k<nBin1;k++)
				{
					tmp	= sc1[k]+sc2[k];
					if(tmp>0.000001)
						dis	+= (sc1[k]-sc2[k])*(sc1[k]-sc2[k])/tmp;
				}

				MAT_SET(cost_mat,c,r,dis,nSamp1);
				if(dis<minDis)					minDis		= dis;
				if(dis<pRowDist[r])				pRowDist[r] = dis;
			}
			dist2	+= minDis;
		}
		dist2 /= nSamp2;

		/* compute the shape context distance */
		dist1	= 0;	
		for(r=0;r<nSamp1;r++)		dist1 += pRowDist[r];
		dist1 /= nSamp1;		

		res_dis	= (dist1<dist2)?dist2:dist1;
		break;
	}


	/* return */
	free(pRowDist);
	*pDist	= res_dis;
	
}


