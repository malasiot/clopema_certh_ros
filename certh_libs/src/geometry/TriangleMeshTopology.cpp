#include <certh_libs/TriangleMeshTopology.h>

//#include "Mesh3D.h"

#include <memory>
#include <cassert>
#include <iostream>
using namespace std ;

#if defined(__MINGW32__)
#define _int64 long long int
#endif

namespace certh_libs {

	TriangleMeshTopology::TriangleMeshTopology(const TriangleMeshTopology &other)
	{
		int i, j ;

        for( i=0 ; i<other.getNumFaces() ; i++ )
		{
			int v1, v2, v3 ;

            other.faceGetVertices(i, v1, v2, v3) ;

            addFace(v1, v2, v3) ;
		}
	}

	TriangleMeshTopology::TriangleMeshTopology()
	{
		nEdges = nFaces = 0 ;
		cFace = 0 ;

	}

	TriangleMeshTopology::~TriangleMeshTopology()
	{

	}

    TriangleMeshTopology::HalfEdge *TriangleMeshTopology::createHalfEdge(int v1, int v2, int face)
	{
		HalfEdge *edge = alloc.malloc() ;

		edge->mate = NULL ;
		edge->face = face ;
		edge->next = 0 ;
		edge->prev = 0 ;

		HalfEdgeList::iterator it1 = emap.find(EdgeKey(v1, v2)) ;
		HalfEdgeList::iterator it2 = emap.find(EdgeKey(v2, v1)) ;

		if ( it1 != emap.end() && it2 != emap.end() ) // error - non manifold or duplicate
		{
			return NULL ;
		}
		else if ( it1 != emap.end() ) // we found a half edge v1->v2 - error ccw direction ?
		{
			return NULL ;
		}
		else if ( it2 != emap.end() ) // we found a half edge v2->v1
		{
			edge->vertex = v1 ;
			edge->mate = (*it2).second ;
			(*it2).second->mate = edge ;
			emap[EdgeKey(v1, v2)] = edge ;
		}
		else // none found 
		{
			edge->vertex = v1 ;
			edge->mate = NULL ;
			emap[EdgeKey(v1, v2)] = edge ;
			vertexMap[v1] = edge ;
			edgeMap[nEdges++] = edge ;
		}

		return edge ;

	}	

    int TriangleMeshTopology::addFace(int v1, int v2, int v3)
	{
		bool cw = true ;
        HalfEdge *e1 = createHalfEdge(v1, v2, cFace) ;
        HalfEdge *e2 = createHalfEdge(v2, v3, cFace) ;
        HalfEdge *e3 = createHalfEdge(v3, v1, cFace) ;

		if ( !e1 || !e2 || !e3 ) return -1 ;

		e1->next = e2 ;	e2->prev = e1 ;
		e2->next = e3 ; e3->prev = e2 ;
		e3->next = e1 ; e1->prev = e3 ;

		faceMap[cFace] = e1 ;

		int ret = cFace ;

		cFace ++ ;
		nFaces ++ ;

		return ret ;
	}


    void TriangleMeshTopology::vtxGetNeighborVertices(int vid, vector<int> &vtcs) const
	{
		HalfEdge *sEdge = (*vertexMap.find(vid)).second ;

		HalfEdge *pEdge = sEdge, *qEdge ;

		vtcs.push_back(pEdge->next->vertex) ;

		do 
		{
			pEdge = pEdge->next->next ;

			if ( pEdge->mate != sEdge ) vtcs.push_back(pEdge->vertex) ;

			pEdge = pEdge->mate ;
		} while ( pEdge && pEdge != sEdge ) ;

		if ( pEdge == sEdge ) return ;

		qEdge = sEdge->mate ;

		while ( qEdge && qEdge != sEdge->prev)
		{
			qEdge = qEdge->next->next ;

			vtcs.push_back(qEdge->next->vertex) ;

			qEdge = qEdge->mate ;
		}  

	}


	bool TriangleMeshTopology::isBoundaryVertex(int vtx) const
	{
		HalfEdge *sEdge = (*vertexMap.find(vtx)).second ;

		HalfEdge *pEdge = sEdge, *qEdge ;

		do 
		{
			if ( pEdge->mate == NULL ) return true ;

			pEdge = pEdge->next->next ;

			pEdge = pEdge->mate ;
		} while ( pEdge && pEdge != sEdge ) ;

		if ( pEdge == sEdge ) return false ;

		qEdge = sEdge->mate ;

		while ( qEdge && qEdge != sEdge->prev)
		{
			qEdge = qEdge->next->next ;

			if ( qEdge->mate == NULL ) return true ;

			qEdge = qEdge->mate ;
		}  

		return false ;
	}

    void TriangleMeshTopology::vtxGetNeighborFaces(int vid, vector<int> &faces) const
	{
		HalfEdge *sEdge = (*vertexMap.find(vid)).second;

		HalfEdge *pEdge = sEdge, *qEdge ;

		do 
		{
			pEdge = pEdge->next->next ;

			faces.push_back(pEdge->face) ;

			pEdge = pEdge->mate ;
		} while ( pEdge && pEdge != sEdge ) ;


		if ( pEdge == sEdge ) return ;

		qEdge = sEdge->mate ;

		while ( qEdge && qEdge != sEdge->prev)
		{
			qEdge = qEdge->next->next ;

			faces.push_back(qEdge->next->face) ;

			qEdge = qEdge->mate ;
		}  

	}

    void TriangleMeshTopology::edgeGetAdjFaces(int v1, int v2, int &f1, int &f2) const
	{
        HalfEdge *pEdge = findEdge(v1, v2) ;

		if ( !pEdge ) return ;

		f1 = pEdge->face ;
		f2 = (pEdge->mate) ? pEdge->mate->face : -1 ;

		if ( pEdge->vertex == v2 ) swap(f1, f2) ;
	}

    void TriangleMeshTopology::edgeGetAdjVertices(int v1, int v2, int &f1, int &f2) const
	{
        HalfEdge *pEdge = findEdge(v1, v2) ;

		if ( !pEdge ) return ;

		f1 = pEdge->prev->vertex ;
		f2 = (pEdge->mate) ? pEdge->mate->prev->vertex : -1 ;

		if ( pEdge->vertex == v2 ) swap(f1, f2) ;
	}

    void TriangleMeshTopology::faceGetVertices(int fid, std::vector<int> &coordIndex) const
	{
		int v1, v2, v3 ;

        faceGetVertices(fid, v1, v2, v3) ;

		coordIndex.push_back(v1) ;
		coordIndex.push_back(v2) ;
		coordIndex.push_back(v3) ;
	}

    void TriangleMeshTopology::faceGetVertices(int fid, int &v1, int &v2, int &v3) const
	{
		map<int, HalfEdge *>::const_iterator it = faceMap.find(fid) ;

		if ( it == faceMap.end() ) 
		{
			v1 = v2 = v3 = -1 ;
			return ;
		}

		HalfEdge *p = (*it).second ;

		v1 = p->vertex ;
		v2 = p->next->vertex ;
		v3 = p->next->next->vertex ;
	}


    TriangleMeshTopology::HalfEdge *TriangleMeshTopology::findHalfEdge(int v1, int v2) const
	{
		HalfEdgeList::const_iterator it = emap.find(EdgeKey(v1, v2)) ;

		if ( it == emap.end() ) return NULL ;
		else return (*it).second ;
	}

    void TriangleMeshTopology::fixVertexMap(HalfEdge *e1)
	{
		HalfEdge *sEdge = e1, *pEdge = sEdge, *qEdge ;
		int v1 = e1->vertex ;

		bool found = false ;

		do 
		{
			if ( pEdge != e1 ) { 
				vertexMap[v1] = pEdge ; 
				found = true ;
				break ; 
			}
			pEdge = pEdge->next->next->mate ;
		}  while ( pEdge && pEdge != sEdge ) ;

		qEdge = sEdge->mate ;

		while ( !found && qEdge && qEdge != sEdge->prev)
		{
			qEdge = qEdge->next->next ;

			if ( qEdge != e1 ) {
				found = true ;
				vertexMap[v1] = qEdge ;
				break ;
			}

			qEdge = qEdge->mate ;
		}

		if ( !found ) vertexMap.erase(v1) ;
	}

    void TriangleMeshTopology::collapseEdge(int v1, int v2)
	{
        HalfEdge *e = findEdge(v1, v2), *e1, *e2 ;

		if ( !e ) return ;

		if ( e->vertex == v1 ) {
			e1 = e ;
			e2 = e->mate ;
		}
		else
		{
			e2 = e ;
			e1 = e->mate ;
		}

		// delete first face

		if ( e1 )
		{
			HalfEdge *en = e1->next, *ep = e1->prev ;

			if ( ep->mate ) { ep->mate->mate = en->mate ; }
			if ( en->mate ) { en->mate->mate = ep->mate ; }
			faceMap.erase(e1->face) ;
			nFaces -- ;
		}

		// delete second face

		if ( e2 ) 
		{
			HalfEdge *en = e2->next, *ep = e2->prev ;

			if ( ep->mate ) { ep->mate->mate = en->mate ; }
			if ( en->mate ) { en->mate->mate = ep->mate ; }
			faceMap.erase(e2->face) ;
			nFaces -- ;
		}

		// v2 will be deleted so fix references to v2

		if ( e2 && e2->prev->mate ) e2->prev->mate->vertex = v1 ;
		if ( e1 && e1->next->mate ) e1->next->mate->next->vertex = v1 ;

		// Fix vertexMap references

		vertexMap.erase(v2) ;

		// since e1 will be deleted we should make vertex map show to a different vertex
        if ( vertexMap[v1]== e1 ) fixVertexMap(e1) ;

		if ( e1 ) // delete halfedges in one face
		{
			int v3 = e1->prev->vertex ;

            if ( vertexMap[v3] == e1->prev ) fixVertexMap(e1->prev) ;

			HalfEdge *en = e1->next, *ep = e1->prev ;
            deleteHalfEdge(e1) ;
            deleteHalfEdge(en) ;
            deleteHalfEdge(ep) ;
		}


		if ( e2 ) // delete half edges in another face
		{
			int v3 = e2->prev->vertex ;

            if ( vertexMap[v3] == e2->prev ) fixVertexMap(e2->prev) ;

			HalfEdge *en = e2->next, *ep = e2->prev ;
            deleteHalfEdge(e2) ;
            deleteHalfEdge(en) ;
            deleteHalfEdge(ep) ;
		}
	}

    void TriangleMeshTopology::deleteHalfEdge(HalfEdge *edge)
	{
		EdgeKey key(edge->vertex, edge->next->vertex) ;
		emap.erase(key) ;
		alloc.free(edge) ;
	}

    TriangleMeshTopology::HalfEdge *TriangleMeshTopology::findEdge(int i1, int i2) const
	{
		HalfEdgeList::const_iterator it1 = emap.find(EdgeKey(i1, i2)) ;
		HalfEdgeList::const_iterator it2 = emap.find(EdgeKey(i2, i1)) ;

		if ( it1 == emap.end() && it2 == emap.end()) return NULL ;
		else if ( it1 == emap.end() ) return (*it2).second ;
		else return (*it1).second ;
	}

    void TriangleMeshTopology::deleteFace(int fid)
	{
		std::map<int, HalfEdge *>::const_iterator it = faceMap.find(fid) ;

		if ( it == faceMap.end() ) return ;

		HalfEdge *e = (*it).second ;
		int v ;

		v = e->vertex ;
        if ( vertexMap[v] == e ) fixVertexMap(e) ;
		e = e->next ;
		v = e->vertex ;
        if ( vertexMap[v] == e ) fixVertexMap(e) ;
		e = e->next ;
		v = e->vertex ;
        if ( vertexMap[v] == e ) fixVertexMap(e) ;


        deleteHalfEdge(e) ;
        deleteHalfEdge(e->next) ;
        deleteHalfEdge(e->next->next) ;

		faceMap.erase(fid) ;


	}

    void TriangleMeshTopology::printTopology()
	{

		map<int, HalfEdge *>::iterator itv = vertexMap.begin() ;

		cout << "Vertices\n" ;
		while ( itv != vertexMap.end() )
		{
			int vtx = (*itv).first ;
			cout << vtx << endl ;
			++itv ;
		}


		map<int, HalfEdge *>::iterator it = faceMap.begin() ;

		cout << "Faces\n" ;
		while ( it != faceMap.end() )
		{
			int face = (*it).second->face ;
			cout << face << ": " ;

			int v1, v2, v3 ;
            faceGetVertices(face, v1, v2, v3) ;

			cout << v1 << ' ' << v2 << ' ' << v3  ;
			cout << endl ;

			++it ;
		}
		cout << "Edges\n" ;

		int k = 0 ;
		map<int, HalfEdge *>::iterator ite = edgeMap.begin() ;

		while ( ite != edgeMap.end() ) 
		{
			HalfEdge *e = (*ite).second ;

			cout << k << ": " ;
			cout << e->vertex << ' ' ;
			cout << e->next->vertex << endl ;

			++ite ; ++k ;
		}

		cout << "Half Edges\n" ;

		k = 0 ;
		HalfEdgeList::iterator ithe = emap.begin() ;

		while ( ithe != emap.end() ) 
		{
			HalfEdge *e = (*ithe).second ;
			EdgeKey key = (*ithe).first ;

			cout << e << ": " ;
			cout << e->vertex << ' ' ;
			cout << e->next->vertex << ' ' ;
			cout << e->face << ' ' ;
			cout << e->mate << endl ;

			++ithe ; ++k ;
		}
	}

    void TriangleMeshTopology::getEdge(int idx, int &v1, int &v2, int &f1, int &f2) const
	{
		v1 = v2 = f1 = f2 = -1 ;

		map<int, HalfEdge *>::const_iterator it = edgeMap.find(idx) ;

		if ( it == edgeMap.end() ) return ;

		HalfEdge *p = (*it).second ;

		v1 = p->vertex ;
		v2 = p->next->vertex ;
		f1 = p->face ;
		f2 = ( p->mate ) ? p->mate->face : -1 ;
	}


	//////////////////////////////////////////////////////////////////////////

	VertexVertexIterator::VertexVertexIterator(const TriangleMeshTopology &topo, int vtx)
	{
		// Find leftmost edge
		std::map<int, TriangleMeshTopology::HalfEdge *>::const_iterator it = topo.vertexMap.find(vtx) ;

		assert(it != topo.vertexMap.end() ) ;

		sEdge = (*it).second ;

		TriangleMeshTopology::HalfEdge *pEdge = sEdge, *qEdge ;

		do {
			if ( pEdge->mate ) pEdge = pEdge->mate->next ;
			else break ;
		}
		while ( pEdge && pEdge != sEdge ) ;

		sEdge = cEdge = pEdge ;

	}

	VertexVertexIterator & VertexVertexIterator::operator = (const VertexVertexIterator &other)
	{
		sEdge = other.sEdge ;
		cEdge = other.cEdge ;

		return *this ;
	}

	bool VertexVertexIterator::operator == (const VertexVertexIterator &other) const
	{
		return ( sEdge == other.sEdge && cEdge == other.cEdge ) ;
	}

	bool VertexVertexIterator::operator != (const VertexVertexIterator &other) const 
	{
		return !(*this == other) ;
	}

	VertexVertexIterator & VertexVertexIterator::operator ++ ()
	{
		if ( cEdge )
		{
			if ( cEdge == sEdge ) cEdge = cEdge->next ;
			else if ( cEdge->next->mate ) {
				if ( cEdge->next->mate->prev->mate &&
					cEdge->next->mate->prev->mate == sEdge ) cEdge = NULL ;
				else cEdge = cEdge->next->mate->next ;
			}
			else cEdge = NULL ;


		}

		return *this ;

	}


	int VertexVertexIterator::operator * () const 
	{
		return cEdge->next->vertex ;
	}

	VertexVertexIterator::operator bool () const 
	{
		return cEdge != 0 ;
	}

	//////////////////////////////////////////////////////////////////////////////////////


	VertexFaceIterator::VertexFaceIterator(const TriangleMeshTopology &topo, int vtx)
	{

		// Find leftmost edge
		std::map<int, TriangleMeshTopology::HalfEdge *>::const_iterator it = topo.vertexMap.find(vtx) ;

		assert(it != topo.vertexMap.end() ) ;

		sEdge = (*it).second ;

		TriangleMeshTopology::HalfEdge *pEdge = sEdge, *qEdge ;

		do {
			if ( pEdge->mate ) pEdge = pEdge->mate->next ;
			else break ;
		}
		while ( pEdge && pEdge != sEdge ) ;

		sEdge = cEdge = pEdge ;

	}

	VertexFaceIterator & VertexFaceIterator::operator = (const VertexFaceIterator &other)
	{
		sEdge = other.sEdge ;
		cEdge = other.cEdge ;

		return *this ;
	}

	bool VertexFaceIterator::operator == (const VertexFaceIterator &other) const
	{
		return ( sEdge == other.sEdge && cEdge == other.cEdge ) ;
	}

	bool VertexFaceIterator::operator != (const VertexFaceIterator &other) const 
	{
		return !(*this == other) ;
	}

	VertexFaceIterator & VertexFaceIterator::operator ++ ()
	{
		if ( cEdge )
		{
			cEdge = cEdge->next->next->mate ;
			if ( cEdge == sEdge ) cEdge = NULL ;
		}

		return *this ;

	}


	int VertexFaceIterator::operator * () const 
	{
		return cEdge->face ;
	}

	VertexFaceIterator::operator bool () const 
	{
		return cEdge != 0 ;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////

    EdgeIterator::EdgeIterator(const TriangleMeshTopology &topo)
    {
        it = topo.edgeMap.begin() ;
        last = topo.edgeMap.end() ;

    }

    EdgeIterator & EdgeIterator::operator = (const EdgeIterator &other)
    {
        it = other.it ;
        return *this ;
    }

    bool EdgeIterator::operator == (const EdgeIterator &other) const
    {
        return ( it == other.it ) ;
    }

    bool EdgeIterator::operator != (const EdgeIterator &other) const
    {
        return !(*this == other) ;
    }

    EdgeIterator & EdgeIterator::operator ++ ()
    {
        ++it ;
        return *this ;
    }


    std::pair<int, int> EdgeIterator::operator * () const
    {
        TriangleMeshTopology::HalfEdge *he = (*it).second ;
        return std::pair<int, int>(he->vertex, he->next->vertex) ;
    }

    EdgeIterator::operator bool () const
    {
        return it != last ;
    }
}
