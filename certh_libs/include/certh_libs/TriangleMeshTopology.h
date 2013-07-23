#ifndef _TRIANGLE_MESH_TOPOLOGY_H_
#define _TRIANGLE_MESH_TOPOLOGY_H_

#include <map>
#include <vector>
#include <boost/pool/object_pool.hpp> 

namespace certh_libs {

    class TriangleMeshTopology
	{
	public:

		TriangleMeshTopology() ;
		TriangleMeshTopology(const TriangleMeshTopology &other) ;

		~TriangleMeshTopology() ;

		// Adds a face to the mesh. Takes as input the IDs of the vertices.
		// Returns the face ID. The order of vertices should always be clockwise

        int addFace(int v1, int v2, int v3) ;
        void deleteFace(int faceID) ;

        void vtxGetNeighborVertices(int vid, std::vector<int> &vtcs) const ;
        void vtxGetNeighborFaces(int vid, std::vector<int> &faces) const ;

		// f1 is the face on the right of the oriented edge v1->v2
        void edgeGetAdjFaces(int v1, int v2, int &f1, int &f2) const ;
		// Get the vertices opposite and on the sides of the edge. a1 is on the right.
        void edgeGetAdjVertices(int v1, int v2, int &a1, int &a2) const ;

        void faceGetVertices(int fid, int &v1, int &v2, int &v3) const ;
        void faceGetVertices(int fid, std::vector<int> &coordIndex) const ;

        void collapseEdge(int v1, int v2) ;

        void printTopology() ;

        unsigned getNumFaces() const { return faceMap.size() ; }
        unsigned getNumVertices() const { return vertexMap.size() ; }
        unsigned getNumEdges() const { return edgeMap.size() ; }

        void getEdge(int idx, int &v1, int &v2, int &f1, int &f2) const;

        bool vtxExists(int vid) const { return vertexMap.find(vid) != vertexMap.end() ; }


		// bool isBoundaryEdge(int vtx) ;

	private:

        struct HalfEdge {
            HalfEdge() {}
            HalfEdge *mate ; // The twin half-edge at the opposite direction if not boundary edge.
            HalfEdge *prev ; // The previous half-edge in the face loop in cw direction
            HalfEdge *next ; // The next half-edge in the face loop in cw direction
            int vertex ;	 // The ID of the vertex from which the half edge starts
            int face ;		 // The ID of the face that this half-edge belongs to
        } ;

        HalfEdge *findEdge(int v1, int v2) const ;

        bool isBoundaryVertex(int vtx) const ;
        bool isBoundaryFace(int vtx) ;

		friend class VertexVertexIterator ;
		friend class VertexFaceIterator ;
        friend class EdgeIterator ;

        HalfEdge *findHalfEdge(int v1, int v2) const ;
        void fixVertexMap(HalfEdge *e1) ;
        void deleteHalfEdge(HalfEdge *) ;

		std::map<int, HalfEdge *> vertexMap ; // list of vertices
		std::map<int, HalfEdge *> faceMap ;	// list of faces
		std::map<int, HalfEdge *> edgeMap;	// list of edges

		typedef std::pair<int, int> EdgeKey ;
		typedef std::map<EdgeKey, HalfEdge *> HalfEdgeList ; // list of half edges indexed by source
		// and target vertices

		HalfEdgeList emap ;

		boost::object_pool<HalfEdge> alloc ;

        HalfEdge *createHalfEdge(int v1, int v2, int face) ;

		HalfEdge *sEdge, *eEdge ;
		int cFace, nEdges, nFaces ;

};

class VertexVertexIterator
{
public: 
	VertexVertexIterator(const TriangleMeshTopology &topo, int vtx) ;

	VertexVertexIterator &operator= (const VertexVertexIterator &other) ;

	bool operator == (const VertexVertexIterator &other) const ;
	bool operator != (const VertexVertexIterator &other) const ;
	VertexVertexIterator & operator ++ () ;
	
	int operator * () const ;
    operator bool () const ;

private:

	TriangleMeshTopology::HalfEdge *sEdge, *cEdge ;
 	
} ;

class VertexFaceIterator
{
public: 
	VertexFaceIterator(const TriangleMeshTopology &topo, int vtx) ;

	VertexFaceIterator &operator= (const VertexFaceIterator &other) ;

	bool operator == (const VertexFaceIterator &other) const ;
	bool operator != (const VertexFaceIterator &other) const ;
	VertexFaceIterator & operator ++ () ;
	
	int operator * () const ;
    operator bool () const ;

private:

	TriangleMeshTopology::HalfEdge *sEdge, *cEdge ;
 	
} ;

class EdgeIterator
{
public:
    EdgeIterator(const TriangleMeshTopology &topo) ;

    EdgeIterator &operator= (const EdgeIterator &other) ;

    bool operator == (const EdgeIterator &other) const ;
    bool operator != (const EdgeIterator &other) const ;
    EdgeIterator & operator ++ () ;

    std::pair<int, int> operator * () const ;
    operator bool () const ;

private:

    std::map<int, TriangleMeshTopology::HalfEdge *>::const_iterator it, last ;
};

}

#endif
