#ifndef	__RECTANGLE_2D_H
#define	__RECTANGLE_2D_H

#include <certh_libs/Point2D.h>

#include <math.h>

namespace certh_libs {

class Rectangle2D 
{
protected:

    Point2D tl;		// top left corner (origin)
    Point2D br;		// bottom right corner (corner)

 public:

    Rectangle2D(double left=0, double top=0, double width=0, double height=0);
    Rectangle2D(const Point2D&, const Point2D&);
    Rectangle2D(const Rectangle2D&);

    Point2D origin() const	{ return tl; }
    Point2D corner() const	{ return br; }
    Point2D topLeft() const	{ return tl; }
    Point2D topCenter() const	{ return Point2D((br.x()+tl.x())/2,tl.y()); }
    Point2D topRight() const	{ return Point2D(br.x(),tl.y()); }
    Point2D rightCenter() const	{ return Point2D(br.x(),(br.y()+tl.y())/2); }
    Point2D bottomRight() const	{ return br; }
    Point2D bottomCenter() const	{ return Point2D((br.x()+tl.x())/2,br.y()); }
    Point2D bottomLeft() const	{ return Point2D(tl.x(),br.y()); }
    Point2D leftCenter() const	{ return Point2D(tl.x(),(br.y()+tl.y())/2); }
    Point2D center() const	{ return Point2D((br.x()+tl.x())/2,(br.y()+tl.y())/2); }
    Point2D extent() const	{ return Point2D(br.x()-tl.x(),br.y()-tl.y()); }
    double area() const		{ return width()*height(); }
    double width() const		{ return br.x()-tl.x() ; }
    double height() const		{ return br.y()-tl.y() ; }

    Rectangle2D operator&&(const Rectangle2D&) const;	// intersection
    Rectangle2D operator||(const Rectangle2D&) const;	// union

    void operator+=(const Point2D&);			// translate
    void operator-=(const Point2D&);

    bool contains(const Point2D &) const;
    bool contains(const Rectangle2D &) const;

    bool intersects(const Rectangle2D &) const;

    void moveTo(const Point2D &);

    friend std::ostream & operator << (std::ostream &strm, const Rectangle2D &r) ;
};

inline Rectangle2D::Rectangle2D(double left, double top, double width, double height)
{
    tl = Point2D(left, top);
    br = Point2D(left + width, top + height);
}

inline Rectangle2D::Rectangle2D(const Point2D& o, const Point2D& c)
{
    tl = o;
    br = c;
}

inline Rectangle2D::Rectangle2D(const Rectangle2D& r) : tl(r.tl), br(r.br) {}

inline Rectangle2D Rectangle2D::operator&&(const Rectangle2D& r) const
{
    return Rectangle2D(Point2D(std::max(tl.x(), r.tl.x()), std::max(tl.y(), r.tl.y())),
                       Point2D(std::min(br.x(), r.br.x()), std::min(br.y(), r.br.y()))) ;
}


inline Rectangle2D Rectangle2D::operator||(const Rectangle2D& r) const
{
    return Rectangle2D(Point2D(std::min(tl.x(), r.tl.x()), std::min(tl.y(), r.tl.y())),
                       Point2D(std::max(br.x(), r.br.x()), std::max(br.y(), r.br.y()))) ;
}

inline void Rectangle2D::operator+=(const Point2D& p)
{
    tl += p;
    br += p;
}

inline void Rectangle2D::operator-=(const Point2D& p)
{
    tl -= p;
    br -= p;
}

inline bool Rectangle2D::contains(const Point2D& p) const
{
    return (tl.x() <= p.x() && tl.y() <= p.y() ) && (p.x() <= br.x() && p.y() <= br.y());
}

inline bool Rectangle2D::contains(const Rectangle2D& r) const
{
    return ( contains(r.tl) && contains(r.br) );
}

inline bool Rectangle2D::intersects(const Rectangle2D& r) const
{
    if ( std::max(tl.x(), r.tl.x()) < std::min(br.x(), r.br.x()) &&
         std::max(tl.y(), r.tl.y()) < std::min(br.y(), r.br.y()) ) return true ;
    return false;
}

inline void Rectangle2D::moveTo(const Point2D& p)
{
    br += p-tl;
    tl = p;
}

inline std::ostream & operator << (std::ostream &strm, const Rectangle2D &r)
{
    return strm << "{ " << r.topLeft() << " " << r.bottomRight() << " }" ;
}
/////////////////////////////////////


} // namespace cpm

#endif
