//extern "C" // required when using C++ compiler

#include <iostream>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Vector_2.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Triangle_2 Triangle_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Vector_2 Vector_2;

struct Point_mass {
    Point_2 pos;
    double mass;
    Point_mass(const Point_2 & p, double m): pos(p), mass(m) {}
};

Point_2 centre_of_mass(Point_mass *cur, Point_mass *beyond)
{
    Point_2 ORIGIN (0.0, 0.0);
    Vector_2 sumv(0.0, 0.0);
    double sumw = 0.0;
    for ( ; cur != beyond; ++cur) {
        sumv = sumv + (cur->pos - ORIGIN) * cur->mass;
        sumw += cur->mass;
    }
    return ORIGIN + sumv/sumw;
}


const int N = 6;
/* 
 * It MUST be declare as main, share object could only link to main for c++, other will be not found.
 * return value: number of inresection points, and its value
*/
int main(double pts_Tri1[], double pts_Tri2[], double pts_data[], double pt_centre[]) // 
{
	unsigned int pt_num = 0; // number of intersection points.
	
	Point_2 A(pts_Tri1[0], pts_Tri1[1]);
	Point_2 B(pts_Tri1[2], pts_Tri1[3]);
	Point_2 C(pts_Tri1[4], pts_Tri1[5]);

	Point_2 D(pts_Tri2[0], pts_Tri2[1]);
	Point_2 E(pts_Tri2[2], pts_Tri2[3]);
	Point_2 F(pts_Tri2[4], pts_Tri2[5]);

	Triangle_2 Tri1(A, B, C);
	Triangle_2 Tri2(D, E, F);

	if ( ! CGAL::do_intersect(Tri1, Tri2) ) {
		std::cout << "empty" << std::endl;
	}
	else {
		CGAL::Object result = CGAL::intersection(Tri1, Tri2);
		Triangle_2 triangle;
		std::vector<Point_2> poly_point; // not a Polygon_2

		if (CGAL::assign(triangle, result)) {
			//std::cout << "triangle." << triangle << std::endl;
			pt_num  = 3;
			for (unsigned i=0; i < pt_num; i ++){
				pts_data[2*i] = triangle.vertex(i).x();
				pts_data[2*i+1] = triangle.vertex(i).y();
			}		
		}
		else if (CGAL::assign(poly_point, result)) {
			pt_num  = poly_point.size();
			//std::cout << "a polygon." << std::endl;
			for (unsigned i =0; i < pt_num; i ++){
				pts_data[2*i] = poly_point.at(i).x();
				pts_data[2*i+1] = poly_point.at(i).y();
			}
		}
		else {
			std::cout << "unknown!" << std::endl;
		}
	}
    std::cout << "number of intesection points " << pt_num << std::endl;

    // find central of mass
    if (pt_num >2){
        std::cout << "calculating centre of mass" << std::endl;
        Point_mass points[N] = {
           Point_mass(Point_2(pts_data[ 0],pts_data[ 1]), 1),
           Point_mass(Point_2(pts_data[ 2],pts_data[ 3]), 1),
           Point_mass(Point_2(pts_data[ 4],pts_data[ 5]), 1),
           Point_mass(Point_2(pts_data[ 6],pts_data[ 7]), 1),
           Point_mass(Point_2(pts_data[ 8],pts_data[ 9]), 1),
           Point_mass(Point_2(pts_data[10],pts_data[11]), 1)
       };
       Point_2 centre = centre_of_mass(points, points+pt_num);
       std::cout << "The centre of mass is: (" << centre.x() <<", "<< centre.y() <<")" << std::endl;
       pt_centre[0] = centre.x();
       pt_centre[1] = centre.y();
    }
    

	return pt_num;
}
