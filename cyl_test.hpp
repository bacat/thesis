#ifndef CYL_TEST_HPP_INCLUDED
#define CYL_TEST_HPP_INCLUDED

// Assume that classes are already given for the objects:
//     Point and Vector with
//          coordinates {float x, y, z;} (z=0  for 2D)
//          appropriate operators for:
//               Point  = Point ± Vector
//               Vector = Point - Point
//               Vector = Scalar * Vector
//     Line with defining endpoints {Point P0, P1;}
//     Segment with defining endpoints {Point P0, P1;}
//===================================================================

// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)     sqrt(dot(v,v))     // norm = length of  vector
#define d(u,v)      norm(u-v)          // distance = norm of difference

struct Vector{

    float x, y, z;
};

Vector operator *(float scalar, const Vector& vec)
{
    return Vector{scalar * vec.x, scalar * vec.y, scalar * vec.z};
}

struct Point{

    float x, y, z;

    Point operator+(const Vector& vec)
    {
        return Point{this->x + vec.x, this->y + vec.y, this->z + vec.z};
    }

    Point operator-(const Vector& vec)
    {
        return Point{this->x - vec.x, this->y - vec.y, this->z - vec.z};
    }

    Vector operator-(const Point& point)
    {
        return Vector{this->x - point.x, this->y - point.y, this->z - point.z};
    }
};

struct Line{Point P0, P1;};
struct Plane{Point V0; Vector n;};

// dist_Point_to_Line(): get the distance of a point to a line
//     Input:  a Point P and a Line L (in any dimension)
//     Return: the shortest distance from P to L

float dist_Point_to_Line( Point P, Line L)
{
     Vector v = L.P1 - L.P0;
     Vector w = P - L.P0;

     double c1 = dot(w,v);
     double c2 = dot(v,v);
     double b = c1 / c2;

     Point Pb = L.P0 + b * v;
     return d(P, Pb);
}

float dist_Point_to_Plane(glm::vec3 normal, glm::vec3 point, glm::vec3 point_on_plane)
{
    return (normal.x * point_on_plane.x + normal.y * point_on_plane.y + normal.z * point_on_plane.z - normal.x * point.x - normal.y * point.y - normal.z * point.z) / glm::length(normal);
}

// dist_Point_to_Plane(): get distance (and perp base) from a point to a plane
//    Input:  P  = a 3D point
//            PL = a  plane with point V0 and normal n
//    Output: *B = base point on PL of perpendicular from P
//    Return: the distance from P to the plane PL
float dist_Point_to_Plane_original( Point P, Plane PL, Point* B)
{
    float    sb, sn, sd;

    sn = -dot( PL.n, (P - PL.V0));
    sd = dot(PL.n, PL.n);
    sb = sn / sd;

    *B = P + sb * PL.n;
    return d(P, *B);
}


#endif // CYL_TEST_HPP_INCLUDED
