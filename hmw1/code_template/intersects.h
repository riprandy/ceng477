#ifndef INTERSECT_H
#define INTERSECT_H

#include "parser.h"

struct ray
{
    parser::Vec3f start;
    parser::Vec3f end; // direction, normalized
};

// Sphere
bool intersect_sphere(const ray &r, const parser::Sphere &sphere, const parser::Scene &scene, float &t);

// Plane
bool intersect_plane(const ray &r, const parser::Plane &plane, const parser::Scene &scene, float &t);

// Triangle
bool intersect_triangle(const ray &r, const parser::Triangle &tri, const parser::Scene &scene, float &t, parser::Vec3f &normal);

// Optional: Mesh, Cylinder
bool intersect_mesh(const ray &r, const parser::Mesh &mesh, const parser::Scene &scene, float &t, parser::Vec3f &normal);
bool intersect_cylinder(const ray &r, const parser::Cylinder &cyl, const parser::Scene &scene, float &t, parser::Vec3f &normal);

#endif
