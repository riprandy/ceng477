#include "intersects.h"
#include <cmath>
#include <algorithm>
#include "math_utils.h"
#include <vector>
bool intersect_sphere(const ray &r, const parser::Sphere &sphere, const parser::Scene &scene, float &t);
// Sphere
bool intersect_sphere(const ray &r, const parser::Sphere &sphere, const parser::Scene &scene, float &t)
{
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1]; // IDs are 1-indexed, starts from 1.
    parser::Vec3f oc = r.start - center;                                   // origin - center //this works if i do the - operator const

    // Ray: o + t*d
    // Sphere: ||p - c||^2 = r^2 p-> point on the sphere
    // Solve: ||o + t*d - c||^2 = r^2

    float a = r.end.x * r.end.x + r.end.y * r.end.y + r.end.z * r.end.z;               // ||d||^2 (should be 1 if normalized)
    float b = 2.0f * (oc.x * r.end.x + oc.y * r.end.y + oc.z * r.end.z);               // 2*(o-c)·d
    float c = oc.x * oc.x + oc.y * oc.y + oc.z * oc.z - sphere.radius * sphere.radius; // ||o-c||^2 - r^2

    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0)
        return false; // No intersection

    float sqrt_disc = sqrt(discriminant);
    float t1 = (-b - sqrt_disc) / (2 * a);
    float t2 = (-b + sqrt_disc) / (2 * a);

    // Take closest positive t
    if (t1 > 0)
        t = t1;
    else if (t2 > 0)
        t = t2;
    else
        return false; // Both behind camera

    return true;
}
// Plane
bool intersect_plane(const ray &r, const parser::Plane &plane, const parser::Scene &scene, float &t)
{
    parser::Vec3f n = plane.normal;
    parser::Vec3f P0 = scene.vertex_data[plane.center_vertex_id - 1];
    float denom = dot(n, r.end);
    if (fabs(denom) < 1e-6)
        return false;
    t = dot(n, P0 - r.start) / denom;
    return t > 0;
}

// Triangle
bool intersect_triangle(const ray &r, const parser::Triangle &tri, const parser::Scene &scene, float &t, parser::Vec3f &normal)
{
    // Get triangle vertices
    parser::Vec3f v0 = scene.vertex_data[tri.indices.v0_id - 1];
    parser::Vec3f v1 = scene.vertex_data[tri.indices.v1_id - 1];
    parser::Vec3f v2 = scene.vertex_data[tri.indices.v2_id - 1];

    parser::Vec3f edge1 = v1 - v0;
    parser::Vec3f edge2 = v2 - v0;
    parser::Vec3f h = cross(r.end, edge2);
    float a = dot(edge1, h);
    if (fabs(a) < 1e-6)
        return false; // parallel

    float f = 1.0f / a;
    parser::Vec3f s = r.start - v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    parser::Vec3f q = cross(s, edge1);
    float v = f * dot(r.end, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    t = f * dot(edge2, q);
    if (t <= 0)
        return false;

    normal = normalize(cross(edge1, edge2)); // face normal
    return true;
}

// Mesh
bool intersect_mesh(const ray &r, const parser::Mesh &mesh, const parser::Scene &scene, float &t, parser::Vec3f &normal)
{
    bool hit = false;
    float closest_t = INFINITY;
    parser::Vec3f temp_normal;

    for (auto &face : mesh.faces)
    {
        parser::Triangle tri;
        tri.indices = face;
        tri.material_id = mesh.material_id;

        float t_tri;
        if (intersect_triangle(r, tri, scene, t_tri, temp_normal))
        {
            if (t_tri < closest_t)
            {
                closest_t = t_tri;
                normal = temp_normal;
                hit = true;
            }
        }
    }

    if (hit)
    {
        t = closest_t;
    }
    return hit;
}

// Cylinder
bool intersect_cylinder(const ray &r, const parser::Cylinder &cyl, const parser::Scene &scene, float &t, parser::Vec3f &normal)
{
    const float EPS = 1e-6f;

    // Midpoint from scene -> treat as cylinder midpoint (common parser convention)
    parser::Vec3f C_mid = scene.vertex_data[cyl.center_vertex_id - 1];
    parser::Vec3f V = normalize(cyl.axis); // axis direction (unit)
    float halfH = cyl.height * 0.5f;

    // Compute actual base and top centers
    parser::Vec3f C_base = C_mid - V * halfH;
    parser::Vec3f C_top = C_mid + V * halfH;

    // Ray origin and (normalized) direction
    parser::Vec3f O = r.start;
    parser::Vec3f D = normalize(r.end);

    // Project ray / origin into plane perpendicular to axis for side intersection
    parser::Vec3f deltaP = O - C_base;
    parser::Vec3f D_proj = D - V * dot(D, V);
    parser::Vec3f deltaP_proj = deltaP - V * dot(deltaP, V);

    float a = dot(D_proj, D_proj);

    std::vector<std::pair<float, parser::Vec3f>> hits;

    // Side (cylindrical surface) intersections
    if (a >= EPS)
    {
        float b = 2.0f * dot(D_proj, deltaP_proj);
        float c = dot(deltaP_proj, deltaP_proj) - cyl.radius * cyl.radius;

        float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f)
        {
            float sq = sqrtf(disc);
            float t0 = (-b - sq) / (2.0f * a);
            float t1 = (-b + sq) / (2.0f * a);

            auto check_side = [&](float tc)
            {
                if (tc <= EPS)
                    return;
                parser::Vec3f P = O + D * tc;
                float h = dot(P - C_base, V); // distance from base along axis
                if (h >= -EPS && h <= cyl.height + EPS)
                {
                    parser::Vec3f P_axis = C_base + V * h;
                    parser::Vec3f n = normalize(P - P_axis);
                    if (dot(n, D) > 0)
                        n = n * -1.0f;
                    hits.emplace_back(tc, n);
                }
            };

            check_side(t0);
            check_side(t1);
        }
    }

    // Cap intersections: planes at C_base and C_top
    float denom = dot(D, V);
    if (fabs(denom) > EPS)
    {
        // Base cap
        float t_base = dot(C_base - O, V) / denom;
        if (t_base > EPS)
        {
            parser::Vec3f P = O + D * t_base;
            if (dot(P - C_base, P - C_base) <= cyl.radius * cyl.radius + EPS)
            {
                parser::Vec3f n = -1 * V;
                if (dot(n, D) > 0)
                    n = n * -1.0f;
                hits.emplace_back(t_base, n);
            }
        }

        // Top cap
        float t_top = dot(C_top - O, V) / denom;
        if (t_top > EPS)
        {
            parser::Vec3f P = O + D * t_top;
            if (dot(P - C_top, P - C_top) <= cyl.radius * cyl.radius + EPS)
            {
                parser::Vec3f n = V;
                if (dot(n, D) > 0)
                    n = n * -1.0f;
                hits.emplace_back(t_top, n);
            }
        }
    }

    if (hits.empty())
        return false;

    std::sort(hits.begin(), hits.end(), [](const auto &a, const auto &b)
              { return a.first < b.first; });
    t = hits.front().first;
    normal = hits.front().second;
    return true;
}
