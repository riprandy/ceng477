#ifndef __HW1__PARSER__
#define __HW1__PARSER__
#include <string>
#include <vector>
namespace parser
{
    // Forward declarations
    struct Vec3f;
    struct Vec3i;

    struct Vec2i
    {
        int x, y;
        Vec2i operator+(Vec2i &s) const
        {
            return {x + s.x, y + s.y};
        }
    };

    struct Vec3f
    {
        float x, y, z;
        Vec3f operator*(float s) const
        {
            return {x * s, y * s, z * s};
        }
        Vec3f operator+(parser::Vec3f const &s) const
        {
            return {s.x + x, s.y + y, s.z + z};
        }
        Vec3f operator-(parser::Vec3f const &s) const
        {
            return {x - s.x, y - s.y, z - s.z};
        }
        };

    inline Vec3f operator*(float s, const Vec3f &v)
    {
        return {v.x * s, v.y * s, v.z * s};
    }

    struct Vec3i
    {
        int x, y, z;
        Vec3i operator*(float s) const
        {
            return {x * s, y * s, z * s};
        }
        Vec3i operator+(Vec3i &s) const
        {
            return {s.x + x, s.y + y, s.z + z};
        }
        Vec3f operator+(const Vec3i &b) const
        {
            return {x + b.x, y + b.y, z + b.z};
        }
    };

    struct Vec4f
    {
        Vec4f operator*(double s) const
        {
            return {x * s, y * s, z * s, w * s};
        }
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Cylinder
    {
        int material_id;
        int center_vertex_id;
        Vec3f axis;
        float radius;
        float height;
    };

    struct Plane
    {
        int material_id;
        int center_vertex_id;
        Vec3f normal;
    };

    struct Scene
    {
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;
        std::vector<Cylinder> cylinders;
        std::vector<Plane> planes;

        void loadFromXml(const std::string &filepath);
    };
}
#endif