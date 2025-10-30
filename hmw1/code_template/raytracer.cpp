#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <vector>
#include "math_utils.h"
#include "math.h"
#include "intersects.h"
typedef unsigned char RGB[3];
struct ray;
parser::Vec3f ray_tracer(ray r, const parser::Scene &scene, int depth);
void generate_image(parser::Camera &camera, parser::Scene &scene);
bool intersect_sphere(const ray &r, const parser::Sphere &sphere, const parser::Scene &scene, float &t);
bool is_in_shadow(const parser::Vec3f &hit_point, const parser::Vec3f &light_pos, const parser::Scene &scene);

parser::Vec3f compute_lighting(const parser::Vec3f &hit_point,
                               const parser::Vec3f &normal,
                               const parser::Material &material,
                               const parser::Scene &scene,
                               const parser::Vec3f &view_pos,
                               const parser::Vec3f &ray_dir,
                               int depth);

int main(int argc, char *argv[])
{
    parser::Scene scene;
    scene.loadFromXml(argv[1]);
    std::vector<parser::Camera> cameras = scene.cameras;

    for (int i = 0; i < size(cameras); i++)
    {
        generate_image(cameras[i], scene);
    }

    const RGB BAR_COLOR[8] =
        {
            {255, 255, 255},
            {255, 255, 0},
            {0, 255, 255},
            {0, 255, 0},
            {255, 0, 255},
            {255, 0, 0},
            {0, 0, 255},
            {0, 0, 0},
        };

    int width = 5, height = 5;
    int columnWidth = width / 8;

    unsigned char *image = new unsigned char[width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    write_ppm("test.ppm", image, width, height);
}

void generate_image(parser::Camera &camera, parser::Scene &scene)
{
    int image_width = camera.image_width;
    int image_height = camera.image_height;

    float left = camera.near_plane.w;
    float right = camera.near_plane.x;
    float bottom = camera.near_plane.y;
    float top = camera.near_plane.z;

    float pixel_width = (right - left) / image_width;
    float pixel_height = (top - bottom) / image_height;

    parser::Vec3f w = normalize(camera.gaze * -1);
    parser::Vec3f u = normalize(cross(w, camera.up));
    parser::Vec3f v = cross(w, u);

    parser::Vec3f m = camera.position + camera.gaze * camera.near_distance;
    parser::Vec3f q = m + u * left + v * top;

    unsigned char *image = new unsigned char[image_width * image_height * 3];
    int idx = 0;

    for (int j = 0; j < image_height; j++)
    {
        for (int i = 0; i < image_width; i++)
        {
            float su = (i + 0.5f) * pixel_width;
            float sv = (j + 0.5f) * pixel_height;

            parser::Vec3f pixel_point = q + u * su - v * sv;

            ray ray_equation;
            ray_equation.start = camera.position;
            ray_equation.end = normalize(pixel_point - camera.position);

            parser::Vec3f color = ray_tracer(ray_equation, scene, 0);

            image[idx++] = (unsigned char)(color.x * 255);
            image[idx++] = (unsigned char)(color.y * 255);
            image[idx++] = (unsigned char)(color.z * 255);
        }
    }

    write_ppm(camera.image_name.c_str(), image, image_width, image_height);
    delete[] image;
}

parser::Vec3f ray_tracer(ray r, const parser::Scene &scene, int depth)
{
    float closest_t = INFINITY;
    bool hit = false;

    parser::Vec3f hit_point;
    parser::Vec3f hit_normal;
    parser::Material hit_material;

    // --- Spheres ---
    for (auto &sphere : scene.spheres)
    {
        float t;
        if (intersect_sphere(r, sphere, scene, t) && t < closest_t)
        {
            closest_t = t;
            hit = true;
            hit_point = r.start + r.end * t;
            hit_normal = normalize(hit_point - scene.vertex_data[sphere.center_vertex_id - 1]);
            hit_material = scene.materials[sphere.material_id - 1];
        }
    }

    // --- Planes ---
    for (auto &plane : scene.planes)
    {
        float t;
        if (intersect_plane(r, plane, scene, t) && t < closest_t)
        {
            closest_t = t;
            hit = true;
            hit_point = r.start + r.end * t;
            hit_normal = plane.normal;
            hit_material = scene.materials[plane.material_id - 1];
        }
    }

    // --- Triangles ---
    for (auto &tri : scene.triangles)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_triangle(r, tri, scene, t, normal) && t < closest_t)
        {
            closest_t = t;
            hit = true;
            hit_point = r.start + r.end * t;
            hit_normal = normal;
            hit_material = scene.materials[tri.material_id - 1];
        }
    }

    // --- Meshes ---
    for (auto &mesh : scene.meshes)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_mesh(r, mesh, scene, t, normal) && t < closest_t)
        {
            closest_t = t;
            hit = true;
            hit_point = r.start + r.end * t;
            hit_normal = normal;
            hit_material = scene.materials[mesh.material_id - 1];
        }
    }

    // --- Cylinders ---
    for (auto &cyl : scene.cylinders)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_cylinder(r, cyl, scene, t, normal) && t < closest_t)
        {
            closest_t = t;
            hit = true;
            hit_point = r.start + r.end * t;
            hit_normal = normal;
            hit_material = scene.materials[cyl.material_id - 1];
        }
    }

    // --- No hit → return background ---
    if (!hit)
    {
        return parser::Vec3f{
            scene.background_color.x / 255.0f,
            scene.background_color.y / 255.0f,
            scene.background_color.z / 255.0f};
    }

    // --- Hit → compute lighting with reflections ---
    return compute_lighting(hit_point, hit_normal, hit_material, scene, r.start, r.end, depth);
}

// Check if point is in shadow for a given light
bool is_in_shadow(const parser::Vec3f &hit_point, const parser::Vec3f &light_pos, const parser::Scene &scene)
{
    const float EPSILON = 0.001f;

    parser::Vec3f light_vec = light_pos - hit_point;
    float light_distance = sqrt(dot(light_vec, light_vec));
    parser::Vec3f light_dir = light_vec * (1.0f / light_distance);

    ray shadow_ray;
    shadow_ray.start = hit_point + light_dir * EPSILON;
    shadow_ray.end = light_dir;

    // --- Spheres ---
    for (const auto &sphere : scene.spheres)
    {
        float t;
        if (intersect_sphere(shadow_ray, sphere, scene, t) && t > 0 && t < light_distance)
        {
            return true;
        }
    }

    // --- Planes ---
    for (const auto &plane : scene.planes)
    {
        float t;
        if (intersect_plane(shadow_ray, plane, scene, t) && t > 0 && t < light_distance)
        {
            return true;
        }
    }

    // --- Triangles ---
    for (const auto &tri : scene.triangles)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_triangle(shadow_ray, tri, scene, t, normal) && t > 0 && t < light_distance)
        {
            return true;
        }
    }

    // --- Meshes ---
    for (const auto &mesh : scene.meshes)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_mesh(shadow_ray, mesh, scene, t, normal) && t > 0 && t < light_distance)
        {
            return true;
        }
    }

    // --- Cylinders ---
    for (const auto &cyl : scene.cylinders)
    {
        float t;
        parser::Vec3f normal;
        if (intersect_cylinder(shadow_ray, cyl, scene, t, normal) && t > 0 && t < light_distance)
        {
            return true;
        }
    }

    return false;
}

parser::Vec3f compute_lighting(const parser::Vec3f &hit_point,
                               const parser::Vec3f &normal,
                               const parser::Material &material,
                               const parser::Scene &scene,
                               const parser::Vec3f &view_pos,
                               const parser::Vec3f &ray_dir,
                               int depth)
{
    // Normalize material colors
    parser::Vec3f mat_ambient = material.ambient;
    parser::Vec3f mat_diffuse = material.diffuse;
    parser::Vec3f mat_specular = material.specular;
    parser::Vec3f mat_mirror = material.mirror;

    // Normalize scene ambient
    parser::Vec3f ambient = scene.ambient_light * (1.0f / 255.0f);

    // Start with ambient
    parser::Vec3f color = mat_ambient * ambient;

    // Iterate over all point lights
    for (const auto &light : scene.point_lights)
    {
        // Check if point is in shadow for this light
        if (is_in_shadow(hit_point, light.position, scene))
        {
            continue;
        }

        parser::Vec3f light_vec = light.position - hit_point;
        float distance2 = dot(light_vec, light_vec);
        parser::Vec3f light_dir = normalize(light_vec);

        // Irradiance using inverse-square law
        parser::Vec3f irradiance = light.intensity * (1.0f / 255.0f) / distance2;

        // Diffuse
        float ndotl = std::max(0.0f, dot(normal, light_dir));
        color = color + mat_diffuse * irradiance * ndotl;

        // Specular (Phong)
        parser::Vec3f view_dir = normalize(view_pos - hit_point);
        parser::Vec3f reflect_dir = normal * (2.0f * dot(normal, light_dir)) - light_dir;
        float rdotv = std::max(0.0f, dot(reflect_dir, view_dir));
        float spec = pow(rdotv, material.phong_exponent);
        color = color + mat_specular * irradiance * spec;
    }

    // Recursive reflections
    if (depth < scene.max_recursion_depth)
    {
        // Check if material has reflective properties
        float mirror_strength = mat_mirror.x + mat_mirror.y + mat_mirror.z;

        if (mirror_strength > 0.0f)
        {
            const float EPSILON = scene.shadow_ray_epsilon; // Use scene epsilon instead of hardcoded

            // Calculate reflection direction: R = D - 2(D·N)N
            parser::Vec3f incoming_dir = normalize(ray_dir); // Normalize incoming direction
            parser::Vec3f reflect_dir = incoming_dir - normal * (2.0f * dot(incoming_dir, normal));
            reflect_dir = normalize(reflect_dir);

            // Create reflection ray
            ray reflection_ray;
            reflection_ray.start = hit_point + reflect_dir * EPSILON;
            reflection_ray.end = reflect_dir;

            // Recursively trace reflection ray
            parser::Vec3f reflected_color = ray_tracer(reflection_ray, scene, depth + 1);

            // Add reflection contribution weighted by mirror reflectance
            color = color + mat_mirror * reflected_color;
        }
    }

    // Clamp color to [0,1]
    color.x = std::min(1.0f, std::max(0.0f, color.x));
    color.y = std::min(1.0f, std::max(0.0f, color.y));
    color.z = std::min(1.0f, std::max(0.0f, color.z));

    return color;
}