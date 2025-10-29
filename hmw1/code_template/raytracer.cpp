#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <vector>
#include "math_utils.h"
#include "math.h"

typedef unsigned char RGB[3]; // RGB x ;  = unsigned char x[3]; rgb is a type for an array of unsigned char[3]
struct ray;
Vec3f ray_tracer(ray r, parser::Scene &scene);
void generate_image(parser::Camera &camera, parser::Scene &scene);
bool intersect_sphere(const ray &r, const parser::Sphere &sphere, const parser::Scene &scene, float &t);
int main(int argc, char *argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);                          // load the file first
    std::vector<parser::Camera> cameras = scene.cameras; //

    for (int i = 0; i < size(cameras); i++)
    {
        generate_image(cameras[i], scene);
    }

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    const RGB BAR_COLOR[8] =
        {
            {255, 255, 255}, // 100% White
            {255, 255, 0},   // Yellow
            {0, 255, 255},   // Cyan
            {0, 255, 0},     // Green
            {255, 0, 255},   // Magenta
            {255, 0, 0},     // Red
            {0, 0, 255},     // Blue
            {0, 0, 0},       // Black
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

    write_ppm("test.ppm", image, width, height); // save images in the ppm format.

    /*
P3 //or p6 p6 stores rgb values is in truly binary and not hexa but we use hexa to read easily.  p6 takes less space 255 -> 3 bytes. FF -> 1 1byte
# feep.ppm
4 4  //for widht=height
15
 0  0  0    0  0  0    0  0  0   15  0 15
 0  0  0    0 15  7    0  0  0    0  0  0  //
 0  0  0    0  0  0    0 15  7    0  0  0
15  0 15    0  0  0    0  0  0    0  0  0*/
}

struct ray
{
    Vec3f start, end;
};

void generate_image(parser::Camera &camera, parser::Scene &scene)
{
    int image_width = camera.image_width;
    int image_height = camera.image_height;

    float left = camera.near_plane.w;
    float right = camera.near_plane.x;
    float bottom = camera.near_plane.y;
    float top = camera.near_plane.z;

    // FIX: Add parentheses
    float pixel_width = (right - left) / image_width;
    float pixel_height = (top - bottom) / image_height;

    // Construct orthonormal basis - normalize w and u
    parser::Vec3f w = normalize(camera.gaze * -1);
    parser::Vec3f u = normalize(cross(w, camera.up));
    parser::Vec3f v = cross(w, u); // Already normalized since u,w perpendicular

    // Center of image plane
    parser::Vec3f m = camera.position + camera.gaze * camera.near_distance;

    // Top-left corner
    parser::Vec3f q = m + u * left + v * top;

    // Allocate image buffer
    unsigned char *image = new unsigned char[image_width * image_height * 3];
    int idx = 0;

    for (int j = 0; j < image_height; j++)
    {
        for (int i = 0; i < image_width; i++)
        {
            // Calculate pixel center offsets
            float su = (i + 0.5f) * pixel_width;
            float sv = (j + 0.5f) * pixel_height;

            // Pixel position on image plane
            parser::Vec3f pixel_point = q + u * su - v * sv;

            // Create ray
            ray ray_equation;
            ray_equation.start = camera.position;
            ray_equation.end = normalize(pixel_point - camera.position); // This is direction, not end

            parser::Vec3f color = ray_tracer(ray_equation, scene);

            // Store in image buffer
            image[idx++] = (unsigned char)(color.x * 255);
            image[idx++] = (unsigned char)(color.y * 255);
            image[idx++] = (unsigned char)(color.z * 255);
        }
    }

    write_ppm(camera.image_name.c_str(), image, image_width, image_height);
    delete[] image;
}

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

parser::Vec3f ray_tracer(ray r, parser::Scene &scene)
{
    float closest_t = INFINITY;
    bool hit = false;
    int hit_sphere_idx = -1;

    // Find closest intersection
    for (int i = 0; i < scene.spheres.size(); i++)
    {
        float t;
        if (intersect_sphere(r, scene.spheres[i], scene, t))
        {
            if (t < closest_t)
            {
                closest_t = t;
                hit = true;
                hit_sphere_idx = i;
            }
        }
    }

    if (!hit)
    {
        // Return background color (normalize from 0-255 -> 0-1)
        return parser::Vec3f{scene.background_color.x / 255.0f,
                             scene.background_color.y / 255.0f,
                             scene.background_color.z / 255.0f};
    }

    // Hit a sphere - compute shading
    parser::Sphere sphere = scene.spheres[hit_sphere_idx];
    parser::Material material = scene.materials[sphere.material_id - 1]; // 1-indexed

    // Normalize material colors from 0-255 -> 0-1
    parser::Vec3f mat_ambient = material.ambient;
    parser::Vec3f mat_diffuse = material.diffuse;
    parser::Vec3f mat_specular = material.specular;

    // Normalize scene ambient (in case it's 0-255)
    parser::Vec3f ambient = scene.ambient_light * (1.0f / 255.0f);

    // Intersection point
    parser::Vec3f hit_point = r.start + r.end * closest_t;

    // Normal at intersection
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];
    parser::Vec3f normal = normalize(hit_point - center);

    // Start with ambient
    parser::Vec3f color = mat_ambient * ambient;

    // Add contribution from each light
    // Add contribution from each light using irradiance
    for (int i = 0; i < scene.point_lights.size(); i++)
    {
        parser::PointLight light = scene.point_lights[i];

        // Light vector and distance
        parser::Vec3f light_vec = light.position - hit_point;
        float distance2 = dot(light_vec, light_vec); // squared distance
        parser::Vec3f light_dir = normalize(light_vec);

        // Irradiance using inverse-square law
        parser::Vec3f irradiance = light.intensity * (1.0f / 255.0f) / distance2;

        // Diffuse
        float ndotl = std::max(0.0f, dot(normal, light_dir));
        color = color + mat_diffuse * irradiance * ndotl;

        // Specular (Phong)
        parser::Vec3f view_dir = normalize(r.start - hit_point); // toward camera
        parser::Vec3f reflect_dir = normal * (2.0f * dot(normal, light_dir)) - light_dir;
        float rdotv = std::max(0.0f, dot(reflect_dir, view_dir));
        float spec = pow(rdotv, material.phong_exponent);
        color = color + mat_specular * irradiance * spec;
    }

    // Clamp to [0,1] to avoid overflow when converting to 0-255
    color.x = std::min(1.0f, std::max(0.0f, color.x));
    color.y = std::min(1.0f, std::max(0.0f, color.y));
    color.z = std::min(1.0f, std::max(0.0f, color.z));

    return color;
}