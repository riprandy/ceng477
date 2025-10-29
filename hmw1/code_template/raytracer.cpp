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
    std::vector<parser::Camera> cameras = scene.cameras; //  t

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
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1]; // IDs are 1-indexed
    parser::Vec3f oc = r.start - center;                                   // origin - center //this works if i do the - operator const

    // Ray: o + t*d
    // Sphere: ||p - c||^2 = r^2
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

    for (int i = 0; i < scene.spheres.size(); i++)
    {
        float t;
        if (intersect_sphere(r, scene.spheres[i], scene, t))
        {
            if (t < closest_t)
            {
                closest_t = t;
                hit = true;
            }
        }
    }

    if (hit)
        return parser::Vec3f{1.0f, 0.0f, 0.0f}; // Red
    else
        return parser::Vec3f{scene.background_color.x / 255.0f,
                             scene.background_color.y / 255.0f,
                             scene.background_color.z / 255.0f};
}