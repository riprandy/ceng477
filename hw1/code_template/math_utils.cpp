#include "math_utils.h"
#include "math.h"
float det2x2(float a, float b, float c, float d)
{
    return a * d - b * c;
}

Vec3f cross(const Vec3f &a, const Vec3f &b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3f normalize(const parser::Vec3f &v)
{
    float length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

    // Avoid division by zero
    if (length < 1e-8f)
        return v;

    return {v.x / length, v.y / length, v.z / length};
}

float dot(const parser::Vec3f& a, const parser::Vec3f& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}