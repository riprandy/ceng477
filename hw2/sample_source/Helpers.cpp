#include <iostream>
#include <cmath>
#include "Helpers.h"
#include "Camera.h"
#include <vector>
#include <algorithm>
using namespace std;
/*
 * Calculate cross product of vec3 a, vec3 b and return resulting vec3.
 */
Vec3 crossProductVec3(Vec3 a, Vec3 b)
{
    return Vec3(a.y * b.z - b.y * a.z, b.x * a.z - a.x * b.z, a.x * b.y - b.x * a.y);
}

/*
 * Calculate dot product of vec3 a, vec3 b and return resulting value.
 */
double dotProductVec3(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
 * Find length (|v|) of vec3 v.
 */
double magnitudeOfVec3(Vec3 v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/*
 * Normalize the vec3 to make it unit vec3.
 */
Vec3 normalizeVec3(Vec3 v)
{
    double d = magnitudeOfVec3(v);
    return Vec3(v.x / d, v.y / d, v.z / d);
}

/*
 * Return -v (inverse of vec3 v)
 */
Vec3 inverseVec3(Vec3 v)
{
    return Vec3(-v.x, -v.y, -v.z);
}

/*
 * Add vec3 a to vec3 b and return resulting vec3 (a+b).
 */
Vec3 addVec3(Vec3 a, Vec3 b)
{
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/*
 * Subtract vec3 b from vec3 a and return resulting vec3 (a-b).
 */
Vec3 subtractVec3(Vec3 a, Vec3 b)
{
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/*
 * Multiply each element of vec3 with scalar.
 */
Vec3 multiplyVec3WithScalar(Vec3 v, double c)
{
    return Vec3(v.x * c, v.y * c, v.z * c);
}

/*
 * Prints elements in a vec3. Can be used for debugging purposes.
 */
void printVec3(Vec3 v)
{
    std::cout << "(" << v.x << "," << v.y << "," << v.z << ")" << std::endl;
}

/*
 * Check whether vec3 a and vec3 b are equal.
 * In case of equality, returns 1.
 * Otherwise, returns 0.
 */
int areEqualVec3(Vec3 a, Vec3 b)
{

    /* if x difference, y difference and z difference is smaller than threshold, then they are equal */
    if ((ABS((a.x - b.x)) < EPSILON) && (ABS((a.y - b.y)) < EPSILON) && (ABS((a.z - b.z)) < EPSILON))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * Returns an identity matrix (values on the diagonal are 1, others are 0).
 */
Matrix4 getIdentityMatrix()
{
    Matrix4 result;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
            {
                result.values[i][j] = 1.0;
            }
            else
            {
                result.values[i][j] = 0.0;
            }
        }
    }

    return result;
}

/*
 * Multiply matrices m1 (Matrix4) and m2 (Matrix4) and return the result matrix r (Matrix4).
 */
Matrix4 multiplyMatrixWithMatrix(Matrix4 m1, Matrix4 m2)
{
    Matrix4 result;
    double total;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            total = 0;
            for (int k = 0; k < 4; k++)
            {
                total += m1.values[i][k] * m2.values[k][j];
            }

            result.values[i][j] = total;
        }
    }

    return result;
}

/*
 * Multiply matrix m (Matrix4) with vector v (Vec4WithColor) and store the result in vector r (Vec4WithColor).
 */
Vec4WithColor multiplyMatrixWithVec4WithColor(Matrix4 m, Vec4WithColor v)
{
    double values[4];
    double total;

    for (int i = 0; i < 4; i++)
    {
        total = 0;
        for (int j = 0; j < 4; j++)
        {
            total += m.values[i][j] * v.getNthComponent(j);
        }
        values[i] = total;
    }

    return Vec4WithColor(values[0], values[1], values[2], values[3], v.color);
}

// new helper functions
Matrix4 getRotationMatrix(float angle, float ux, float uy, float uz)
{
    // Normalize the axis
    float length = sqrt(ux * ux + uy * uy + uz * uz);
    ux /= length;
    uy /= length;
    uz /= length;

    // Convert angle to radians
    float alpha = angle * M_PI / 180.0;
    float c = cos(alpha);
    float s = sin(alpha);
    float t = 1.0 - c;

    // Build Rodrigues' rotation matrix
    Matrix4 M;

    M.values[0][0] = t * ux * ux + c;
    M.values[0][1] = t * ux * uy - s * uz;
    M.values[0][2] = t * ux * uz + s * uy;
    M.values[0][3] = 0;

    M.values[1][0] = t * ux * uy + s * uz;
    M.values[1][1] = t * uy * uy + c;
    M.values[1][2] = t * uy * uz - s * ux;
    M.values[1][3] = 0;

    M.values[2][0] = t * ux * uz - s * uy;
    M.values[2][1] = t * uy * uz + s * ux;
    M.values[2][2] = t * uz * uz + c;
    M.values[2][3] = 0;

    M.values[3][0] = 0;
    M.values[3][1] = 0;
    M.values[3][2] = 0;
    M.values[3][3] = 1;

    return M;
}

Matrix4 getCameraTransformMatrix(Camera *camera)
{
    // Use camera's already computed u, v, w vectors
    Vec3 u = camera->u;
    Vec3 v = camera->v;
    Vec3 w = camera->w;
    Vec3 position = camera->position;

    // Build camera transformation matrix
    Matrix4 M;

    // Rotation part (inverse of camera orientation)
    M.values[0][0] = u.x;
    M.values[0][1] = u.y;
    M.values[0][2] = u.z;
    M.values[0][3] = -dotProductVec3(u, position);

    M.values[1][0] = v.x;
    M.values[1][1] = v.y;
    M.values[1][2] = v.z;
    M.values[1][3] = -dotProductVec3(v, position);

    M.values[2][0] = w.x;
    M.values[2][1] = w.y;
    M.values[2][2] = w.z;
    M.values[2][3] = -dotProductVec3(w, position);

    M.values[3][0] = 0;
    M.values[3][1] = 0;
    M.values[3][2] = 0;
    M.values[3][3] = 1;

    return M;
}

Matrix4 getProjectionMatrix(Camera *camera)
{
    Matrix4 M;
    double l = camera->left;
    double r = camera->right;
    double b = camera->bottom;
    double t = camera->top;
    double n = camera->near;
    double f = camera->far;

    if (camera->projectionType == 1)
    { // 1 for perspective
        M.values[0][0] = (2 * n) / (r - l);
        M.values[0][1] = 0;
        M.values[0][2] = (r + l) / (r - l);
        M.values[0][3] = 0;

        M.values[1][0] = 0;
        M.values[1][1] = (2 * n) / (t - b);
        M.values[1][2] = (t + b) / (t - b);
        M.values[1][3] = 0;

        M.values[2][0] = 0;
        M.values[2][1] = 0;
        M.values[2][2] = -(f + n) / (f - n);
        M.values[2][3] = -(2 * f * n) / (f - n);

        M.values[3][0] = 0;
        M.values[3][1] = 0;
        M.values[3][2] = -1;
        M.values[3][3] = 0;
    }
    else
    { // ORTHOGRAPHIC //0 for ortographic
        M.values[0][0] = 2 / (r - l);
        M.values[0][1] = 0;
        M.values[0][2] = 0;
        M.values[0][3] = -(r + l) / (r - l);

        M.values[1][0] = 0;
        M.values[1][1] = 2 / (t - b);
        M.values[1][2] = 0;
        M.values[1][3] = -(t + b) / (t - b);

        M.values[2][0] = 0;
        M.values[2][1] = 0;
        M.values[2][2] = -2 / (f - n);
        M.values[2][3] = -(f + n) / (f - n);

        M.values[3][0] = 0;
        M.values[3][1] = 0;
        M.values[3][2] = 0;
        M.values[3][3] = 1;
    }

    return M;
}

Matrix4 getViewportMatrix(Camera *camera)
{
    Matrix4 M;
    double width = camera->horRes;
    double height = camera->verRes;

    M.values[0][0] = width / 2.0;
    M.values[0][1] = 0;
    M.values[0][2] = 0;
    M.values[0][3] = (width - 1) / 2.0;

    M.values[1][0] = 0;
    M.values[1][1] = height / 2.0;
    M.values[1][2] = 0;
    M.values[1][3] = (height - 1) / 2.0;

    M.values[2][0] = 0;
    M.values[2][1] = 0;
    M.values[2][2] = 0.5;
    M.values[2][3] = 0.5;

    M.values[3][0] = 0;
    M.values[3][1] = 0;
    M.values[3][2] = 0;
    M.values[3][3] = 1;

    return M;
}

void drawLine(Vec4WithColor v0, Vec4WithColor v1,
              std::vector<std::vector<Color>> &image,
              std::vector<std::vector<double>> &depthBuffer)
{
    int x0 = round(v0.x), y0 = round(v0.y);
    int x1 = round(v1.x), y1 = round(v1.y);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0, y = y0;
    double t = 0.0;
    double totalDist = sqrt(dx * dx + dy * dy);

    while (true)
    {
        // Bounds check
        if (x >= 0 && x < image[0].size() && y >= 0 && y < image.size())
        {
            // Interpolate depth and color
            double dist = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
            t = (totalDist > 0) ? dist / totalDist : 0;

            double z = v0.z * (1 - t) + v1.z * t;

            if (z < depthBuffer[y][x])
            {
                depthBuffer[y][x] = z;

                // Interpolate color (use colors carried in the Vec4WithColor)
                Color c0 = v0.color;
                Color c1 = v1.color;

                image[y][x].r = round(c0.r * (1 - t) + c1.r * t);
                image[y][x].g = round(c0.g * (1 - t) + c1.g * t);
                image[y][x].b = round(c0.b * (1 - t) + c1.b * t);
            }
        }

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

void rasterizeTriangle(Vec4WithColor v0, Vec4WithColor v1, Vec4WithColor v2,
                       std::vector<std::vector<Color>> &image,
                       std::vector<std::vector<double>> &depthBuffer)
{
    // Find bounding box
    int minX = max(0, (int)floor(min({v0.x, v1.x, v2.x})));
    int maxX = min((int)image[0].size() - 1, (int)ceil(max({v0.x, v1.x, v2.x})));
    int minY = max(0, (int)floor(min({v0.y, v1.y, v2.y})));
    int maxY = min((int)image.size() - 1, (int)ceil(max({v0.y, v1.y, v2.y})));

    // Triangle area (for barycentric coords)
    double area = (v1.x - v0.x) * (v2.y - v0.y) - (v2.x - v0.x) * (v1.y - v0.y);
    if (abs(area) < 0.0001)
        return; // Degenerate triangle

    for (int y = minY; y <= maxY; y++)
    {
        for (int x = minX; x <= maxX; x++)
        {
            double px = x + 0.5;
            double py = y + 0.5;

            // Barycentric coordinates
            double alpha = ((v1.x - px) * (v2.y - py) - (v2.x - px) * (v1.y - py)) / area;
            double beta = ((v2.x - px) * (v0.y - py) - (v0.x - px) * (v2.y - py)) / area;
            double gamma = 1.0 - alpha - beta;

            // Check if point is inside triangle (with small epsilon for edge cases)
            if (alpha >= -1e-9 && beta >= -1e-9 && gamma >= -1e-9)
            {
                // Interpolate depth
                double z = alpha * v0.z + beta * v1.z + gamma * v2.z;

                if (z < depthBuffer[y][x])
                {
                    depthBuffer[y][x] = z;

                    // Interpolate color (use colors carried in the Vec4WithColor)
                    Color c0 = v0.color;
                    Color c1 = v1.color;
                    Color c2 = v2.color;

                    image[y][x].r = round(alpha * c0.r + beta * c1.r + gamma * c2.r);
                    image[y][x].g = round(alpha * c0.g + beta * c1.g + gamma * c2.g);
                    image[y][x].b = round(alpha * c0.b + beta * c1.b + gamma * c2.b);
                }
            }
        }
    }
}