#pragma once  // prevents multiple inclusion

#include "parser.h"

using namespace parser;

float det2x2(float a, float b, float c, float d); // declaration only
Vec3f cross(const Vec3f& a, const Vec3f& b);      // declaration only
Vec3f normalize(const parser::Vec3f &v);
float dot(const parser::Vec3f& a, const parser::Vec3f& b);


