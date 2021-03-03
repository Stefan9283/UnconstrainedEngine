//
// Created by Stefan on 03-Mar-21.
//

#ifndef TRIANGLE_VEC3_H
#define TRIANGLE_VEC3_H

namespace pe {
    template<typename T>
    class vec3 {
    public:
        T x, y, z;

        vec3(T x, T y, T z) {
            this->x = x;
            this->y = y;
            this->z = z;
        }
        static T dot(vec3 e1, vec3 e2) {
            return e1.x * e2.x + e1.y * e2.y + e1.z * e2.z;
        }
        T length() {
            return sqrt(dot(*this, *this));
        }
        static vec3 cross(vec3 a, vec3 b) {
            return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        }
        std::string toString() {
            return std::string("vec3(") + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + std::string(")");
        }
    };
}

#endif //TRIANGLE_VEC3_H
