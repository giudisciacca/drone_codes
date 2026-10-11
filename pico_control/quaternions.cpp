#include <stdio.h>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using matrix4x4 = Eigen::Matrix<float, 4, 4>;
using vector3d = Eigen::Matrix<float, 3, 1>;


class quaternion{
    public:
        float w, x, y, z;
        quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
        quaternion() : w(1), x(0), y(0), z(0) {}
        quaternion(const quaternion& q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
        ~quaternion() {}

    quaternion operator*(const quaternion& q) const {
        return quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    quaternion operator*(const float scalar) const {
        return quaternion(
            w * scalar,
            x * scalar,
            y * scalar,
            z * scalar
        );
    }

    quaternion operator+(const quaternion& q) const {
        return quaternion(
            w + q.w,
            x + q.x,
            y + q.y,
            z + q.z
        );
    }

    quaternion normalize() const {
        float norm = sqrt(w * w + x * x + y * y + z * z);
        return quaternion(
            w / norm,
            x / norm,
            y / norm,
            z / norm
        );
    }

    quaternion conjugate() const {
        return quaternion(
            w,
            -x,
            -y,
            -z
        );
    }

    matrix4x4 to_left_matrix() const {
        return matrix4x4(
            w, -x, -y, -z,
            x,  w, -z,  y,
            y,  z,  w, -x,
            z, -y,  x,  w
        );
    }                                                                           

    matrix4x4 to_right_matrix() const {
        return matrix4x4(
            w, -x, -y, -z,
            x,  w,  z, -y,
            y, -z,  w,  x,
            z,  y, -x,  w
        );
    }

    Eigen::Matrix<float, 3, 4> dqdtheta() const {
        Eigen::Matrix<float, 3, 4> J;
        J << -x,  w, -z,  y,
             -y,  z,  w, -x,
             -z, -y,  x,  w;
        return J;
    } 
};


quaternion vector_to_quaternion(vecotr3d v) {
    return quaternion(0, v[0], v[1], v[2]);
}

quaternion vector_to_quaternion_exponential_map(vector3d v) {
    float theta = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (theta < 1e-6) {
        return quaternion(1, 0, 0, 0);
    } else {
        float half_theta = theta / 2.0f;
        float sin_half_theta = sin(half_theta);
        return quaternion(cos(half_theta), sin_half_theta * v[0] / theta, sin_half_theta * v[1] / theta, sin_half_theta * v[2] / theta);
    }
}



quaternion quaternion_product(const quaternion* q1, const quaternion* q2) {
    return quaternion(
        q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,
        q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,
        q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,
        q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w
    );
}

quaternion quaternion_product(const quaternion* q1, const float* scalar) {
    return quaternion(
        q1->w * (*scalar),
        q1->x * (*scalar),
        q1->y * (*scalar),
        q1->z * (*scalar)
    );
}


int _main_quaternions() {
    quaternion q1(1.0, 2.0, 3.0, 4.0);
    quaternion q2(5.0, 6.0, 7.0, 8.0);
    
    // Example usage of quaternion_product
    quaternion q3 = quaternion_product(&q1, &q2);
    printf("Quaternion q3: (%f, %f, %f, %f)\n", q3.w, q3.x, q3.y, q3.z);
    
    printf("Quaternion q1: (%f, %f, %f, %f)\n", q1.w, q1.x, q1.y, q1.z);
    printf("Quaternion q2: (%f, %f, %f, %f)\n", q2.w, q2.x, q2.y, q2.z);
    
    return 0;
}

