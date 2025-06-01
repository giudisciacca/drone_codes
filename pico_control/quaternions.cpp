#include <stdio.h>
#include <math.h>

using namespace std;

class quaternion{
    public:
        float w, x, y, z;
        quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
        quaternion() : w(1), x(0), y(0), z(0) {}
        quaternion(const quaternion& q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
        ~quaternion() {}
};


quaternion quaternion_product(const quaternion* q1, const quaternion* q2) {
    return quaternion(
        q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,
        q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,
        q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,
        q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w
    );
}

int main() {
    quaternion q1(1.0, 2.0, 3.0, 4.0);
    quaternion q2(5.0, 6.0, 7.0, 8.0);
    
    // Example usage of quaternion_product
    quaternion q3 = quaternion_product(&q1, &q2);
    printf("Quaternion q3: (%f, %f, %f, %f)\n", q3.w, q3.x, q3.y, q3.z);
    
    printf("Quaternion q1: (%f, %f, %f, %f)\n", q1.w, q1.x, q1.y, q1.z);
    printf("Quaternion q2: (%f, %f, %f, %f)\n", q2.w, q2.x, q2.y, q2.z);
    
    return 0;
}