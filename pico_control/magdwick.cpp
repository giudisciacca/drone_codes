# include <stdio.h>
# include <math.h>
# include <pico_control/quaternions.cpp>

using namespace std;

quaternion compute_magdick_potential(){
    // Placeholder for the actual implementation of the Magdwick potential computation
    // This function should return a quaternion representing the computed potential
    // For now, we return a default quaternion
    return quaternion(1.0, 0.0, 0.0, 0.0);
}

quaternion from_acceleration_get_quaternion(float ax, float ay, float az) {
    // Normalize the acceleration vector
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) {
        return quaternion(1.0, 0.0, 0.0, 0.0); // Return identity quaternion if norm is zero
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Compute the quaternion from the normalized acceleration vector
    return quaternion(1.0, ax, ay, az);
}


quaternion compute_quaternion_derivative(const quaternion* q, float gx, float gy, float gz, float dt) {
    // Compute the quaternion derivative based on gyroscope data
    float half_dt = dt * 0.5f;
    return quaternion(
        -q->x * gx - q->y * gy - q->z * gz,
        q->w * gx + q->y * gz - q->z * gy,
        q->w * gy - q->x * gz + q->z * gx,
        q->w * gz + q->x * gy - q->y * gx
    ); //* half_dt;
}


quaternion get_magdwick_gradient(){
    // Placeholder for the actual implementation of the Magdwick gradient computation
    // This function should return a quaternion representing the computed gradient
    // For now, we return a default quaternion
    return quaternion(0.0, 1.0, 0.0, 0.0);
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











