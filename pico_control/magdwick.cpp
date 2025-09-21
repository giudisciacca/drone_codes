# include <stdio.h>
# include <math.h>
# include <pico_control/quaternions.cpp>
# include <pico_control/imc29048_io.cpp>
using namespace std;

quaternion compute_magdick_potential(){
    // Placeholder for the actual implementation of the Magdwick potential computation
    // This function should return a quaternion representing the computed potential
    // For now, we return a default quaternion
    return quaternion(1.0, 0.0, 0.0, 0.0);
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

quaternion from_euler_get_quaternion(double roll, double pitch, double yaw) {
    // Convert Euler angles (roll, pitch, yaw) to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    return quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}



quaternion a2quaternion(double ax, double ay, double az) {
    // Convert acceleration vector to quaternion
    printf("Accel: ax=%f ay=%f az=%f\n", ax, ay, az);
    double norm; norm = sqrt(ax * ax + ay * ay + az * az);
    printf("Accel norm: %f\n", norm);
    if (norm == 0) {
        return quaternion(1.0, 0.0, 0.0, 0.0); // Return identity quaternion if norm is zero
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    double roll; 
    double pitch; 
    pitch = atan2(ay, ax);
    roll = atan2(-az, sqrt(ax * ax + ay * ay));
    double yaw; yaw = 0.0; // Assuming no yaw information from accelerometer
    printf("Roll: %f Pitch: %f Yaw: %f\n", roll, pitch, yaw);
    quaternion q; q = from_euler_get_quaternion(roll, pitch, yaw);
    return q;
}



quaternion m2quaternion(float mx, float my, float mz) {
    // Convert magnetometer vector to quaternion
    // Placeholder implementation; replace with actual conversion logic
    return quaternion(0.0, mx, my, mz);
}

class MadgwickFilter {
public:
    float beta; // Algorithm gain
    quaternion q; // Estimated orientation quaternion
    
    MadgwickFilter(float beta, accel_data a) {
        // Initialize quaternion from accelerometer data
        beta = beta;
        q = a2quaternion(a.x, a.y, a.z);
        normalize(q);
    }

    
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        // Compute the quaternion derivative from gyroscope data
        quaternion q_dot = compute_quaternion_derivative(&q, gx, gy, gz, dt);
        
        // Compute the gradient descent correction
        quaternion gradient = get_magdwick_gradient();
        
        // Apply feedback step
        //q_dot = quaternion_product(&q_dot, &beta);
        q_dot = quaternion_product(&q_dot, &gradient);
        
        // Integrate to yield new quaternion
        q.w += q_dot.w * dt;
        q.x += q_dot.x * dt;
        q.y += q_dot.y * dt;
        q.z += q_dot.z * dt;
        
        // Normalize the quaternion
        normalize(q);
    }
    
    void normalize(quaternion &q) {
        float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm == 0) {
            q = quaternion(1.0, 0.0, 0.0, 0.0); // Reset to identity if norm is zero
            return;
        }
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    
    //quaternion get_orientation() const {
    //    return q;
    //}
};

int _main_magdwick() {
    quaternion q1(1.0, 2.0, 3.0, 4.0);
    quaternion q2(5.0, 6.0, 7.0, 8.0);
    
    // Example usage of quaternion_product
    quaternion q3 = quaternion_product(&q1, &q2);
    printf("Quaternion q3: (%f, %f, %f, %f)\n", q3.w, q3.x, q3.y, q3.z);
    
    printf("Quaternion q1: (%f, %f, %f, %f)\n", q1.w, q1.x, q1.y, q1.z);
    printf("Quaternion q2: (%f, %f, %f, %f)\n", q2.w, q2.x, q2.y, q2.z);
    
    return 0;
}











