# include <stdio.h>
# include <math.h>
# include "pico_control/quaternions.cpp"
# include <vector>
# include "lib/eigen/Eigen/Core"


constexpr int nstate = 22;
constexpr int nmeas = 10;
constexpr float g = 9.81f;
constexpr float atmospheric_pressure = 101325.0f;
constexpr float R = 287.05f;
constexpr float lapse_rate = 0.0065f;
constexpr float sea_level_temperature = 288.15f;
constexpr float exponent = g/(R * lapse_rate);


using namespace std;
using covariance_matrix_process = Eigen::Matrix<float, nstate, nstate>;
using vector3d = Eigen::Matrix<float, 3, 1>;
using covariance_matrix_measurement = Eigen::Matrix<float, nmeas, nmeas>;
using jac_matrix_process = Eigen::Matrix<float, nstate, nstate>;
using jac_matrix_measurement = Eigen::Matrix<float, nmeas, nstate>;
using error_jac_matrix_process = Eigen::Matrix<float, nstate - 1, nstate - 1>;

constexpr vector3d gvec = vector3d(0, 0, g);    



struct state{
    vector3d position;
    vector3d velocity;
    vector3d acceleration;
    quaternion q;
    vector3d omega;
    vector3d gyro_bias;
    vector3d accel_bias;
};

struct measurement{
    vector3d accel;
    vector3d gyro;
    vector3d mag;
    float pressure;
};

struct error_state{
    vector3d position_error;
    vector3d velocity_error;
    vector3d acceleration_error;
    quaternion q_error;
    vector3d omega_error;
    vector3d gyro_bias_error;
    vector3d accel_bias_error;
};

enum class stateIndex : size_t {
    x, y, z,
    vx,vy,vz,
    ax, ay, az,
    qw,qx,qy,qz,
    wx,wy,wz,
    gyro_bias_x,gyro_bias_y,gyro_bias_z,
    accel_bias_x,accel_bias_y,accel_bias_z,
};


constexpr int operator+(stateIndex i) {
    return static_cast<int>(i);
}


enum class errorIndex : size_t {
    x, y, z,
    vx,vy,vz,
    ax,ay,az,
    roll, pitch, yaw,
    wx,wy,wz,
    gyro_bias_x,gyro_bias_y,gyro_bias_z,
    accel_bias_x,accel_bias_y,accel_bias_z,
};


enum class measIndex : size_t {
    ax,ay,az,
    wx,wy,wz,
    mx,my,mz,
    p
};
constexpr int operator+(measIndex i) {
    return static_cast<int>(i);
}
constexpr int operator+(errorIndex i) {
    return static_cast<int>(i);
}

Eigen::Matrix<float, nstate, 1> state2vector(state s){
    Eigen::Matrix<float, nstate, 1> v;
    v << s.position(0), s.position(1), s.position(2),
         s.velocity(0), s.velocity(1), s.velocity(2),
         s.acceleration(0), s.acceleration(1), s.acceleration(2),
         s.q.w, s.q.x, s.q.y, s.q.z,
         s.omega(0), s.omega(1), s.omega(2),
         s.gyro_bias(0), s.gyro_bias(1), s.gyro_bias(2),
         s.accel_bias(0), s.accel_bias(1), s.accel_bias(2);
    return v;
}

state vector2state(Eigen::Matrix<float, nstate, 1> v){
    state s;
    s.position << v(0), v(1), v(2);
    s.velocity << v(3), v(4), v(5);
    s.acceleration << v(6), v(7), v(8);
    s.q.w = v(9);
    s.q.x = v(10);
    s.q.y = v(11);
    s.q.z = v(12);
    s.omega << v(13), v(14), v(15);
    s.gyro_bias << v(16), v(17), v(18);
    s.accel_bias << v(19), v(20), v(21);
    return s;
}

class ESKF{

    public:
        state current_state;
        covariance_matrix_process process_noise; // State covariance matrix
        covariance_matrix_measurement measurement_noise; // Process noise covariance matrix
        float dt; // Time step for the filter

        ESKF(float dt, state drone_state){
            
            this->dt = dt;
            this->current_state = drone_state;
            this->process_noise = covariance_matrix_process::Identity() * 0.01; // Initialize process noise covariance
            this->measurement_noise = covariance_matrix_measurement::Identity() * 0.01; // Initialize measurement noise covariance
        }

        ESKF(float dt){
            
            this->dt = dt;
            this->current_state = state();
            this->process_noise = covariance_matrix_process::Identity() * 0.01; // Initialize process noise covariance
            this->measurement_noise = covariance_matrix_measurement::Identity() * 0.01; // Initialize measurement noise covariance
        }


    state update_state(state drone_state, measurement z){
        // Implement the state update based on the measurement operator
        // This function should return the updated state based on the current state and the measurement

        // predict the next state using the forward dynamics

        // update noise covariance matrices based on the current state and the measurement

        // compute jacobian matrices for the forward dynamics and the measurement operator

        // solve system in error space

        // update the current state based on the error state
        state updated_state;

        this->current_state = drone_state; // Update the current state with the new state
        return this->current_state;        
    }

    state forward_dynamics(){
        // Implement the forward model for the measurement operator
        // This function should return the predicted state based on the current state and the measurement model
        state drone_state = this->current_state;
        float dt = this->dt;
        state predicted_state;
        predicted_state.position = drone_state.position + drone_state.velocity * dt + 0.5 * drone_state.acceleration * dt * dt;
        predicted_state.velocity = drone_state.velocity + drone_state.acceleration * dt;
        predicted_state.acceleration = drone_state.acceleration;
        predicted_state.q = drone_state.q * vector_to_quaternion_exponential_map(0.5 * drone_state.omega * dt);
        predicted_state.omega = drone_state.omega;
        predicted_state.gyro_bias = drone_state.gyro_bias;
        predicted_state.accel_bias = drone_state.accel_bias;
        return predicted_state;

    }

    jac_matrix_process jacobian_dynamics(){
        // Implement the Jacobian of the forward model for the measurement operator
        // This function should return the Jacobian matrix based on the current state and the measurement model
       jac_matrix_process jacobian_matrix; 
       jacobian_matrix.block<3,3>(+errorIndex::x, +errorIndex::x).setIdentity();
       jacobian_matrix.block<3,3>(+errorIndex::x, +errorIndex::vx).diagonal().setConstant(this->dt);
       jacobian_matrix.block<3,3>(+errorIndex::x, +errorIndex::ax).diagonal().setConstant(0.5*this->dt*this->dt);

       jacobian_matrix.block<3,3>(+errorIndex::vx, +errorIndex::vx).setIdentity();
       jacobian_matrix.block<3,3>(+errorIndex::x, +errorIndex::ax).diagonal().setConstant(this->dt);

       jacobian_matrix.block<3,3>(+errorIndex::ax, +errorIndex::ax).setIdentity();

       jacobian_matrix.block<3,3>(+errorIndex::roll, +errorIndex::roll).setIdentity();// TODO: Implement the correct Jacobian for the quaternion error state
       jacobian_matrix.block<3,3>(+errorIndex::roll, +errorIndex::wx).setIdentity();//TODO: Implement the correct Jacobian for the quaternion error state

       jacobian_matrix.block<3,3>(+errorIndex::wx, +errorIndex::wx).setIdentity();

       jacobian_matrix.block<3,3>(+errorIndex::gyro_bias_x, +errorIndex::gyro_bias_x).setIdentity();
       jacobian_matrix.block<3,3>(+errorIndex::accel_bias_x, +errorIndex::accel_bias_x).setIdentity();

       return jacobian_matrix;
        
    }

    measurement forward_measurement(){
        // Implement the measurement model for the measurement operator
        // This function should return the predicted measurement based on the current state and the measurement model
        measurement predicted_measurement;
        state drone_state = this->current_state;
        predicted_measurement.accel = expected_acceleration(drone_state);
        predicted_measurement.gyro = expected_angular_velocity(drone_state);
        predicted_measurement.mag = expected_magneticfield(drone_state);
        predicted_measurement.pressure = expected_pressure(drone_state);
        return predicted_measurement;
    }

    jac_matrix_measurement jacobian_measurement(state expected_state){
        // Implement the Jacobian of the measurement model for the measurement operator
        // This function should return the Jacobian matrix based on the current state and the measurement model


        jac_matrix_measurement jacobian_matrix;   
        Eigen::Matrix<float, 4, 3> dq_dtheta = expected_state.q.dqdtheta();
        Eigen::Matrix<float, 4, 4> dmeas_dq;

        
        vector3d expected_accel = expected_acceleration(expected_state);
        
        jacobian_matrix.block<3,3>(+measIndex::ax, +errorIndex::ax) = expected_state.q.to_rotation_matrix().block<3,3>(0,0);
        jacobian_matrix.block<3,3>(+measIndex::ax, +errorIndex::accel_bias_x).setIdentity();

        // meas a = q * (a + g) * q_conjugate + accel_bias
        // dmeas/dq = Lq * La * Mconj +    Rqconj Ra
        quaternion qa = quaternion(expected_accel);
        quaternion qg = quaternion(gvec);
        quaternion q = qa + qg;
        dmeas_dq = (expected_state.q.to_left_matrix()*q.to_left_matrix()*to_conjugate_matrix() +
         expected_state.q.to_right_matrix()*q.to_right_matrix());

        jacobian_matrix.block<3,3>(+measIndex::ax, +errorIndex::roll) = q2vector_matrix() * dmeas_dq * dq_dtheta;
        
        jacobian_matrix.block<3,3>(+measIndex::wx, +errorIndex::wx).setIdentity();

        jacobian_matrix.block<3,3>(+measIndex::wx, +errorIndex::gyro_bias_x).setIdentity();

        vector3d expected_mag = expected_magneticfield(expected_state);
        q = quaternion(expected_mag);
        dmeas_dq = (expected_state.q.to_left_matrix()*q.to_left_matrix()*to_conjugate_matrix() +
         expected_state.q.to_right_matrix()*q.to_right_matrix());
        jacobian_matrix.block<3,3>(+measIndex::mx, +errorIndex::roll) = q2vector_matrix() * dmeas_dq * dq_dtheta;

        float expected_p = expected_pressure(expected_state);
        jacobian_matrix(+measIndex::p, +errorIndex::z) = -exponent * lapse_rate / (sea_level_temperature - lapse_rate * expected_state.position(2)) * expected_p;

        return jacobian_matrix;



    }


    vector3d expected_acceleration(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
        vector3d expected_accel;
        expected_accel = drone_state.accel_bias + drone_state.q.rotate_vector(drone_state.acceleration + vector3d(0, 0, -g));
        return expected_accel;
    }
  
    vector3d expected_angular_velocity(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
        vector3d expected_gyro;
        expected_gyro = drone_state.omega + drone_state.gyro_bias;
        return expected_gyro;
    }

    vector3d expected_magneticfield(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
        vector3d expected_mag;
        // Assuming the magnetic field is aligned with the world frame's z-axis
        vector3d world_magnetic_field(0, 1, 0); 
        expected_mag = drone_state.q.rotate_vector(world_magnetic_field);
        return expected_mag;
    }

    float expected_pressure(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
        float expected_pressure;
        float altitude = drone_state.position(2);
        
        expected_pressure = atmospheric_pressure * pow(1 - (lapse_rate * altitude) / sea_level_temperature, exponent);
        return expected_pressure;
    }


};


