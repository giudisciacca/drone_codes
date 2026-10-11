# include <stdio.h>
# include <math.h>
# include <pico_control/quaternions.cpp>
# include <vector>
# include <Eigen/Core>

using namespace std;
using covariance_matrix_process = Eigen::Matrix<float, 22, 22>;
using vector3d = Eigen::Matrix<float, 3, 1>;
using covariance_matrix_measurement = Eigen::Matrix<float, 10, 10>;

using jac_matrix_process = Eigen::Matrix<float, 22, 22>;
using jac_matrix_measurement = Eigen::Matrix<float, 10, 22>;

using error_jac_matrix_process = Eigen::Matrix<float, 21, 21>;


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

    state update_state(state drone_state, measurement z){
        // Implement the state update based on the measurement operator
        // This function should return the updated state based on the current state and the measurement


        
        state updated_state;
    }

    state forward_dynamics(state drone_state){
        // Implement the forward model for the measurement operator
        // This function should return the predicted state based on the current state and the measurement model
        state predicted_state;
        predicted_state.position = drone_state.position + drone_state.velocity * dt + 0.5 * drone_state.acceleration * dt * dt;
        predicted_state.velocity = drone_state.velocity + drone_state.acceleration * dt;
        predicted_state.acceleration = drone_state.acceleration;
        predicted_state.q = drone_state.q * vector_to_quaternion_exponential_map(0.5 * drone_state.omega * dt);
        predicted_state.omega = drone_state.omega;
        predicted_state.gyro_bias = drone_state.gyro_bias;
        predicted_state.accel_bias = drone_state.accel_bias;

    }

    state jacobian_dynamics(state drone_state){
        // Implement the Jacobian of the forward model for the measurement operator
        // This function should return the Jacobian matrix based on the current state and the measurement model
        state jacobian_matrix;   
        jacobian_matrix[][]
    }

    measurement forward_measurement(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the predicted measurement based on the current state and the measurement model
        measurement predicted_measurement;
    }

    measurement jacobian_measurement(state drone_state){
        // Implement the Jacobian of the measurement model for the measurement operator
        // This function should return the Jacobian matrix based on the current state and the measurement model
        measurement jacobian_matrix;   
    }

    vector3d expected_measurement(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
    }
    
    vector3d expected_acceleration(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
    }
  
    vector3d expected_angular_velocity(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
    }

    vector3d expected_magneticfield(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
    }

    vector3d expected_pressure(state drone_state){
        // Implement the measurement model for the measurement operator
        // This function should return the expected measurement based on the current state
    }




};


