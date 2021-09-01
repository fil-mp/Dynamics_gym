import math
import numpy as np

def linear_dynamics(params, state):
    """
    Linear acceleration calculation given the copter parameters
    """
    
    m = params.mass
    b = params.thrust_factor
    Kt = params.translational_drag
    O = state.rotor_speeds
    n = state.attitude
    v = state.velocity

    thrust = b/m * (O[0]**2 + O[1]**2 + O[2]**2 + O[3]**2) * body_to_world_z(n)
    #print("check", thrust)

    Ktw = np.dot(body_to_world_matrix(n), np.dot(np.diag(Kt), world_to_body_matrix(n)))
    drag = np.dot(Ktw, v) / m
    #print('check:',params.gravity-drag)

    return thrust - drag + params.gravity


def torques(params, state):

    Lb = params.arm_length * params.thrust_factor
    d = params.drag_factor
    O = state.rotor_speeds
    motor_torque = O[3] ** 2 + O[1] ** 2 - O[2] ** 2 - O[0] ** 2
    B = np.array([Lb * (O[3] ** 2 - O[1] ** 2), Lb * (O[0] ** 2 - O[2] ** 2), d * motor_torque])
    return B


def angular_momentum_body_frame(params, state):

    J = params.rotor_inertia
    I = params.frame_inertia
    O = state.rotor_speeds
    w = state.angular_velocity
    Kr = params.rotational_drag

    gyro = state.net_rotor_speed * J * np.array([w[2], -w[1], 0])
    drag = Kr * w
    Mp = torques(params, state)
    B = Mp - drag + gyro - np.cross(w, I*w)

    return B


def euler_rate(state):

    return angvel_to_euler(state.attitude, state.angular_velocity)

def simulate_quadrotor(params, state, dt):

    # let rotor speed approach desired rotor speed and prevent negative rotation
    gamma = 1.0 - 0.5**(dt / params.rotor_speed_half_time)
    dw = gamma * (state.desired_rotor_speeds - state.rotor_speeds)
    state._rotorspeeds += dw
    state._rotorspeeds = np.maximum(state._rotorspeeds, 0.0)
    
    acceleration =s linear_dynamics(params, state)
    ang_momentum = angular_momentum_body_frame(params, state)
    angular_acc = ang_momentum / params.frame_inertia

    state._position += 0.5 * dt * dt * acceleration + 0.5 * dt * state._velocity
    state._velocity += dt * acceleration

    state._angular_velocity += dt * angular_acc
    #R_dot=np.dot(skew(state),world_to_body_matrix(state._attitude))
    #print(R_dot)
    #print(mat)
    state._attitude.rotate(dt * euler_rate(state))