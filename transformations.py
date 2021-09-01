import numpy as np

class Euler(object):

    def __init__(self, roll, pitch, yaw):
        self._euler = np.array([roll, pitch, yaw])

    @staticmethod
    def from_numpy_array(array):
        array = np.asarray(array)
        assert array.shape == (3,)
        return Euler(array[0], array[1], array[2])

    @staticmethod
    def zero():
        return Euler(0, 0, 0)

    @property
    def roll(self):
        return self._euler[0]

    @roll.setter
    def roll(self, value):
        self._euler[0] = value

    @property
    def pitch(self):
        return self._euler[1]

    @pitch.setter
    def pitch(self, value):
        self._euler[1] = value

    @property
    def yaw(self):
        return self._euler[2]

    @yaw.setter
    def yaw(self, value):
        self._euler[2] = value

    def rotate(self, amount):
        self._euler += amount

    def rotated(self, amount):
        return Euler(self.roll + amount[0], self.pitch + amount[1], self.yaw + amount[2])

    def __repr__(self):
        return "Euler(roll=%g, pitch=%g, yaw=%g)" % (self.roll, self.pitch, self.yaw)


def body_to_world_matrix(euler):
    """
    Body frame to world frame. Returns transformation matrix
    """
    return np.transpose(world_to_body_matrix(euler))


def world_to_body_matrix(euler):
 
    roll = euler.roll
    pitch = euler.pitch
    yaw = euler.yaw

    Cy = np.cos(yaw)
    Sy = np.sin(yaw)
    Cp = np.cos(pitch)
    Sp = np.sin(pitch)
    Cr = np.cos(roll)
    Sr = np.sin(roll)

    matrix = np.array(
        [[Cy * Cp, Sy * Cp, -Sp],
         [Cy * Sp * Sr - Cr * Sy, Cr * Cy + Sr * Sy * Sp, Cp * Sr],
         [Cy * Sp * Cr + Sr * Sy, Cr * Sy * Sp - Cy * Sr, Cr * Cp]])


    return matrix


def body_to_world(euler, vector):
    """
    Transforms a direction vector from body to world coordinates.
    """
    return np.dot(body_to_world_matrix(euler), vector)


def world_to_body(euler, vector):
    """
    Transforms a direction vector from world to body coordinates.
    """
    return np.dot(world_to_body_matrix(euler), vector)


def body_to_world_z(euler):

    return body_to_world(euler, [0, 0, 1])


def world_to_body_z(euler):

    return world_to_body(euler, [0, 0, 1])


def angular_velocity_to_euler_matrix(euler):

    roll = euler.roll
    pitch = euler.pitch

    Cp = np.cos(pitch)
    Sp = np.sin(pitch)
    Cr = np.cos(roll)
    Sr = np.sin(roll)

    matrix = np.array([
        [1,   0,   -Sp],
        [0,  Cr, Cp*Sr],
        [0, -Sr, Cp*Cr]
    ])
    #euler.add_to_cache("angvel_euler", matrix)
    return matrix


def angvel_to_euler(euler, angular_velocity):
    """ calculate the rate of change of euler angles for a given angular velocity."""
    return np.dot(angular_velocity_to_euler_matrix(euler), angular_velocity)


def euler_to_angvel(euler, deuler):
    """calculate the angular velocity given a rate of change for the euler angles."""
    return np.dot(np.linalg.inv(angular_velocity_to_euler_matrix(euler)), deuler)


def normalize_angle(angle):
    
    #Normalizes an angle (in radians) to the interval [0, 2pi].

    return np.remainder(angle, 2 * np.pi)


def angle_difference(a, b):

    n = normalize_angle(a - b)
    # [0, 2pi]
    if n > np.pi:
        return n - 2*np.pi
    else:
        return n