import math

def set_omni_platform_velocity(platform_velocity, R, L):
    V1 = 1/R * (math.sqrt(3.0)/2 * platform_velocity['x'] + 0.5 * platform_velocity['y'] + L * platform_velocity['t'])
    V2 = 1/R * (-math.sqrt(3.0)/2 * platform_velocity['x'] + 0.5 * platform_velocity['y'] + L * platform_velocity['t'])
    V3 = 1/R * (-platform_velocity['y'] + L*platform_velocity['t'])

    return {'v1': V1, 'v2': V2, 'v3':V3}


def omni_platform_forward_kinematics(velocities, R, L):
    v1 = velocities['v1']
    v2 = velocities['v2']
    v3 = velocities['v3']
    
    x = R * (1/math.sqrt(3)*v1 - 1/math.sqrt(3)*v2)
    y = R * (1/3*v1 + 1/3*v2 - 2/3 * v3)
    t = R * (1/(3*L)*v1 + 1/(3*L)*v2 + 1/(3*L)*v3)

    return {'x': x, 'y': y, 't': t}

R = 2
L = 3
platform_velocity = {'x': 100.0, 'y': 0.0, 't': 0.0}
print("Input velocities:" )
print(platform_velocity)

print("Inverse kinematics")
velocities = set_omni_platform_velocity(platform_velocity, R, L)
print(velocities)

print("Forward kinematics")
odometry = omni_platform_forward_kinematics(velocities, R, L)
print(odometry)