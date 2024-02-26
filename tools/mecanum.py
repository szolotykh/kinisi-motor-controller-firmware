# Wheels orientation
# x
# |
# 1 - 2
# |   |
# 0 - 3 --> y

# TODO: Verify y axis direction

# Inverse kinematics of mecanum platform
def inverse_kinematics_mecanum(platform_target_velocity, R, L, W):
    x = platform_target_velocity['x']
    y = platform_target_velocity['y']
    t = platform_target_velocity['t']

    V1 = 1.0 / R * (x + y + (L + W) / 2 * t)
    V2 = 1.0 / R * (x - y + (L + W) / 2 * t)
    V3 = 1.0 / R * (x + y - (L + W) / 2 * t)
    V4 = 1.0 / R * (x - y - (L + W) / 2 * t)
    return {'v1': V1, 'v2': V2, 'v3': V3, 'v4': V4}

# Forward kinematics of mecanum platform
def forward_kinematics_mecanum(velocities, R, L, W):
    v1 = velocities['v1']
    v2 = velocities['v2']
    v3 = velocities['v3']
    v4 = velocities['v4']
    
    x = R/4.0 * (v1 + v2 + v3 + v4)
    y = R/4.0  * (v1 - v2 + v3 - v4)
    t = R / (2.0 * (L + W)) * (v1 + v2 - v3 - v4)
    
    return {'x': x, 'y': y, 't': t}

R = 0.1
L = 0.4
W = 0.3
platform_velocity = {'x': 15.0, 'y': 30.0, 't': 20.0}
print("Input velocities:" )
print(platform_velocity)

print("Inverse kinematics")
velocities = inverse_kinematics_mecanum(platform_velocity, R, L, W)
print(velocities)

print("Forward kinematics")
odometry = forward_kinematics_mecanum(velocities, R, L, W)
print(odometry)