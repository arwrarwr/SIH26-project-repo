import argparse
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# --- 155mm M107 Shell Physical Constants ---
MASS = 43.2           # kg
DIAMETER = 0.155      # meters
AREA = np.pi * (DIAMETER / 2)**2
LENGTH = 0.605        # meters
IX = 0.15             # Axial moment of inertia (kg*m^2)
IY = 1.30             # Transverse moment of inertia (kg*m^2)
IZ = 1.30

# --- Environmental Constants ---
G0 = 9.80665          # Sea level gravity (m/s^2)
R_EARTH = 6371000.0   # Earth radius (meters)
OMEGA_EARTH = np.array([0, 0, 7.2921159e-5]) # Earth angular velocity (rad/s)
MU_AIR = 1.81e-5      # Dynamic viscosity of air

# --- Coordinate Translation ---
def haversine(lat1, lon1, lat2, lon2):
    """Returns distance (meters) and initial bearing (radians)."""
    R_e = 6371e3
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    distance = R_e * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
    bearing = math.atan2(y, x)
    return distance, bearing

# --- Environmental Physics Models ---
def atmosphere(altitude):
    """International Standard Atmosphere (ISA) Model for Troposphere."""
    if altitude < 0: altitude = 0
    T0 = 288.15
    P0 = 101325.0
    rho0 = 1.225
    L = 0.0065 # Lapse rate
    
    if altitude < 11000:
        T = T0 - L * altitude
        P = P0 * (1 - L * altitude / T0) ** 5.255
        rho = rho0 * (1 - L * altitude / T0) ** 4.255
    else:
        # Simplified Stratosphere
        T = 216.65
        P = 22632 * math.exp(-0.000157 * (altitude - 11000))
        rho = P / (287.05 * T)
        
    speed_of_sound = math.sqrt(1.4 * 287.05 * T)
    return rho, speed_of_sound

def gravity(altitude):
    """Altitude-dependent gravity."""
    return G0 * (R_EARTH / (R_EARTH + altitude))**2

# --- Aerodynamic Coefficients ---
def get_aero_coeffs(mach):
    """Approximated Mach-dependent coefficients for a 155mm shell."""
    # Transonic drag rise
    if mach < 0.9:
        cd = 0.15
    elif mach < 1.2:
        cd = 0.15 + 0.2 * (mach - 0.9) / 0.3
    else:
        cd = 0.35 - 0.1 * (mach - 1.2) / 1.8
        
    cl_alpha = 1.5       # Lift coefficient per radian
    c_mag = 0.2          # Magnus force coefficient
    cm_alpha = -2.5      # Pitching moment (Overturning)
    cm_q = -5.0          # Pitch damping
    cn_p = -0.05         # Spin damping
    cm_mag = 0.3         # Magnus moment
    return cd, cl_alpha, c_mag, cm_alpha, cm_q, cn_p, cm_mag

# --- 6-DOF Integrator (RK4 Step) ---
def rk4_step(state, dt, use_aero=True):
    """
    State vector: [X, Y, Z, Vx, Vy, Vz, q0, q1, q2, q3, p, q, r]
    X, Y = downrange/crossrange, Z = altitude
    """
    def derivatives(s):
        pos = s[0:3]
        vel = s[3:6]
        quat = s[6:10]
        omega = s[10:13] # Body angular rates

        alt = pos[2]
        g = gravity(alt)
        rho, sos = atmosphere(alt)
        
        # Wind & Relative Velocity
        wind = np.array([0.0, 0.0, 0.0]) # Can inject wind profile here
        v_rel = vel - wind
        v_mag = np.linalg.norm(v_rel)
        mach = v_mag / sos if sos > 0 else 0

        # Coriolis
        a_coriolis = -2.0 * np.cross(OMEGA_EARTH, vel)
        
        # Orientation
        rotation = R.from_quat(quat) # quat format: [x,y,z,w]
        
        if not use_aero or v_mag == 0:
            a_tot = np.array([0, 0, -g]) + a_coriolis
            alpha_dot = np.zeros(3)
        else:
            # Body axes
            u_axis = rotation.apply(np.array([1, 0, 0])) # Nose pointing vector
            
            # Angle of attack calculation
            v_dir = v_rel / v_mag
            alpha = math.acos(np.clip(np.dot(u_axis, v_dir), -1.0, 1.0))
            
            # Cross products for force directions
            lift_dir = np.cross(np.cross(u_axis, v_dir), v_dir)
            if np.linalg.norm(lift_dir) > 0: lift_dir = lift_dir / np.linalg.norm(lift_dir)
            
            magnus_dir = np.cross(u_axis, v_dir)
            if np.linalg.norm(magnus_dir) > 0: magnus_dir = magnus_dir / np.linalg.norm(magnus_dir)

            # Coefficients
            cd, cl_alpha, c_mag, cm_alpha, cm_q, cn_p, cm_mag = get_aero_coeffs(mach)
            q_dyn = 0.5 * rho * v_mag**2
            
            # Translational Forces
            f_drag = -cd * q_dyn * AREA * v_dir
            f_lift = cl_alpha * alpha * q_dyn * AREA * lift_dir
            
            # Magnus force uses spin rate (omega[0])
            spin_ratio = (omega[0] * DIAMETER) / (2 * v_mag)
            f_magnus = c_mag * spin_ratio * q_dyn * AREA * magnus_dir
            
            f_aero = f_drag + f_lift + f_magnus
            a_tot = (f_aero / MASS) + np.array([0, 0, -g]) + a_coriolis

            # Rotational Moments (calculated in body frame)
            moment = np.zeros(3)
            # Overturning moment tries to flip the nose
            moment[1] += cm_alpha * alpha * q_dyn * AREA * LENGTH 
            # Damping moments
            moment[0] += cn_p * (omega[0] * LENGTH / (2*v_mag)) * q_dyn * AREA * LENGTH
            moment[1] += cm_q * (omega[1] * LENGTH / (2*v_mag)) * q_dyn * AREA * LENGTH
            moment[2] += cm_q * (omega[2] * LENGTH / (2*v_mag)) * q_dyn * AREA * LENGTH
            
            I = np.diag([IX, IY, IZ])
            alpha_dot = np.linalg.inv(I).dot(moment - np.cross(omega, I.dot(omega)))

        # Quaternion derivative
        p, q_rate, r = omega
        q_dot = 0.5 * np.array([
            quat[3]*p - quat[2]*q_rate + quat[1]*r,
            quat[2]*p + quat[3]*q_rate - quat[0]*r,
            -quat[1]*p + quat[0]*q_rate + quat[3]*r,
            -quat[0]*p - quat[1]*q_rate - quat[2]*r
        ])

        return np.concatenate((vel, a_tot, q_dot, alpha_dot))

    k1 = derivatives(state)
    k2 = derivatives(state + 0.5 * dt * k1)
    k3 = derivatives(state + 0.5 * dt * k2)
    k4 = derivatives(state + dt * k3)
    
    new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    # Normalize quaternion safely
    q_norm = np.linalg.norm(new_state[6:10])
    if q_norm > 1e-8:
        new_state[6:10] = new_state[6:10] / q_norm
    else:
        new_state[6:10] = np.array([0.0, 0.0, 0.0, 1.0]) # Fallback to identity
    return new_state

# --- Simulation Execution ---
def simulate_trajectory(elevation_deg, bearing_rad, v_muzzle=827.0, use_aero=True):
    """Runs a single trajectory and returns the impact distance and path."""
    elev = math.radians(elevation_deg)
    
    # Initial state
    pos = np.array([0.0, 0.0, 0.0])
    vel = np.array([v_muzzle * math.cos(elev) * math.cos(bearing_rad),
                    v_muzzle * math.cos(elev) * math.sin(bearing_rad),
                    v_muzzle * math.sin(elev)])
    
    # Align initial quaternion with velocity vector
    quat = R.from_euler('ZYX', [bearing_rad, elev, 0]).as_quat()
    
    # 15,000 RPM spin rate -> 1570 rad/s
    omega = np.array([1570.0, 0.0, 0.0]) 
    
    state = np.concatenate((pos, vel, quat, omega))
    
    dt = 0.001
    path = []
    
    while state[2] >= 0:
        path.append(state.copy())
        state = rk4_step(state, dt, use_aero)
        # Fail-safe to prevent infinite orbit
        if len(path) * dt > 120: break 
        
    return np.array(path)

# --- Firing Solution Finder ---
def find_firing_solution(target_dist, bearing_rad):
    """Uses a binary search (Shooting Method) to find the correct elevation angle."""
    print("Calculating firing solution (Shooting Method)...")
    min_angle = 1.0
    max_angle = 60.0
    best_angle = 45.0
    
    for _ in range(15): # 15 iterations is usually enough for convergence
        mid_angle = (min_angle + max_angle) / 2
        path = simulate_trajectory(mid_angle, bearing_rad, use_aero=True)
        impact_dist = np.linalg.norm(path[-1][0:2])
        
        if impact_dist < target_dist:
            min_angle = mid_angle
        else:
            max_angle = mid_angle
        best_angle = mid_angle
        
    return best_angle

# --- Plotting Logic ---
def plot_trajectories(actual_path, ideal_path, target_dist):
    """Generates 3D and 2D plots comparing the actual and ideal trajectories."""
    print("Generating plots...")
    
    fig = plt.figure(figsize=(14, 6))

    # --- Plot 1: 3D Trajectory ---
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Plot the paths
    ax1.plot(actual_path[:, 0], actual_path[:, 1], actual_path[:, 2], 
             label='Actual (Aero + Spin)', color='red', linewidth=2)
    ax1.plot(ideal_path[:, 0], ideal_path[:, 1], ideal_path[:, 2], 
             label='Ideal (Vacuum)', color='blue', linestyle='--', alpha=0.7)
    
    ax1.set_xlabel('Downrange X (m)')
    ax1.set_ylabel('Crossrange Y (m)')
    ax1.set_zlabel('Altitude Z (m)')
    ax1.set_title('3D Artillery Trajectory')
    ax1.legend()

    # --- Plot 2: 2D Side Profile (Altitude vs. Ground Range) ---
    ax2 = fig.add_subplot(122)
    
    # Calculate horizontal ground distance for each point in the arrays
    actual_range = np.sqrt(actual_path[:, 0]**2 + actual_path[:, 1]**2)
    ideal_range = np.sqrt(ideal_path[:, 0]**2 + ideal_path[:, 1]**2)
    
    ax2.plot(actual_range, actual_path[:, 2], label='Actual Path', color='red', linewidth=2)
    ax2.plot(ideal_range, ideal_path[:, 2], label='Ideal Path', color='blue', linestyle='--')
    
    # Mark the target location
    ax2.axvline(x=target_dist, color='green', linestyle=':', linewidth=2, label=f'Target ({target_dist:.1f}m)')
    
    ax2.set_xlabel('Ground Range (m)')
    ax2.set_ylabel('Altitude Z (m)')
    ax2.set_title('Side Profile: Altitude vs. Range')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--lat0', type=float, required=True, help="Launch Latitude")
    parser.add_argument('--lon0', type=float, required=True, help="Launch Longitude")
    parser.add_argument('--lat1', type=float, required=True, help="Target Latitude")
    parser.add_argument('--lon1', type=float, required=True, help="Target Longitude")
    args = parser.parse_args()

    # 1. Target Data
    distance, bearing = haversine(args.lat0, args.lon0, args.lat1, args.lon1)
    print(f"Target Distance: {distance:.1f} meters")
    print(f"Target Bearing:  {math.degrees(bearing):.2f} degrees")

    if distance > 30000:
        print("Warning: Target is beyond standard unassisted 155mm range.")

    # 2. Find Firing Solution
    elevation = find_firing_solution(distance, bearing)
    print(f"Calculated Firing Elevation: {elevation:.3f} degrees\n")

    # 3. Generate Paths
    print("Simulating Actual Path (Atmosphere + Aero + Coriolis...)")
    actual_path = simulate_trajectory(elevation, bearing, use_aero=True)
    
    print("Simulating Ideal Path (Vacuum / Gravity Only)")
    ideal_path = simulate_trajectory(elevation, bearing, use_aero=False)
    
    # 4. Compare Impacts
    actual_impact = np.linalg.norm(actual_path[-1][0:2])
    ideal_impact = np.linalg.norm(ideal_path[-1][0:2])        
    print("\n--- RESULTS ---")
    print(f"Actual Impact Distance: {actual_impact:.1f} meters")
    print(f"Actual Apogee (Max H):  {np.max(actual_path[:, 2]):.1f} meters")
    print(f"Ideal Impact Distance:  {ideal_impact:.1f} meters (Vacuum overshoot)")
    print(f"Ideal Apogee (Max H):   {np.max(ideal_path[:, 2]):.1f} meters")
        
    # 5. Display the Graphs
    plot_trajectories(actual_path, ideal_path, distance)
    
