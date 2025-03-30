# -*- coding: utf-8 -*-
"""
Wrapper for the Quadcopter_SimCon simulation that allows a user to
input a target point and visualize the drone flying to that location.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from numpy import pi

simulation_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Quadcopter_SimCon', 'Simulation')
sys.path.insert(0, simulation_dir)

# Import necessary modules
from trajectory import Trajectory
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config
from matplotlib.widgets import Button
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation


class CustomTrajectory(Trajectory):
    """Extends the Trajectory class to allow user-defined waypoints"""

    def __init__(self, quad, ctrlType, trajSelect, prev_point, target_point):
        super().__init__(quad, ctrlType, trajSelect)
        self.prev_point = prev_point
        self.target_point = target_point

        # Create waypoints for flight
        t_wps, wps, y_wps, v_wp = self.custom_waypoints(prev_point, target_point)
        self.t_wps = t_wps
        self.wps = wps
        self.y_wps = y_wps
        self.v_wp = v_wp

    def custom_waypoints(self, prev_point, target_point):
        """Create a simple trajectory from origin to target point"""
        t_ini = 0  # Initial hover time
        v_average = 10.0  # Average velocity in m/s
        t = np.array([0.1])

        # Starting position and target
        wp_ini = np.array(prev_point)
        wp = np.array([target_point])

        # Yaw angles (radians)
        yaw_ini = 0
        yaw = np.array([0, 0])

        # Create arrays in format expected by parent class
        t_combined = np.hstack((t_ini, t)).astype(float)
        wp_combined = np.vstack((wp_ini, wp)).astype(float)
        yaw_combined = np.hstack((yaw_ini, yaw)).astype(float) * (pi / 180)

        return t_combined, wp_combined, yaw_combined, v_average


def run_quadcopter_sim(prev_point, target_point):
    """Run the quadcopter simulation with a user-defined target point"""
    print(f"Starting simulation to target point: {target_point}")

    # Simulation Setup
    Ti = 0
    Ts = 0.005
    Tf = 5  # Simulation time

    # Trajectory and control settings
    trajSelect = np.zeros(3)
    ctrlType = "xyz_pos"
    trajSelect[0] = 1  # Select Position Trajectory Type
    trajSelect[1] = 0  # Select Yaw Trajectory Type
    trajSelect[2] = 0  # Use average speed for waypoint time

    # Initialize simulation components
    quad = Quadcopter(Ti)
    quad.pos[0:3] = prev_point  # Set initial position
    traj = CustomTrajectory(quad, ctrlType, trajSelect, prev_point, target_point)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 0.0, 0, 0)  # No wind

    # Initial desired states and commands
    sDes = traj.desiredState(0, Ts, quad)
    ctrl.controller(traj, quad, sDes, Ts)

    # Initialize result matrices
    numTimeStep = int(Tf / Ts + 1)
    t_all = np.zeros(numTimeStep)
    pos_all = np.zeros([numTimeStep, len(quad.pos)])
    quat_all = np.zeros([numTimeStep, len(quad.quat)])
    sDes_traj_all = np.zeros([numTimeStep, len(traj.sDes)])
    thr_all = np.zeros([numTimeStep, 4])  # Store thrust values for all 4 motors

    t_all[0] = Ti
    pos_all[0, :] = quad.pos
    quat_all[0, :] = quad.quat
    sDes_traj_all[0, :] = traj.sDes

    # Run Simulation
    t = Ti
    i = 1

    while round(t, 3) < Tf:
        # Dynamics (using last timestep's commands)
        quad.update(t, Ts, ctrl.w_cmd, wind)
        t += Ts

        # Trajectory for Desired States
        sDes = traj.desiredState(t, Ts, quad)

        # Generate Commands (for next iteration)
        ctrl.controller(traj, quad, sDes, Ts)

        t_all[i] = t
        pos_all[i, :] = quad.pos
        quat_all[i, :] = quad.quat
        sDes_traj_all[i, :] = traj.sDes
        thr_all[i, :] = quad.thr  # Store the current thrust values
        i += 1

    # Create and show animation
    ani = custom_sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all,
                                   Ts, quad.params, traj.xyzType, traj.yawType, quad, thr_all)

    plt.show(block=True)
    plt.close()  # Force close any remaining figures

    return ani


def custom_sameAxisAnimation(t_all, waypoints, pos_all, quat_all, sDes_tr_all, Ts, params, xyzType, yawType, quad,
                             thr_all):
    """Custom implementation of sameAxisAnimation with fixed axes and thrust vectors"""

    numFrames = 8

    x = pos_all[:, 0]
    y = pos_all[:, 1]
    z = pos_all[:, 2]

    xDes = sDes_tr_all[:, 0]
    yDes = sDes_tr_all[:, 1]
    zDes = sDes_tr_all[:, 2]

    x_wp = waypoints[:, 0]
    y_wp = waypoints[:, 1]
    z_wp = waypoints[:, 2]

    # Using NED coordinate system
    z = -z
    zDes = -zDes
    z_wp = -z_wp

    # Create figure with space at bottom for button
    fig = plt.figure(figsize=(10, 8))
    ax = p3.Axes3D(fig, auto_add_to_figure=False, rect=[0.05, 0.1, 0.9, 0.85])
    fig.add_axes(ax)

    line1, = ax.plot([], [], [], lw=2, color='red')
    line2, = ax.plot([], [], [], lw=2, color='blue')
    line3, = ax.plot([], [], [], '--', lw=1, color='blue')

    # Lines for individual motor thrust vectors
    thrust_vectors = []
    for i in range(4):
        thrust_line, = ax.plot([], [], [], lw=1, color='green', marker='^', markersize=1)
        thrust_vectors.append(thrust_line)

    # Setting fixed axes limits (NED system)
    ax.set_xlim3d([0, 5])
    ax.set_xlabel('X')
    ax.set_ylim3d([5, 0])  # Reversed for NED
    ax.set_ylabel('Y')
    ax.set_zlim3d([0, 5])
    ax.set_zlabel('Altitude')

    thrustText = ax.text2D(0.05, 0.9, "", transform=ax.transAxes, family='monospace')
    # Draw waypoints
    ax.scatter(x_wp, y_wp, z_wp, color='green', alpha=1, marker='o', s=50)  # Make them bigger (s=50)

    # Add a trajectory line
    if xyzType > 0:
        ax.plot(xDes, yDes, zDes, ':', lw=1.3, color='green')

    def updateLines(i):
        time = t_all[i * numFrames]
        pos = pos_all[i * numFrames]
        x = pos[0]
        y = pos[1]
        z = pos[2]

        x_from0 = pos_all[0:i * numFrames, 0]
        y_from0 = pos_all[0:i * numFrames, 1]
        z_from0 = pos_all[0:i * numFrames, 2]

        dxm = params["dxm"]
        dym = params["dym"]
        dzm = params["dzm"]

        quat = quat_all[i * numFrames]

        # Convert to NED for display
        z_display = -z
        z_from0_display = -z_from0
        quat_display = np.array([quat[0], -quat[1], -quat[2], quat[3]])

        R = utils.quat2Dcm(quat_display)
        motorPoints = np.array(
            [[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
        motorPoints = np.dot(R, np.transpose(motorPoints))
        motorPoints[0, :] += x
        motorPoints[1, :] += y
        motorPoints[2, :] += z_display

        line1.set_data(motorPoints[0, 0:3], motorPoints[1, 0:3])
        line1.set_3d_properties(motorPoints[2, 0:3])
        line2.set_data(motorPoints[0, 3:6], motorPoints[1, 3:6])
        line2.set_3d_properties(motorPoints[2, 3:6])
        line3.set_data(x_from0, y_from0)
        line3.set_3d_properties(z_from0_display)

        # Constants for thrust normalization
        MIN_THRUST = 0.1
        MAX_THRUST = 9.18

        # Get the current thrust values for each motor
        motor_thrusts = thr_all[i * numFrames]

        # Calculate the normalized percentage for each motor
        thrust_percentages = [(t - MIN_THRUST) / (MAX_THRUST - MIN_THRUST) * 100 for t in motor_thrusts]

        thrust_grid = f"Motor Thrusts:\n"
        thrust_grid += f"[{thrust_percentages[0]:.1f}%, {thrust_percentages[1]:.1f}%]\n"
        thrust_grid += f"[{thrust_percentages[3]:.1f}%, {thrust_percentages[2]:.1f}%]"
        thrustText.set_text(thrust_grid)

        # Motor positions in the drone's local frame
        motor_positions = [
            [dxm, -dym, dzm],  # Front right
            [dxm, dym, dzm],  # Back right
            [-dxm, dym, dzm],  # Back left
            [-dxm, -dym, dzm]  # Front left
        ]

        # Transform motor positions to global frame
        global_motor_positions = []
        for motor_pos in motor_positions:
            motor_pos_global = np.dot(R, np.array(motor_pos))
            motor_x = x + motor_pos_global[0]
            motor_y = y + motor_pos_global[1]
            motor_z = z_display + motor_pos_global[2]
            global_motor_positions.append([motor_x, motor_y, motor_z])

        # Get z-axis unit vector from rotation matrix for thrust direction
        z_axis = R[:, 2]  # Extract the 3rd column (z-axis direction)

        # Update each motor's thrust vector
        for j in range(4):
            # Scale thrust vector based on this motor's thrust percentage
            thrust_scale = 2 * (motor_thrusts[j] / MAX_THRUST)
            thrust_vector = z_axis * thrust_scale

            # Set the start point at the motor position
            start_x, start_y, start_z = global_motor_positions[j]
            end_x = start_x + thrust_vector[0]
            end_y = start_y + thrust_vector[1]
            end_z = start_z + thrust_vector[2]

            # Update this motor's thrust vector line
            thrust_vectors[j].set_data([start_x, end_x], [start_y, end_y])
            thrust_vectors[j].set_3d_properties([start_z, end_z])

        # Clean up existing motor labels
        for txt in ax.texts[:]:
            if hasattr(txt, 'motor_index'):
                txt.remove()

        # Add motor labels
        for j in range(4):
            start_x, start_y, start_z = global_motor_positions[j]
            label_offset = 0.1
            txt = ax.text(start_x + label_offset, start_y + label_offset, start_z + label_offset,
                          f"{j + 1}", color='green', fontweight='bold')
            txt.motor_index = j + 1

        return [line1, line2] + thrust_vectors

    def ini_plot():
        line1.set_data(np.empty([1]), np.empty([1]))
        line1.set_3d_properties(np.empty([1]))
        line2.set_data(np.empty([1]), np.empty([1]))
        line2.set_3d_properties(np.empty([1]))
        line3.set_data(np.empty([1]), np.empty([1]))
        line3.set_3d_properties(np.empty([1]))

        for vector in thrust_vectors:
            vector.set_data(np.empty([1]), np.empty([1]))
            vector.set_3d_properties(np.empty([1]))

        return [line1, line2, line3] + thrust_vectors

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, updateLines, init_func=ini_plot,
                                       frames=len(t_all[0:-2:numFrames]),
                                       interval=(Ts * 1000 * numFrames), blit=False)

    # Animation is created but not saved

    # Add the continue button
    ax_button = plt.axes([0.35, 0.01, 0.3, 0.05])  # [left, bottom, width, height]
    btn_continue = Button(ax_button, 'Continue to Next Destination')

    # Button click handler
    def on_button_click(event):
        plt.close(fig)

    btn_continue.on_clicked(on_button_click)
    plt.show(block=True)  # Use block=True to ensure it blocks until closed
    plt.close()

    return line_ani


def main():
    """Main function to get user input and run the simulation"""
    print("Quadcopter Simulation - Go to Target Point")

    prev_point = [0, 0, 0]

    while True:
        x = float(input("X coordinate (m): "))
        y = float(input("Y coordinate (m): "))
        z = float(input("Z coordinate (m): "))

        # In NED, z is negative for altitude
        target_point = np.array([x, y, -z])

        # Run the simulation
        print("Running simulation to target point...")

        # Run simulation
        ani = run_quadcopter_sim(prev_point, target_point)

        # Update previous point for next iteration
        prev_point = target_point


if __name__ == "__main__":
    main()