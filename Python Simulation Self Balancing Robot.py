import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
import matplotlib.pyplot as plt
import pygame
import sys
import tkinter as tk

# System parameters
mass = 0.064
length = 0.4
gravity = 9.81
damping = 9.0
spring_k = 3.5
time_step = 0.01

# State-Space Representation (linearized about the upright position)
A = np.array([
    [0, 1, 0, 0],
    [0, -damping / mass, gravity / length, 0],
    [0, 0, 0, 1],
    [0, -spring_k / mass, 0, 0]
])
B = np.array([[0], [1 / mass], [0], [1 / (mass * length)]])

# Desired pole locations for Pole Placement
desired_poles = [-2, -2.5, -3, -3.5]
K = place_poles(A, B, desired_poles).gain_matrix  # Feedback gain matrix


# Pendulum dynamics with damping and spring stiffness
def pendulum_dynamics(t, state, force_control):
    pos, angle, vel_pos, vel_angle = state
    sin_theta, cos_theta = np.sin(angle), np.cos(angle)
    effective_mass = mass + 1

    spring_force = -spring_k * pos

    angle_accel = (gravity * sin_theta - cos_theta * (
            force_control + spring_force) / effective_mass -
                 damping * vel_angle) / (length * (4 / 3 - mass * cos_theta ** 2 / effective_mass))
    pos_accel = (force_control + spring_force - damping * vel_pos) / effective_mass
    return [vel_pos, vel_angle, pos_accel, angle_accel]


# Stability check function
def check_stability(angle_vals, pos_vals, angle_thresh=0.05, pos_thresh=0.1, win_size=100):
    if len(angle_vals) < win_size or len(pos_vals) < win_size:
        return False
    recent_angles = np.abs(np.array(angle_vals[-win_size:]) - np.pi)
    recent_positions = np.abs(np.array(pos_vals[-win_size:]))
    return np.all(recent_angles < angle_thresh) and np.all(recent_positions < pos_thresh)


# Simulation variables setup
state = [0, np.pi + 0.1, 0, 0]
time_vals, angle_vals, pos_vals, force_vals = [], [], [], []
time_current = 0

# PID Controller with anti-windup
class PIDController:
    def __init__(self, kp, ki, kd, limit=None, dead_zone=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_err, self.integral_sum = 0, 0
        self.limit = limit
        self.dead_zone = dead_zone

    def update(self, error, time_step):
        if abs(error) < self.dead_zone:
            error = 0

        proportional = self.kp * error
        self.integral_sum += error * time_step
        integral = self.ki * self.integral_sum
        derivative = self.kd * (error - self.prev_err) / time_step
        self.prev_err = error

        output = proportional + integral + derivative

        if self.limit is not None:
            if abs(output) > self.limit:
                self.integral_sum *= 0.9
            output = max(min(output, self.limit), -self.limit)

        return output


# Initialize PID Controller
pid_controller = PIDController(kp=1.5, ki=0.05, kd=10.0, limit=5.0)


# Tkinter GUI for controller switching
def create_control_panel():
    root = tk.Tk()
    root.title("Controller Selection")

    # Label for the current controller
    controller_label = tk.Label(root, text="Current Controller: PID", font=("Arial", 14))
    controller_label.pack(pady=10)

    # Function to toggle controller
    def toggle_controller():
        global current_controller  # Use global instead of nonlocal
        if current_controller == "PID":
            current_controller = "Pole Placement"
        else:
            current_controller = "PID"
        controller_label.config(text=f"Current Controller: {current_controller}")

    # Button for toggling controller
    toggle_button = tk.Button(root, text="Switch Controller", command=toggle_controller, font=("Arial", 12))
    toggle_button.pack(pady=20)

    return root


# Initialize Tkinter control panel
current_controller = "PID"  # Default to PID controller
control_panel = create_control_panel()

# Real-time plot setup for monitoring system behavior
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
ax1.set_title("Pendulum Angle (θ in radians)")
ax2.set_title("Cart Position (x in meters)")
ax3.set_title("Control Force (N)")

(line1,) = ax1.plot([], [], label="Pendulum Angle (rad)", color="blue")
(line2,) = ax2.plot([], [], label="Cart Position (m)", color="green")
(line3,) = ax3.plot([], [], label="Control Force (N)", color="red")

# Function to update plot data
def update_plot():
    line1.set_data(time_vals, angle_vals)
    line2.set_data(time_vals, pos_vals)
    line3.set_data(time_vals, force_vals)
    for ax in (ax1, ax2, ax3):
        ax.set_xlabel("Time (s)")
        ax.relim()
        ax.autoscale_view()
    plt.draw()
    plt.pause(0.001)


# Initialize pygame
pygame.init()
screen_width, screen_height = 800, 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Inverted Pendulum Simulation")

# Colors for visualization
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)  # Pendulum color
GREY = (169, 169, 169)  # Grey color for wheels (tire color)
RED = (255, 0, 0)  # Pendulum tip color

# Visualization variables
cart_width, cart_height = 60, 30
pendulum_length = 150
plate_thickness = 5  # Thickness of each thin plate
plate_gap = 3  # Gap between the plates
num_plates = 3
box_height = cart_height // 3  # Box between cart and pendulum
wheel_radius = 10
wheel_offset = cart_width // 3  # Position of the wheels
clock = pygame.time.Clock()

# Main simulation loop
while True:
    # Update Tkinter GUI
    control_panel.update()

    # Handle Pygame events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Select control force based on the active controller
    if current_controller == "PID":
        error = state[1] - np.pi  # Error from desired angle (upward position)
        control_force = pid_controller.update(error, time_step)
    else:  # Pole Placement
        x = np.array([[state[0]], [state[2]], [state[1]], [state[3]]])  # State vector
        control_force = float(-K @ x)  # Compute control force

    # Integrate dynamics
    sol = solve_ivp(pendulum_dynamics, [0, time_step], state, args=(control_force,), t_eval=[time_step])
    state = sol.y[:, -1]

    # Data collection for plots
    time_vals.append(time_current)
    angle_vals.append(state[1])
    pos_vals.append(state[0])
    force_vals.append(control_force)
    time_current += time_step

    # Real-time plotting
    update_plot()

    # Stability check
    print("System Stable" if check_stability(angle_vals, pos_vals) else "System Unstable")

    # Pygame display for cart and pendulum
    screen.fill(WHITE)

    # Define cart position
    cart_x = screen_width // 2 + int(state[0] * 100)
    cart_y = screen_height // 2

    # Draw the cart as three thin plates
    for i in range(num_plates):
        plate_y = cart_y - cart_height // 2 + i * (plate_thickness + plate_gap)
        pygame.draw.rect(screen, BLACK, (cart_x - cart_width // 2, plate_y, cart_width, plate_thickness))

        # Add visual support pins between the plates
        if i > 0:
            pin_x = cart_x - cart_width // 4
            pin_y_top = plate_y - plate_gap
            pin_y_bottom = plate_y
            pygame.draw.line(screen, BLACK, (pin_x, pin_y_top), (pin_x, pin_y_bottom), 2)
            pin_x = cart_x + cart_width // 4
            pygame.draw.line(screen, BLACK, (pin_x, pin_y_top), (pin_x, pin_y_bottom), 2)

    # Add box between cart and pendulum (1/3 length of the cart)
    box_y = cart_y - cart_height // 2 - plate_gap * (num_plates - 1) - plate_thickness
    pygame.draw.rect(screen, BLACK, (cart_x - cart_width // 6, box_y, cart_width // 3, box_height))

    # Attach wheels directly to the bottom of the cart (no gap)
    wheel_y = cart_y + cart_height // 2 + wheel_radius  # Wheels directly under the cart
    pygame.draw.circle(
        screen, GREY,  # Change tire color to grey
        (cart_x - wheel_offset, wheel_y), wheel_radius
    )
    pygame.draw.circle(
        screen, GREY,  # Change tire color to grey
        (cart_x + wheel_offset, wheel_y), wheel_radius
    )

    # Calculate pendulum tip position
    pendulum_x = cart_x + pendulum_length * np.sin(state[1])
    pendulum_y = cart_y - cart_height // 2 - plate_gap * (num_plates - 1) - plate_thickness + pendulum_length * np.cos(state[1])

    # Draw the pendulum
    pendulum_origin_y = cart_y - cart_height // 2 - plate_gap * (num_plates - 1) - plate_thickness
    pygame.draw.line(screen, BLUE, (cart_x, pendulum_origin_y), (pendulum_x, pendulum_y), 5)

    # Draw the pendulum tip (black)
    pygame.draw.circle(screen, BLACK, (int(pendulum_x), int(pendulum_y)), 10)

    # Update display
    pygame.display.flip()
    clock.tick(60)









