import matplotlib.pyplot as plt

# --- DEINE ORIGINAL KLASSE ---
class Controller:
    def __init__(self):
        self.last_error = 0
        self.first_run = True # Kleine Ergänzung für den Start-Schlag

    def PDController(self, target: float, actual: float, kp: float, kd: float, dt: float) -> float:
        ACCEL = 0.05 # Annahme für die Konstante
        error = target - actual
    
        # Fix für den ersten Schritt (verhindert D-Sprung von 0 auf Zielwert)
        if self.first_run:
            self.last_error = error
            self.first_run = False

        error_diff = (error - self.last_error) / dt
        self.last_error = error

        p_out = kp * error
        d_out = kd * error_diff

        combined = p_out + d_out

        if combined > ACCEL:
            accel = ACCEL
        elif combined < -ACCEL:
            accel = -ACCEL
        else:
            accel = combined
        
        return accel

# --- SIMULATIONS-SETUP ---
controller = Controller()
target_pos = 0.1
current_pos = 0.0
current_vel = 0.0
accel = 0.0

dt_sim = 0.01  # 100 Hz (Integrator/Roboter)
dt_regler = 0.1 # 10 Hz (Regler-Takt)
sim_time = 50.0 # Sekunden
steps = int(sim_time / dt_sim)

history_pos = []
history_time = []

# Simulationsschleife
for i in range(steps):
    t = i * dt_sim
    
    # Der Regler läuft nur alle 0.1 Sekunden (wie in ROS2)
    if i % 10 == 0:
        accel = controller.PDController(target_pos, current_pos, 1.0, 2.0, dt_regler)
    
    # DEINE DOPPELINTEGRATION (100 Hz)
    current_vel += accel * dt_sim
    current_pos += current_vel * dt_sim
    
    history_pos.append(current_pos)
    history_time.append(t)

# Visualisierung
plt.figure(figsize=(10, 5))
plt.plot(history_time, history_pos, label="Position (Python Simulation)")
plt.axhline(y=target_pos, color='r', linestyle='--', label="Ziel")
plt.title(f"PD-Regler Simulation (Regler: 10Hz, Integration: 100Hz)\nKp=1, Kd=2")
plt.xlabel("Zeit [s]")
plt.ylabel("Position")
plt.legend()
plt.grid(True)
plt.show()