import math
import time
import threading
import tkinter as tk
from tkinter import ttk

import pygame
import pymunk


# =========================================================
# ESTADO COMPARTIDO ENTRE LA GUI Y LA SIMULACIÓN
# =========================================================
state_lock = threading.Lock()

STATE = {
    # -----------------------------
    # PARÁMETROS FÍSICOS DEL PÉNDULO
    # -----------------------------
    "length": 2.0,                  # Longitud de la cuerda (m)
    "mass": 1.0,                    # Masa de la bola (kg)
    "bob_radius": 0.12,             # Radio visual de la bola (m)
    "pivot_radius": 0.04,           # Radio visual del pivote (m)
    "initial_angle_deg": 35.0,      # Ángulo inicial respecto a la vertical (grados)
    "initial_speed_tangent": 0.0,   # Velocidad inicial tangencial (m/s)

    # -----------------------------
    # AMORTIGUAMIENTOS
    # -----------------------------
    "global_damping": 0.05,
    "pendulum_damping": 0.10,

    "gravity": 9.81,
    "time_step": 1.0 / 240.0,

    # En Pymunk usamos 2D.
    # El pivote está en coordenadas físicas, no en píxeles.
    "pivot_position": [0.0, 4.0],

    "real_time": True,
    "running": False,
    "reset_requested": False,
    "print_requested": False,
    "exit_requested": False
}


# =========================================================
# CONFIGURACIÓN VISUAL
# =========================================================
WIDTH, HEIGHT = 900, 700
PIXELS_PER_METER = 120

BACKGROUND = (245, 245, 245)
BLACK = (20, 20, 20)
WHITE = (255, 255, 255)
BLUE = (40, 80, 220)
RED = (210, 40, 40)
GRAY = (120, 120, 120)


# =========================================================
# CONVERSIÓN DE COORDENADAS
# =========================================================
def world_to_screen(pos):
    """
    Convierte coordenadas físicas de Pymunk a coordenadas de pantalla.

    Pymunk:
        x hacia la derecha
        y hacia arriba

    Pygame:
        x hacia la derecha
        y hacia abajo
    """
    x, y = pos

    screen_x = WIDTH // 2 + x * PIXELS_PER_METER
    screen_y = HEIGHT - 80 - y * PIXELS_PER_METER

    return int(screen_x), int(screen_y)


# =========================================================
# CREACIÓN DEL PÉNDULO
# =========================================================
def create_pendulum(space, params):
    """
    Crea un péndulo simple en Pymunk.

    Elementos:
    - Un pivote estático.
    - Una bola dinámica.
    - Una restricción PinJoint que mantiene fija la distancia.
    """
    length = params["length"]
    mass = params["mass"]
    bob_radius = params["bob_radius"]
    pivot_radius = params["pivot_radius"]

    pivot_x, pivot_y = params["pivot_position"]

    angle = math.radians(params["initial_angle_deg"])

    # Si theta = 0, la bola está justo debajo del pivote.
    bob_x = pivot_x + length * math.sin(angle)
    bob_y = pivot_y - length * math.cos(angle)

    # -----------------------------
    # PIVOTE ESTÁTICO
    # -----------------------------
    pivot_body = pymunk.Body(body_type=pymunk.Body.STATIC)
    pivot_body.position = pivot_x, pivot_y

    pivot_shape = pymunk.Circle(pivot_body, pivot_radius)
    pivot_shape.sensor = True

    space.add(pivot_body, pivot_shape)

    # -----------------------------
    # BOLA DINÁMICA
    # -----------------------------
    moment = pymunk.moment_for_circle(
        mass,
        0,
        bob_radius
    )

    bob_body = pymunk.Body(mass, moment)
    bob_body.position = bob_x, bob_y

    bob_shape = pymunk.Circle(bob_body, bob_radius)
    bob_shape.elasticity = 0.0
    bob_shape.friction = 0.5

    space.add(bob_body, bob_shape)

    # -----------------------------
    # CUERDA FÍSICA
    # -----------------------------
    # PinJoint mantiene constante la distancia entre pivote y bola.
    joint = pymunk.PinJoint(
        pivot_body,
        bob_body,
        (0, 0),
        (0, 0)
    )

    joint.distance = length

    space.add(joint)

    # -----------------------------
    # VELOCIDAD INICIAL TANGENCIAL
    # -----------------------------
    vt = params["initial_speed_tangent"]

    # Vector tangente al movimiento.
    # Para el ángulo medido desde la vertical:
    # posición radial = (sin(theta), -cos(theta))
    # tangente = (cos(theta), sin(theta))
    tangent_x = math.cos(angle)
    tangent_y = math.sin(angle)

    bob_body.velocity = (
        vt * tangent_x,
        vt * tangent_y
    )

    return pivot_body, pivot_shape, bob_body, bob_shape, joint


def remove_pendulum(space, objects):
    """
    Elimina el péndulo actual del espacio.
    """
    for obj in objects:
        try:
            space.remove(obj)
        except Exception:
            pass


# =========================================================
# MAGNITUDES FÍSICAS
# =========================================================
def get_angle_deg(bob_body, pivot_position):
    """
    Devuelve el ángulo respecto a la vertical.

    theta = 0 significa que la bola está justo debajo del pivote.
    """
    pivot_x, pivot_y = pivot_position

    rx = bob_body.position.x - pivot_x
    ry = bob_body.position.y - pivot_y

    return math.degrees(math.atan2(rx, -ry))


def get_speed(bob_body):
    """
    Devuelve el módulo de la velocidad de la bola.
    """
    vx, vy = bob_body.velocity
    return math.sqrt(vx ** 2 + vy ** 2)


def apply_damping(bob_body, params):
    """
    Aplica amortiguamiento manual sobre la velocidad.

    En PyBullet se usaba linearDamping.
    En Pymunk lo simulamos reduciendo la velocidad en cada paso.
    """
    total_damping = params["global_damping"] + params["pendulum_damping"]
    dt = params["time_step"]

    if total_damping <= 0:
        return

    damping_factor = max(0.0, 1.0 - total_damping * dt)

    bob_body.velocity = (
        bob_body.velocity.x * damping_factor,
        bob_body.velocity.y * damping_factor
    )

    bob_body.angular_velocity *= damping_factor


# =========================================================
# DIBUJO
# =========================================================
def draw_pendulum(screen, pivot_body, bob_body, params):
    """
    Dibuja el pivote, la cuerda y la bola.
    """
    pivot_pos = pivot_body.position
    bob_pos = bob_body.position

    pivot_screen = world_to_screen(pivot_pos)
    bob_screen = world_to_screen(bob_pos)

    bob_radius_px = max(4, int(params["bob_radius"] * PIXELS_PER_METER))
    pivot_radius_px = max(3, int(params["pivot_radius"] * PIXELS_PER_METER))

    # Cuerda
    pygame.draw.line(
        screen,
        BLACK,
        pivot_screen,
        bob_screen,
        3
    )

    # Pivote
    pygame.draw.circle(
        screen,
        BLUE,
        pivot_screen,
        pivot_radius_px
    )

    pygame.draw.circle(
        screen,
        BLACK,
        pivot_screen,
        pivot_radius_px,
        1
    )

    # Bola
    pygame.draw.circle(
        screen,
        RED,
        bob_screen,
        bob_radius_px
    )

    pygame.draw.circle(
        screen,
        BLACK,
        bob_screen,
        bob_radius_px,
        2
    )

    # Marcador de rotación de la bola
    marker_angle = -bob_body.angle

    marker_x = bob_screen[0] + int(math.cos(marker_angle) * bob_radius_px * 0.65)
    marker_y = bob_screen[1] + int(math.sin(marker_angle) * bob_radius_px * 0.65)

    pygame.draw.circle(
        screen,
        WHITE,
        (marker_x, marker_y),
        max(3, bob_radius_px // 6)
    )


def draw_ground(screen):
    """
    Dibuja una línea de referencia inferior.
    """
    y = HEIGHT - 80

    pygame.draw.line(
        screen,
        GRAY,
        (0, y),
        (WIDTH, y),
        2
    )


def draw_debug_text(screen, bob_body, params):
    """
    Dibuja información física en pantalla.
    """
    font = pygame.font.SysFont("Consolas", 18)

    pivot = params["pivot_position"]
    angle_deg = get_angle_deg(bob_body, pivot)
    speed = get_speed(bob_body)

    total_damping = params["global_damping"] + params["pendulum_damping"]

    lines = [
        f"angulo: {angle_deg:.2f} grados",
        f"velocidad: {speed:.3f} m/s",
        f"longitud: {params['length']:.2f} m",
        f"masa: {params['mass']:.2f} kg",
        f"damping global: {params['global_damping']:.2f}",
        f"damping pendulo: {params['pendulum_damping']:.2f}",
        f"damping total: {total_damping:.2f}",
        f"gravedad: {params['gravity']:.2f} m/s²",
    ]

    x, y = 20, 20

    for i, line in enumerate(lines):
        text_surface = font.render(line, True, BLACK)
        screen.blit(text_surface, (x, y + i * 24))


# =========================================================
# BUCLE PRINCIPAL DE SIMULACIÓN
# =========================================================
def simulation_loop():
    """
    Hilo principal de simulación con Pymunk + Pygame.
    """
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pymunk · Péndulo amortiguado")

    clock = pygame.time.Clock()

    with state_lock:
        params = dict(STATE)

    space = pymunk.Space()
    space.gravity = (0.0, -params["gravity"])

    pendulum_objects = create_pendulum(space, params)

    show_debug_text = False

    while True:
        with state_lock:
            params = dict(STATE)

        if params["exit_requested"]:
            break

        # -----------------------------------------
        # Eventos de Pygame
        # -----------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                with state_lock:
                    STATE["exit_requested"] = True
                break

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    with state_lock:
                        STATE["exit_requested"] = True
                    break

        # -----------------------------------------
        # Reset del péndulo
        # -----------------------------------------
        if params["reset_requested"]:
            remove_pendulum(space, pendulum_objects)

            with state_lock:
                STATE["reset_requested"] = False
                params = dict(STATE)

            space = pymunk.Space()
            space.gravity = (0.0, -params["gravity"])

            pendulum_objects = create_pendulum(space, params)

            if params["real_time"]:
                time.sleep(params["time_step"])

            continue

        # -----------------------------------------
        # Actualización de gravedad
        # -----------------------------------------
        space.gravity = (0.0, -params["gravity"])

        pivot_body, pivot_shape, bob_body, bob_shape, joint = pendulum_objects

        # -----------------------------------------
        # Avance físico
        # -----------------------------------------
        if params["running"]:
            apply_damping(bob_body, params)
            space.step(params["time_step"])

        # -----------------------------------------
        # Petición de imprimir datos
        # -----------------------------------------
        if params["print_requested"]:
            show_debug_text = True

            angle_deg = get_angle_deg(bob_body, params["pivot_position"])
            speed = get_speed(bob_body)

            print("===================================")
            print(f"Ángulo: {angle_deg:.2f} grados")
            print(f"Velocidad: {speed:.3f} m/s")
            print(f"Longitud: {params['length']:.2f} m")
            print(f"Masa: {params['mass']:.2f} kg")
            print(f"Damping global: {params['global_damping']:.2f}")
            print(f"Damping péndulo: {params['pendulum_damping']:.2f}")
            print(f"Damping total: {params['global_damping'] + params['pendulum_damping']:.2f}")
            print(f"Gravedad: {params['gravity']:.2f} m/s²")
            print("===================================")

            with state_lock:
                STATE["print_requested"] = False

        # -----------------------------------------
        # Dibujo
        # -----------------------------------------
        screen.fill(BACKGROUND)

        draw_ground(screen)
        draw_pendulum(screen, pivot_body, bob_body, params)

        if show_debug_text:
            draw_debug_text(screen, bob_body, params)

        pygame.display.flip()

        # -----------------------------------------
        # Tiempo real
        # -----------------------------------------
        if params["real_time"]:
            time.sleep(params["time_step"])

        clock.tick(240)

    pygame.quit()


# =========================================================
# FUNCIONES DE CONTROL DESDE TKINTER
# =========================================================
def set_running(value: bool):
    with state_lock:
        STATE["running"] = value


def request_reset():
    with state_lock:
        STATE["reset_requested"] = True
        STATE["running"] = False


def request_print():
    with state_lock:
        STATE["print_requested"] = True


def request_exit(root):
    with state_lock:
        STATE["exit_requested"] = True

    root.destroy()


def update_float(key, value):
    with state_lock:
        STATE[key] = float(value)


# =========================================================
# INTERFAZ DE CONTROL
# =========================================================
def build_control_window():
    root = tk.Tk()
    root.title("Control del péndulo amortiguado · Pymunk")
    root.geometry("460x820")
    root.resizable(True, True)

    root.protocol("WM_DELETE_WINDOW", lambda: request_exit(root))

    frame = ttk.Frame(root, padding=12)
    frame.pack(fill="both", expand=True)

    ttk.Label(
        frame,
        text="Parámetros",
        font=("Segoe UI", 12, "bold")
    ).pack(anchor="w", pady=(0, 10))

    def make_slider(label, key, frm, to):
        container = ttk.Frame(frame)
        container.pack(fill="x", pady=6)

        top_row = ttk.Frame(container)
        top_row.pack(fill="x")

        ttk.Label(top_row, text=label).pack(side="left")

        value_label = ttk.Label(top_row, text=f"{STATE[key]:.3f}")
        value_label.pack(side="right")

        scale = tk.Scale(
            container,
            from_=frm,
            to=to,
            orient="horizontal",
            resolution=0.01,
            length=400,
            command=lambda v, k=key, lbl=value_label: (
                update_float(k, v),
                lbl.config(text=f"{float(v):.3f}")
            )
        )

        scale.set(STATE[key])
        scale.pack(fill="x")

    # Sliders físicos
    make_slider("Longitud", "length", 0.5, 5.0)
    make_slider("Masa", "mass", 0.1, 10.0)
    make_slider("Ángulo inicial (deg)", "initial_angle_deg", -170.0, 170.0)
    make_slider("Velocidad tangencial inicial", "initial_speed_tangent", -10.0, 10.0)
    make_slider("Damping global", "global_damping", 0.0, 1.0)
    make_slider("Damping propio del péndulo", "pendulum_damping", 0.0, 1.0)
    make_slider("Gravedad", "gravity", 0.0, 20.0)

    ttk.Separator(frame).pack(fill="x", pady=12)

    ttk.Label(
        frame,
        text="Controles",
        font=("Segoe UI", 11, "bold")
    ).pack(anchor="w", pady=(0, 8))

    buttons1 = ttk.Frame(frame)
    buttons1.pack(fill="x", pady=4)

    ttk.Button(
        buttons1,
        text="Iniciar",
        command=lambda: set_running(True)
    ).pack(side="left", expand=True, fill="x", padx=4)

    ttk.Button(
        buttons1,
        text="Pausa",
        command=lambda: set_running(False)
    ).pack(side="left", expand=True, fill="x", padx=4)

    buttons2 = ttk.Frame(frame)
    buttons2.pack(fill="x", pady=4)

    ttk.Button(
        buttons2,
        text="Reset",
        command=request_reset
    ).pack(side="left", expand=True, fill="x", padx=4)

    ttk.Button(
        buttons2,
        text="Imprimir datos",
        command=request_print
    ).pack(side="left", expand=True, fill="x", padx=4)

    buttons3 = ttk.Frame(frame)
    buttons3.pack(fill="x", pady=4)

    ttk.Button(
        buttons3,
        text="Salir",
        command=lambda: request_exit(root)
    ).pack(side="left", expand=True, fill="x", padx=4)

    info = (
        "Longitud, masa, ángulo inicial y velocidad inicial\n"
        "se aplican al pulsar Reset.\n"
        "El damping y la gravedad se actualizan en vivo.\n"
        "Q o ESC cierran la simulación.\n"
        "La simulación se ejecuta en una ventana Pygame."
    )

    ttk.Label(
        frame,
        text=info,
        justify="left"
    ).pack(anchor="w", pady=(14, 0))

    root.mainloop()


# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":
    sim_thread = threading.Thread(target=simulation_loop, daemon=True)
    sim_thread.start()

    build_control_window()

    sim_thread.join(timeout=1.0)