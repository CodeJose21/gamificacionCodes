import math
import queue
import threading
import time
from dataclasses import dataclass, asdict

import pygame
import pymunk
import tkinter as tk
from tkinter import ttk, messagebox


# ============================================================
# PROYECTO: DESLIZAMIENTO Y RODADURA DE UNA BOLA DE BOLOS
# VERSIÓN PYMUNK 2D
# ============================================================

G = 9.81
SIM_DT = 1.0 / 1000.0
STOP_SPEED = 0.01

PURE_ROLLING_REL_TOL = 0.01


# ============================================================
# PARÁMETROS DEL PROBLEMA
# ============================================================
@dataclass
class Parameters:
    mass: float = 7.0
    radius: float = 1.0
    hollow: bool = False

    mu_contact_pymunk: float = 0.22
    mu_rolling: float = 0.0

    v0: float = 8.0
    omega0: float = 0.0

    lane_length: float = 2500.0
    launch_height: float = 1.0001
    time_scale: float = 1.0

    @property
    def inertia_factor(self) -> float:
        return (2.0 / 3.0) if self.hollow else (2.0 / 5.0)

    @property
    def inertia(self) -> float:
        return self.inertia_factor * self.mass * self.radius ** 2

    @property
    def body_type_label(self) -> str:
        return "Hueca (cascarón fino)" if self.hollow else "Maciza"


# ============================================================
# RESULTADO TEÓRICO
# ============================================================
@dataclass
class TheoryResult:
    body_type: str
    inertia_factor: float
    mu_used: float | None
    initial_slip: float
    slip_sign: int
    t_to_pure_rolling: float
    x_to_pure_rolling: float
    v_at_pure_rolling: float
    omega_at_pure_rolling: float
    regime: str
    slip_decay_rate: float


def sign_nonzero(x: float) -> int:
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def compute_theory(params: Parameters, mu_used: float | None = None) -> TheoryResult:
    mu = params.mu_contact_pymunk if mu_used is None else mu_used

    s0 = params.v0 - params.radius * params.omega0
    k = params.inertia_factor

    if abs(s0) < 1e-12:
        return TheoryResult(
            body_type=params.body_type_label,
            inertia_factor=k,
            mu_used=mu,
            initial_slip=s0,
            slip_sign=0,
            t_to_pure_rolling=0.0,
            x_to_pure_rolling=0.0,
            v_at_pure_rolling=params.v0,
            omega_at_pure_rolling=params.omega0,
            regime="Ya sale en rodadura pura",
            slip_decay_rate=0.0,
        )

    if mu <= 0:
        return TheoryResult(
            body_type=params.body_type_label,
            inertia_factor=k,
            mu_used=mu,
            initial_slip=s0,
            slip_sign=sign_nonzero(s0),
            t_to_pure_rolling=math.inf,
            x_to_pure_rolling=math.inf,
            v_at_pure_rolling=math.nan,
            omega_at_pure_rolling=math.nan,
            regime="No alcanza rodadura pura porque μ = 0",
            slip_decay_rate=0.0,
        )

    sgn = sign_nonzero(s0)

    a = -mu * G * sgn
    alpha = (mu * G / (k * params.radius)) * sgn

    slip_decay_rate = mu * G * (1.0 + 1.0 / k)

    t_star = abs(s0) / slip_decay_rate
    v_star = params.v0 + a * t_star
    omega_star = params.omega0 + alpha * t_star
    x_star = params.v0 * t_star + 0.5 * a * t_star ** 2

    return TheoryResult(
        body_type=params.body_type_label,
        inertia_factor=k,
        mu_used=mu,
        initial_slip=s0,
        slip_sign=sgn,
        t_to_pure_rolling=t_star,
        x_to_pure_rolling=x_star,
        v_at_pure_rolling=v_star,
        omega_at_pure_rolling=omega_star,
        regime="Empieza deslizando" if s0 > 0 else "Sale con exceso de spin",
        slip_decay_rate=slip_decay_rate,
    )


# ============================================================
# SIMULACIÓN PYMUNK + PYGAME
# ============================================================
class BowlingSimulation(threading.Thread):
    def __init__(self, params: Parameters, ui_callback):
        super().__init__(daemon=True)

        self.params = params
        self.ui_callback = ui_callback

        self._commands = queue.Queue()
        self._stop_event = threading.Event()

        self.space = None
        self.ball_body = None
        self.ball_shape = None
        self.floor_shape = None

        self.running = False
        self.ball_launched = False
        self.ball_has_reached_rolling = False

        self.sim_time = 0.0
        self.last_ui_send_time = 0.0

        self.initial_slip = None
        self.mu_equiv_theory = None
        self.theory_initial = None
        self.theory_equiv = None

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

        self.screen = None
        self.clock = None

        self.WIDTH = 1100
        self.HEIGHT = 520
        self.PIXELS_PER_METER = 85

        self.camera_x = 0.0

    def post_command(self, cmd, payload=None):
        self._commands.put((cmd, payload))

    def stop(self):
        self._stop_event.set()
        self.post_command("quit")

    def send_ui(self, kind, data):
        self.ui_callback((kind, data))

    # --------------------------------------------------------
    # Mundo Pymunk
    # --------------------------------------------------------
    def reset_world(self):
        self.sim_time = 0.0
        self.last_ui_send_time = 0.0

        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)

        # Suelo estático
        floor_body = self.space.static_body
        self.floor_shape = pymunk.Segment(
            floor_body,
            (-1000.0, 0.0),
            (self.params.lane_length + 1000.0, 0.0),
            0.05,
        )
        self.floor_shape.friction = 0.0
        self.floor_shape.elasticity = 0.0
        self.space.add(self.floor_shape)

        # Bola
        moment = self.params.inertia

        self.ball_body = pymunk.Body(self.params.mass, moment)
        self.ball_body.position = (0.0, self.params.radius)
        self.ball_body.velocity = (0.0, 0.0)
        self.ball_body.angular_velocity = 0.0

        self.ball_shape = pymunk.Circle(self.ball_body, self.params.radius)
        self.ball_shape.friction = 0.0
        self.ball_shape.elasticity = 0.0

        self.space.add(self.ball_body, self.ball_shape)

        self.running = False
        self.ball_launched = False
        self.ball_has_reached_rolling = False

        self.initial_slip = None
        self.mu_equiv_theory = None
        self.theory_initial = None
        self.theory_equiv = None

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

        self.camera_x = 0.0

        self.send_ui("reset_done", {})

    def launch_ball(self):
        if self.ball_body is None:
            return

        self.theory_initial = compute_theory(self.params)
        self.initial_slip = self.params.v0 - self.params.radius * self.params.omega0

        self.sim_time = 0.0

        self.ball_body.position = (0.0, self.params.radius)
        self.ball_body.velocity = (self.params.v0, 0.0)

        # IMPORTANTE:
        # En Pymunk, angular_velocity positiva es antihoraria.
        # Para que la fórmula del proyecto siga siendo:
        # slip = v - R·ω
        # guardamos la ω física como -angular_velocity.
        self.ball_body.angular_velocity = -self.params.omega0

        self.running = True
        self.ball_launched = True
        self.ball_has_reached_rolling = False

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        self.mu_equiv_theory = None
        self.theory_equiv = None

        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

        self.send_ui("launched", {})

    def stop_ball_motion(self):
        if self.ball_body is None:
            return

        self.ball_body.velocity = (0.0, 0.0)
        self.ball_body.angular_velocity = 0.0

        self.running = False
        self.ball_launched = False

        self.send_ui("stopped", {})

    # --------------------------------------------------------
    # Comandos desde Tkinter
    # --------------------------------------------------------
    def _handle_commands(self):
        while True:
            try:
                cmd, payload = self._commands.get_nowait()
            except queue.Empty:
                break

            if cmd == "reset":
                self.params = payload
                self.reset_world()

            elif cmd == "update_params":
                self.params = payload

            elif cmd == "launch":
                self.launch_ball()

            elif cmd == "stop_motion":
                self.stop_ball_motion()

            elif cmd == "quit":
                self.running = False
                return False

        return True

    # --------------------------------------------------------
    # Bucle principal
    # --------------------------------------------------------
    def run(self):
        pygame.init()
        pygame.display.set_caption("Pymunk · Deslizamiento y rodadura")
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()

        self.reset_world()

        while not self._stop_event.is_set():
            if not self._handle_commands():
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.stop()
                    break

            self.step_simulation()
            self.draw()

            pygame.display.flip()
            self.clock.tick(60)

            time.sleep(SIM_DT / max(self.params.time_scale, 1e-6))

        pygame.quit()

    # --------------------------------------------------------
    # Física del deslizamiento / rodadura
    # --------------------------------------------------------
    def step_simulation(self):
        if self.space is None or self.ball_body is None:
            return

        vx = self.ball_body.velocity.x

        # ω física usada en la teoría
        omega = -self.ball_body.angular_velocity

        slip = vx - self.params.radius * omega

        speed_ref = max(abs(vx), abs(self.params.radius * omega), 1e-6)
        pure_rolling = abs(slip) / speed_ref <= PURE_ROLLING_REL_TOL

        if self.running and self.ball_launched:
            if not pure_rolling:
                self.apply_sliding_friction(vx, omega, slip)
            else:
                self.apply_rolling_motion(vx)

            self.sim_time += SIM_DT

        # Forzamos la bola a mantenerse sobre el suelo 2D
        self.ball_body.position = (
            self.ball_body.position.x,
            self.params.radius,
        )

        self.space.step(SIM_DT)

        self.check_rolling_transition()
        self.send_telemetry()

    def apply_sliding_friction(self, vx, omega, slip):
        mu = self.params.mu_contact_pymunk
        k = self.params.inertia_factor

        if mu <= 0:
            return

        sgn = sign_nonzero(slip)

        a = -mu * G * sgn
        alpha = (mu * G / (k * self.params.radius)) * sgn

        new_vx = vx + a * SIM_DT
        new_omega = omega + alpha * SIM_DT

        new_slip = new_vx - self.params.radius * new_omega

        # Si nos pasamos del punto de rodadura pura, ajustamos.
        if sign_nonzero(new_slip) != sgn:
            new_omega = new_vx / self.params.radius

        self.ball_body.velocity = (new_vx, 0.0)
        self.ball_body.angular_velocity = -new_omega

    def apply_rolling_motion(self, vx):
        mu_r = self.params.mu_rolling

        if abs(vx) <= STOP_SPEED:
            self.ball_body.velocity = (0.0, 0.0)
            self.ball_body.angular_velocity = 0.0
            self.running = False
            return

        if mu_r > 0:
            sgn = sign_nonzero(vx)
            new_vx = vx - mu_r * G * sgn * SIM_DT

            if sign_nonzero(new_vx) != sgn:
                new_vx = 0.0
        else:
            new_vx = vx

        new_omega = new_vx / self.params.radius

        self.ball_body.velocity = (new_vx, 0.0)
        self.ball_body.angular_velocity = -new_omega

    def check_rolling_transition(self):
        if not self.running or not self.ball_launched:
            return

        if self.ball_has_reached_rolling:
            return

        vx = self.ball_body.velocity.x
        omega = -self.ball_body.angular_velocity

        slip = vx - self.params.radius * omega
        speed_ref = max(abs(vx), abs(self.params.radius * omega), 1e-6)

        pure_rolling = abs(slip) / speed_ref <= PURE_ROLLING_REL_TOL

        if pure_rolling:
            self.ball_has_reached_rolling = True
            self.rolling_start_time = self.sim_time
            self.rolling_start_speed = vx
            self.rolling_start_omega = omega

            k = self.params.inertia_factor
            s0 = self.initial_slip

            if s0 is not None and abs(s0) > 1e-12 and self.rolling_start_time > 1e-12:
                self.mu_equiv_theory = abs(s0) / (
                    G * (1.0 + 1.0 / k) * self.rolling_start_time
                )

                self.theory_equiv = compute_theory(
                    self.params,
                    mu_used=self.mu_equiv_theory,
                )

            if self.theory_equiv is not None:
                if (
                    math.isfinite(self.theory_equiv.t_to_pure_rolling)
                    and self.theory_equiv.t_to_pure_rolling > 1e-12
                ):
                    self.error_t_percent = abs(
                        self.rolling_start_time - self.theory_equiv.t_to_pure_rolling
                    ) / self.theory_equiv.t_to_pure_rolling * 100.0

                if (
                    math.isfinite(self.theory_equiv.v_at_pure_rolling)
                    and abs(self.theory_equiv.v_at_pure_rolling) > 1e-12
                ):
                    self.error_v_percent = abs(
                        self.rolling_start_speed - self.theory_equiv.v_at_pure_rolling
                    ) / abs(self.theory_equiv.v_at_pure_rolling) * 100.0

                if (
                    math.isfinite(self.theory_equiv.omega_at_pure_rolling)
                    and abs(self.theory_equiv.omega_at_pure_rolling) > 1e-12
                ):
                    self.error_w_percent = abs(
                        self.rolling_start_omega - self.theory_equiv.omega_at_pure_rolling
                    ) / abs(self.theory_equiv.omega_at_pure_rolling) * 100.0

    def send_telemetry(self):
        if self.ball_body is None:
            return

        now = time.perf_counter()
        if now - self.last_ui_send_time < 1.0 / 30.0:
            return

        vx = self.ball_body.velocity.x
        omega = -self.ball_body.angular_velocity
        slip = vx - self.params.radius * omega

        speed_ref = max(abs(vx), abs(self.params.radius * omega), 1e-6)
        pure_rolling = abs(slip) / speed_ref <= PURE_ROLLING_REL_TOL

        if not self.ball_launched:
            regime = "En espera"
        else:
            if abs(vx) <= STOP_SPEED and abs(omega) <= STOP_SPEED / max(self.params.radius, 1e-9):
                regime = "Parada"
            elif pure_rolling:
                regime = "Rodadura pura"
            else:
                regime = "Deslizamiento"

        self.send_ui(
            "telemetry",
            {
                "t": self.sim_time,
                "x": self.ball_body.position.x,
                "vx": vx,
                "omega": omega,
                "slip": slip,
                "regime": regime,
                "rolling_start_time": self.rolling_start_time,
                "rolling_start_speed": self.rolling_start_speed,
                "rolling_start_omega": self.rolling_start_omega,
                "mu_equiv_theory": self.mu_equiv_theory,
                "error_t_percent": self.error_t_percent,
                "error_v_percent": self.error_v_percent,
                "error_w_percent": self.error_w_percent,
            },
        )

        self.last_ui_send_time = now

    # --------------------------------------------------------
    # Dibujo
    # --------------------------------------------------------
    def world_to_screen(self, x, y):
        ground_y_px = self.HEIGHT - 90

        sx = int((x - self.camera_x) * self.PIXELS_PER_METER + self.WIDTH * 0.28)
        sy = int(ground_y_px - y * self.PIXELS_PER_METER)

        return sx, sy

    def update_camera(self):
        if self.ball_body is None:
            return

        self.camera_x = max(0.0, self.ball_body.position.x - 2.0)

    def draw(self):
        self.update_camera()

        self.screen.fill((245, 245, 245))

        self.draw_lane()
        self.draw_ball()
        self.draw_hud()

    def draw_lane(self):
        ground_y_px = self.HEIGHT - 90

        pygame.draw.line(
            self.screen,
            (60, 60, 60),
            (0, ground_y_px),
            (self.WIDTH, ground_y_px),
            4,
        )

        # Marcas de metros
        start_m = int(self.camera_x)
        end_m = start_m + int(self.WIDTH / self.PIXELS_PER_METER) + 5

        for m in range(start_m, end_m):
            sx, sy = self.world_to_screen(m, 0.0)

            pygame.draw.line(
                self.screen,
                (160, 160, 160),
                (sx, sy),
                (sx, sy + 12),
                1,
            )

            if m % 5 == 0:
                font = pygame.font.SysFont("Arial", 14)
                txt = font.render(f"{m} m", True, (90, 90, 90))
                self.screen.blit(txt, (sx - 12, sy + 16))

    def draw_ball(self):
        if self.ball_body is None:
            return

        x = self.ball_body.position.x
        y = self.params.radius

        sx, sy = self.world_to_screen(x, y)
        r_px = max(8, int(self.params.radius * self.PIXELS_PER_METER))

        # Bola base
        pygame.draw.circle(self.screen, (240, 240, 240), (sx, sy), r_px)
        pygame.draw.circle(self.screen, (30, 30, 30), (sx, sy), r_px, 2)

        # Rayas tipo pelota de playa
        colors = [
            (235, 70, 70),
            (255, 210, 60),
            (70, 170, 255),
            (70, 210, 140),
            (255, 140, 60),
            (255, 105, 180),
        ]

        angle = self.ball_body.angle

        for i, color in enumerate(colors):
            a0 = angle + i * math.tau / len(colors)
            a1 = angle + (i + 1) * math.tau / len(colors)

            p0 = (sx, sy)
            p1 = (
                sx + int(math.cos(a0) * r_px),
                sy - int(math.sin(a0) * r_px),
            )
            p2 = (
                sx + int(math.cos(a1) * r_px),
                sy - int(math.sin(a1) * r_px),
            )

            pygame.draw.polygon(self.screen, color, [p0, p1, p2])

        pygame.draw.circle(self.screen, (30, 30, 30), (sx, sy), r_px, 2)

        # Marcador de rotación
        marker_angle = -self.ball_body.angle
        mx = sx + int(math.cos(marker_angle) * r_px * 0.65)
        my = sy + int(math.sin(marker_angle) * r_px * 0.65)

        pygame.draw.circle(self.screen, (255, 255, 0), (mx, my), max(4, r_px // 10))

    def draw_hud(self):
        if self.ball_body is None:
            return

        font = pygame.font.SysFont("Consolas", 18)

        vx = self.ball_body.velocity.x
        omega = -self.ball_body.angular_velocity
        slip = vx - self.params.radius * omega

        lines = [
            f"t = {self.sim_time:.3f} s",
            f"x = {self.ball_body.position.x:.3f} m",
            f"v = {vx:.3f} m/s",
            f"ω = {omega:.3f} rad/s",
            f"slip = {slip:.4f} m/s",
            f"tipo = {self.params.body_type_label}",
        ]

        x0, y0 = 20, 20

        for i, line in enumerate(lines):
            txt = font.render(line, True, (20, 20, 20))
            self.screen.blit(txt, (x0, y0 + i * 24))


# ============================================================
# INTERFAZ TKINTER
# ============================================================
class BowlingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pymunk · Deslizamiento y rodadura de bola de bolos")
        self.root.geometry("920x900")

        self.ui_queue = queue.Queue()
        self.params_vars = {}
        self.hollow_var = tk.BooleanVar(value=False)

        self.theory_labels = {}
        self.telemetry_labels = {}

        self.sim = None

        self.canvas = None
        self.v_scrollbar = None
        self.scrollable_frame = None
        self.canvas_window = None

        self.build_ui()
        self.start_simulation()

        self.root.after(30, self.process_ui_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def default_params(self):
        return Parameters()

    def build_ui(self):
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(outer, highlightthickness=0)
        self.v_scrollbar = ttk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set)

        self.v_scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.scrollable_frame = ttk.Frame(self.canvas, padding=12)

        self.canvas_window = self.canvas.create_window(
            (0, 0),
            window=self.scrollable_frame,
            anchor="nw",
        )

        def on_frame_configure(_event):
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        self.scrollable_frame.bind("<Configure>", on_frame_configure)

        def on_canvas_configure(event):
            self.canvas.itemconfig(self.canvas_window, width=event.width)

        self.canvas.bind("<Configure>", on_canvas_configure)

        def _on_mousewheel(event):
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _bind_mousewheel(_event):
            self.canvas.bind_all("<MouseWheel>", _on_mousewheel)

        def _unbind_mousewheel(_event):
            self.canvas.unbind_all("<MouseWheel>")

        self.canvas.bind("<Enter>", _bind_mousewheel)
        self.canvas.bind("<Leave>", _unbind_mousewheel)

        main = self.scrollable_frame

        ttk.Label(
            main,
            text="Estudio de deslizamiento y rodadura · bola de bolos · Pymunk",
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 10))

        params_frame = ttk.LabelFrame(main, text="Parámetros personalizables", padding=10)
        params_frame.pack(fill="x", pady=(0, 10))

        fields = [
            ("mass", "Masa (kg)"),
            ("radius", "Radio (m)"),
            ("mu_contact_pymunk", "Rozamiento de contacto Pymunk"),
            ("mu_rolling", "Rozamiento de rodadura μ_r"),
            ("v0", "Velocidad lineal inicial v₀ (m/s)"),
            ("omega0", "Velocidad angular inicial ω₀ (rad/s)"),
            ("lane_length", "Longitud de pista (m)"),
            ("launch_height", "Altura de salida (m)"),
            ("time_scale", "Escala temporal"),
        ]

        defaults = asdict(self.default_params())

        for i, (key, label) in enumerate(fields):
            row = i // 2
            col = (i % 2) * 2

            ttk.Label(params_frame, text=label).grid(
                row=row,
                column=col,
                sticky="w",
                padx=(0, 8),
                pady=4,
            )

            var = tk.StringVar(value=str(defaults[key]))

            entry = ttk.Entry(params_frame, textvariable=var, width=18)
            entry.grid(row=row, column=col + 1, sticky="ew", pady=4)

            self.params_vars[key] = var

        ttk.Checkbutton(
            params_frame,
            text="Bola hueca (cascarón fino)",
            variable=self.hollow_var,
            command=lambda: self.apply_parameters(update_sim=False),
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(6, 2))

        for c in range(4):
            params_frame.columnconfigure(c, weight=1)

        buttons = ttk.Frame(main)
        buttons.pack(fill="x", pady=(0, 10))

        ttk.Button(buttons, text="Aplicar parámetros", command=self.apply_parameters).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Lanzar bola", command=self.launch_ball).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Detener bola", command=self.stop_ball).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Reset", command=self.reset_simulation).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Valores por defecto", command=self.load_defaults).pack(side="left")

        theory_frame = ttk.LabelFrame(main, text="Cálculo teórico previo / equivalente", padding=10)
        theory_frame.pack(fill="x", pady=(0, 10))

        theory_keys = [
            ("body_type", "Tipo de bola"),
            ("inertia_factor", "Factor de inercia k en I = k mR²"),
            ("mu_used", "μ usado en teoría inicial"),
            ("regime", "Régimen inicial"),
            ("initial_slip", "Slip inicial v₀ - R·ω₀ (m/s)"),
            ("slip_decay_rate", "Rapidez teórica de reducción de slip (m/s²)"),
            ("t_to_pure_rolling", "Tiempo teórico hasta rodadura pura (s)"),
            ("x_to_pure_rolling", "Distancia teórica hasta rodadura pura (m)"),
            ("v_at_pure_rolling", "Velocidad teórica al empezar a rodar (m/s)"),
            ("omega_at_pure_rolling", "ω teórica al empezar a rodar (rad/s)"),
        ]

        for i, (key, label) in enumerate(theory_keys):
            ttk.Label(theory_frame, text=label).grid(row=i, column=0, sticky="w", pady=3)
            value = ttk.Label(theory_frame, text="—")
            value.grid(row=i, column=1, sticky="w", pady=3, padx=(10, 0))
            self.theory_labels[key] = value

        telemetry_frame = ttk.LabelFrame(main, text="Medidas en simulación", padding=10)
        telemetry_frame.pack(fill="both", expand=True)

        telemetry_keys = [
            ("t", "Tiempo actual (s)"),
            ("x", "Posición x (m)"),
            ("vx", "Velocidad lineal (m/s)"),
            ("omega", "Velocidad angular (rad/s)"),
            ("slip", "Slip instantáneo v - R·ω (m/s)"),
            ("regime", "Régimen actual"),
            ("rolling_start_time", "Instante medido de rodadura pura (s)"),
            ("rolling_start_speed", "Velocidad medida al empezar a rodar (m/s)"),
            ("rolling_start_omega", "ω medida al empezar a rodar (rad/s)"),
            ("mu_equiv_theory", "μ equivalente medido en simulación"),
            ("error_t_percent", "Error relativo en t* (%)"),
            ("error_v_percent", "Error relativo en v* (%)"),
            ("error_w_percent", "Error relativo en ω* (%)"),
        ]

        for i, (key, label) in enumerate(telemetry_keys):
            ttk.Label(telemetry_frame, text=label).grid(row=i, column=0, sticky="w", pady=3)
            value = ttk.Label(telemetry_frame, text="—")
            value.grid(row=i, column=1, sticky="w", pady=3, padx=(10, 0))
            self.telemetry_labels[key] = value

        formula = (
            "Modelo usado en esta versión Pymunk:\n"
            "• Pymunk se usa como motor 2D para representar la bola y su rotación\n"
            "• La fricción de deslizamiento se aplica manualmente para mantener el modelo físico claro\n"
            "• La fórmula usada es slip = v - Rω\n"
            "• En Pymunk la velocidad angular positiva es antihoraria, por eso internamente se usa -angular_velocity\n"
            "• Rodadura pura: |v - Rω| / max(|v|, |Rω|) <= tolerancia relativa"
        )

        ttk.Label(main, text=formula, justify="left").pack(anchor="w", pady=(10, 0))

        self.load_defaults()

    def parse_params(self) -> Parameters:
        try:
            params = Parameters(
                mass=float(self.params_vars["mass"].get()),
                radius=float(self.params_vars["radius"].get()),
                hollow=bool(self.hollow_var.get()),
                mu_contact_pymunk=float(self.params_vars["mu_contact_pymunk"].get()),
                mu_rolling=float(self.params_vars["mu_rolling"].get()),
                v0=float(self.params_vars["v0"].get()),
                omega0=float(self.params_vars["omega0"].get()),
                lane_length=float(self.params_vars["lane_length"].get()),
                launch_height=float(self.params_vars["launch_height"].get()),
                time_scale=float(self.params_vars["time_scale"].get()),
            )
        except ValueError:
            raise ValueError("Todos los parámetros deben ser numéricos.")

        if params.mass <= 0:
            raise ValueError("La masa debe ser positiva.")

        if params.radius <= 0:
            raise ValueError("El radio debe ser positivo.")

        if params.lane_length <= 0:
            raise ValueError("La longitud de pista debe ser positiva.")

        if params.launch_height <= 0:
            raise ValueError("La altura debe ser positiva.")

        if params.mu_contact_pymunk < 0 or params.mu_rolling < 0:
            raise ValueError("Los coeficientes de rozamiento no pueden ser negativos.")

        if params.time_scale <= 0:
            raise ValueError("La escala temporal debe ser mayor que 0.")

        return params

    def update_theory(self, theory: TheoryResult):
        def fnum(x):
            if x is None:
                return "—"
            if isinstance(x, float):
                if math.isinf(x):
                    return "∞"
                if math.isnan(x):
                    return "NaN"
                return f"{x:.4f}"
            return str(x)

        self.theory_labels["body_type"].config(text=fnum(theory.body_type))
        self.theory_labels["inertia_factor"].config(text=fnum(theory.inertia_factor))
        self.theory_labels["mu_used"].config(text=fnum(theory.mu_used))
        self.theory_labels["regime"].config(text=fnum(theory.regime))
        self.theory_labels["initial_slip"].config(text=fnum(theory.initial_slip))
        self.theory_labels["slip_decay_rate"].config(text=fnum(theory.slip_decay_rate))
        self.theory_labels["t_to_pure_rolling"].config(text=fnum(theory.t_to_pure_rolling))
        self.theory_labels["x_to_pure_rolling"].config(text=fnum(theory.x_to_pure_rolling))
        self.theory_labels["v_at_pure_rolling"].config(text=fnum(theory.v_at_pure_rolling))
        self.theory_labels["omega_at_pure_rolling"].config(text=fnum(theory.omega_at_pure_rolling))

    def load_defaults(self):
        defaults = asdict(self.default_params())

        for key, value in defaults.items():
            if key == "hollow":
                self.hollow_var.set(bool(value))
            else:
                self.params_vars[key].set(str(value))

        self.apply_parameters(update_sim=False)

    def apply_parameters(self, update_sim=True):
        try:
            params = self.parse_params()
        except ValueError as e:
            messagebox.showerror("Parámetros no válidos", str(e))
            return

        self.update_theory(compute_theory(params))

        if update_sim and self.sim is not None:
            self.sim.post_command("update_params", params)

    def launch_ball(self):
        try:
            params = self.parse_params()
        except ValueError as e:
            messagebox.showerror("Parámetros no válidos", str(e))
            return

        self.update_theory(compute_theory(params))

        if self.sim is not None:
            self.sim.post_command("update_params", params)
            self.sim.post_command("launch", None)

    def stop_ball(self):
        if self.sim is not None:
            self.sim.post_command("stop_motion", None)

    def reset_simulation(self):
        try:
            params = self.parse_params()
        except ValueError as e:
            messagebox.showerror("Parámetros no válidos", str(e))
            return

        self.update_theory(compute_theory(params))

        if self.sim is not None:
            self.sim.post_command("reset", params)

    def start_simulation(self):
        params = self.parse_params()
        self.update_theory(compute_theory(params))

        self.sim = BowlingSimulation(params, self.ui_queue.put)
        self.sim.start()

    def process_ui_queue(self):
        try:
            while True:
                kind, data = self.ui_queue.get_nowait()

                if kind == "telemetry":
                    self.refresh_telemetry(data)

                elif kind == "reset_done":
                    for key, label in self.telemetry_labels.items():
                        label.config(text="—")
                    self.telemetry_labels["regime"].config(text="En espera")

                elif kind == "launched":
                    self.telemetry_labels["regime"].config(text="Deslizamiento")

                elif kind == "stopped":
                    self.telemetry_labels["regime"].config(text="Pausada")

        except queue.Empty:
            pass

        self.root.after(30, self.process_ui_queue)

    def refresh_telemetry(self, data):
        def f(key):
            value = data.get(key)

            if value is None:
                return "—"

            if isinstance(value, float):
                if math.isinf(value):
                    return "∞"
                if math.isnan(value):
                    return "NaN"
                return f"{value:.4f}"

            return str(value)

        for key in self.telemetry_labels:
            self.telemetry_labels[key].config(text=f(key))

    def on_close(self):
        if self.sim is not None:
            self.sim.stop()

        self.root.destroy()


# ============================================================
# MAIN
# ============================================================
def main():
    root = tk.Tk()

    style = ttk.Style(root)

    try:
        style.theme_use("clam")
    except Exception:
        pass

    BowlingApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()