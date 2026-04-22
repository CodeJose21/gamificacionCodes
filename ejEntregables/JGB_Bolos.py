import math
import os
import queue
import tempfile
import threading
import time
from dataclasses import dataclass, asdict

from PIL import Image, ImageDraw
import pybullet as p
import pybullet_data
import tkinter as tk
from tkinter import ttk, messagebox


# ============================================================
# PROYECTO: DESLIZAMIENTO Y RODADURA DE UNA BOLA DE BOLOS
# ============================================================
# AJUSTE CONCEPTUAL IMPORTANTE
# ----------------------------
# En esta versión se separan dos conceptos que antes se mezclaban:
#
# 1) mu_contact_pybullet
#    Es el coeficiente que usa PyBullet en el contacto suelo-bola.
#
# 2) mu_equiv_theory
#    Es el coeficiente equivalente reconstruido a partir de la simulación,
#    para que la teoría refleje el comportamiento efectivo observado.
#
# Esto evita comparar directamente dos magnitudes que no significan
# exactamente lo mismo.
# ============================================================

G = 9.81
SIM_DT = 1.0 / 1000.0
STOP_SPEED = 0.01

# Criterio relativo para considerar rodadura pura:
# slip / velocidad característica <= PURE_ROLLING_REL_TOL
PURE_ROLLING_REL_TOL = 0.01


# ============================================================
# PARÁMETROS DEL PROBLEMA
# ============================================================
@dataclass
class Parameters:
    # Geometría y masa
    mass: float = 7.0
    radius: float = 1.0
    hollow: bool = False   # False -> esfera maciza, True -> cascarón fino

    # Rozamientos
    mu_contact_pybullet: float = 0.22
    mu_rolling: float = 0.0

    # Condiciones iniciales
    v0: float = 8.0
    omega0: float = 0.0

    # Entorno / simulación
    lane_length: float = 2500.0
    launch_height: float = 1.0001
    time_scale: float = 1.0

    @property
    def inertia_factor(self) -> float:
        """
        Factor k en I = k m R².

        - esfera maciza: k = 2/5
        - esfera hueca fina: k = 2/3
        """
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
    """
    Cálculo teórico idealizado del tramo de deslizamiento.

    Si mu_used es None, usamos mu_contact_pybullet como referencia inicial.
    Si mu_used tiene valor, usamos ese coeficiente equivalente efectivo.
    """
    mu = params.mu_contact_pybullet if mu_used is None else mu_used
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
        regime="Empieza deslizando" if s0 > 0 else "Sale con exceso de spin (over-spin)",
        slip_decay_rate=slip_decay_rate,
    )


# ============================================================
# HILO DE SIMULACIÓN PYBULLET
# ============================================================
class BowlingSimulation(threading.Thread):
    def __init__(self, params: Parameters, ui_callback):
        super().__init__(daemon=True)
        self.params = params
        self.ui_callback = ui_callback

        self._commands = queue.Queue()
        self._stop_event = threading.Event()

        self.sim_time = 0.0
        self.last_ui_send_time = 0.0

        # IDs PyBullet
        self.client = None
        self.plane_id = None
        self.ball_id = None
        self.beach_ball_texture_id = None

        # Estado dinámico
        self.running = False
        self.ball_launched = False
        self.ball_has_reached_rolling = False

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        # Valores teóricos de referencia inicial
        self.theory_initial = None

        # Mu equivalente reconstruido a partir de la simulación
        self.initial_slip = None
        self.mu_equiv_theory = None
        self.theory_equiv = None

        # Errores teoría equivalente vs simulación
        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

    def post_command(self, cmd, payload=None):
        self._commands.put((cmd, payload))

    def stop(self):
        self._stop_event.set()
        self.post_command("quit")

    def send_ui(self, kind, data):
        self.ui_callback((kind, data))

    def create_beach_ball_texture(self):
        """
        Crea una textura procedural tipo pelota de playa.
        """
        width, height = 1024, 512
        img = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(img)

        colors = [
            (235, 70, 70),
            (255, 210, 60),
            (70, 170, 255),
            (70, 210, 140),
            (255, 140, 60),
            (255, 105, 180),
        ]

        n = len(colors)
        stripe_w = width / n
        for i, color in enumerate(colors):
            x0 = int(i * stripe_w)
            x1 = int((i + 1) * stripe_w)
            draw.rectangle([x0, 0, x1, height], fill=color)

        pole_radius = 55
        cx = width // 2
        draw.ellipse(
            [cx - pole_radius, 20 - pole_radius, cx + pole_radius, 20 + pole_radius],
            fill=(245, 245, 245),
        )
        draw.ellipse(
            [cx - pole_radius, height - 20 - pole_radius, cx + pole_radius, height - 20 + pole_radius],
            fill=(245, 245, 245),
        )

        tmp_dir = tempfile.gettempdir()
        texture_path = os.path.join(tmp_dir, "beach_ball_texture.png")
        img.save(texture_path)

        return p.loadTexture(texture_path, physicsClientId=self.client)

    def reset_world(self):
        if self.client is None:
            return

        self.sim_time = 0.0
        self.last_ui_send_time = 0.0

        p.resetSimulation(physicsClientId=self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -G, physicsClientId=self.client)
        p.setTimeStep(SIM_DT, physicsClientId=self.client)
        p.setPhysicsEngineParameter(
            fixedTimeStep=SIM_DT,
            numSolverIterations=1000,
            physicsClientId=self.client,
        )

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)

        # Suelo
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        p.changeDynamics(
            self.plane_id,
            -1,
            lateralFriction=max(self.params.mu_contact_pybullet, 1e-6),
            rollingFriction=max(self.params.mu_rolling, 0.0),
            spinningFriction=0.0,
            linearDamping=0.0,
            angularDamping=0.0,
            restitution=0.0,
            physicsClientId=self.client,
        )

        # Bola
        collision = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=self.params.radius,
            physicsClientId=self.client,
        )
        visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.params.radius,
            rgbaColor=[1.0, 1.0, 1.0, 1.0],
            physicsClientId=self.client,
        )

        self.ball_id = p.createMultiBody(
            baseMass=self.params.mass,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[0.0, 0.0, self.params.launch_height],
            physicsClientId=self.client,
        )

        self.beach_ball_texture_id = self.create_beach_ball_texture()
        p.changeVisualShape(
            self.ball_id,
            -1,
            textureUniqueId=self.beach_ball_texture_id,
            physicsClientId=self.client,
        )

        p.changeDynamics(
            self.ball_id,
            -1,
            lateralFriction=max(self.params.mu_contact_pybullet, 1e-6),
            rollingFriction=max(self.params.mu_rolling, 0.0),
            spinningFriction=0.0,
            linearDamping=0.0,
            angularDamping=0.0,
            restitution=0.0,
            localInertiaDiagonal=[self.params.inertia] * 3,
            physicsClientId=self.client,
        )

        p.resetBaseVelocity(
            self.ball_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client,
        )

        self.running = False
        self.ball_launched = False
        self.ball_has_reached_rolling = False

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        self.theory_initial = None
        self.initial_slip = None
        self.mu_equiv_theory = None
        self.theory_equiv = None

        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

        p.resetDebugVisualizerCamera(
            cameraDistance=4.8,
            cameraYaw=0,
            cameraPitch=-18,
            cameraTargetPosition=[2.0, 0.0, 0.0],
            physicsClientId=self.client,
        )

        self.send_ui("reset_done", {})

    def launch_ball(self):
        if self.client is None or self.ball_id is None:
            return

        self.theory_initial = compute_theory(self.params)
        self.initial_slip = self.params.v0 - self.params.radius * self.params.omega0
        self.mu_equiv_theory = None
        self.theory_equiv = None

        self.sim_time = 0.0

        p.resetBaseVelocity(
            self.ball_id,
            linearVelocity=[self.params.v0, 0.0, 0.0],
            angularVelocity=[0.0, self.params.omega0, 0.0],
            physicsClientId=self.client,
        )

        self.running = True
        self.ball_launched = True
        self.ball_has_reached_rolling = False

        self.rolling_start_time = None
        self.rolling_start_speed = None
        self.rolling_start_omega = None

        self.error_t_percent = None
        self.error_v_percent = None
        self.error_w_percent = None

        self.send_ui("launched", {})

    def stop_ball_motion(self):
        if self.client is None or self.ball_id is None:
            return

        p.resetBaseVelocity(
            self.ball_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client,
        )
        self.running = False
        self.ball_launched = False
        self.send_ui("stopped", {})

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

    def run(self):
        self.client = p.connect(p.GUI)
        self.reset_world()

        while not self._stop_event.is_set():
            if not self._handle_commands():
                break

            if self.ball_id is not None:
                self.step_simulation()

            p.stepSimulation(physicsClientId=self.client)

            if self.running and self.ball_launched:
                self.sim_time += SIM_DT

            time.sleep(SIM_DT / max(self.params.time_scale, 1e-6))

        if self.client is not None:
            p.disconnect(self.client)

    def step_simulation(self):
        pos, _orn = p.getBasePositionAndOrientation(self.ball_id, physicsClientId=self.client)
        lin_vel, ang_vel = p.getBaseVelocity(self.ball_id, physicsClientId=self.client)

        vx = lin_vel[0]
        omega = ang_vel[1]
        slip = vx - self.params.radius * omega
        elapsed = self.sim_time

        speed_ref = max(abs(vx), abs(self.params.radius * omega), 1e-6)
        pure_rolling = abs(slip) / speed_ref <= PURE_ROLLING_REL_TOL

        if not self.ball_launched:
            regime = "En espera"
        else:
            speed_abs = abs(vx)
            if speed_abs <= STOP_SPEED and abs(omega) <= STOP_SPEED / max(self.params.radius, 1e-9):
                regime = "Parada"
            elif not pure_rolling:
                regime = "Deslizamiento"
            else:
                regime = "Rodadura pura"

        if self.running and self.ball_launched and (not self.ball_has_reached_rolling):
            if pure_rolling:
                self.ball_has_reached_rolling = True
                self.rolling_start_time = elapsed
                self.rolling_start_speed = vx
                self.rolling_start_omega = omega

                k = self.params.inertia_factor
                s0 = self.initial_slip

                if s0 is not None and abs(s0) > 1e-12 and self.rolling_start_time > 1e-12:
                    self.mu_equiv_theory = abs(s0) / (
                        G * (1.0 + 1.0 / k) * self.rolling_start_time
                    )
                    self.theory_equiv = compute_theory(self.params, mu_used=self.mu_equiv_theory)

                if self.theory_equiv is not None:
                    if math.isfinite(self.theory_equiv.t_to_pure_rolling) and self.theory_equiv.t_to_pure_rolling > 1e-12:
                        self.error_t_percent = abs(
                            self.rolling_start_time - self.theory_equiv.t_to_pure_rolling
                        ) / self.theory_equiv.t_to_pure_rolling * 100.0

                    if math.isfinite(self.theory_equiv.v_at_pure_rolling) and abs(self.theory_equiv.v_at_pure_rolling) > 1e-12:
                        self.error_v_percent = abs(
                            self.rolling_start_speed - self.theory_equiv.v_at_pure_rolling
                        ) / abs(self.theory_equiv.v_at_pure_rolling) * 100.0

                    if math.isfinite(self.theory_equiv.omega_at_pure_rolling) and abs(self.theory_equiv.omega_at_pure_rolling) > 1e-12:
                        self.error_w_percent = abs(
                            self.rolling_start_omega - self.theory_equiv.omega_at_pure_rolling
                        ) / abs(self.theory_equiv.omega_at_pure_rolling) * 100.0

        self.update_camera(pos)

        now = time.perf_counter()
        if now - self.last_ui_send_time >= 1.0 / 30.0:
            self.send_ui(
                "telemetry",
                {
                    "t": elapsed,
                    "x": pos[0],
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

    def update_camera(self, pos):
        if self.client is None:
            return

        target_x = max(pos[0] + 1.8, 1.8)
        p.resetDebugVisualizerCamera(
            cameraDistance=4.8,
            cameraYaw=0,
            cameraPitch=-18,
            cameraTargetPosition=[target_x, pos[1], 0.0],
            physicsClientId=self.client,
        )


# ============================================================
# INTERFAZ TKINTER
# ============================================================
class BowlingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PyBullet · Deslizamiento y rodadura de bola de bolos")
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
            anchor="nw"
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
            text="Estudio de deslizamiento y rodadura · bola de bolos",
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 10))

        params_frame = ttk.LabelFrame(main, text="Parámetros personalizables", padding=10)
        params_frame.pack(fill="x", pady=(0, 10))

        fields = [
            ("mass", "Masa (kg)"),
            ("radius", "Radio (m)"),
            ("mu_contact_pybullet", "Rozamiento de contacto PyBullet"),
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

            ttk.Label(params_frame, text=label).grid(row=row, column=col, sticky="w", padx=(0, 8), pady=4)
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
            "Modelo usado en esta versión:\n"
            "• PyBullet usa 'mu_contact_pybullet' como rozamiento de contacto del motor\n"
            "• La teoría inicial usa ese valor como referencia\n"
            "• Al medir la transición a rodadura pura, se reconstruye 'μ equivalente'\n"
            "• Los errores se comparan contra la teoría equivalente, no contra el μ nominal\n"
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
                mu_contact_pybullet=float(self.params_vars["mu_contact_pybullet"].get()),
                mu_rolling=float(self.params_vars["mu_rolling"].get()),
                v0=float(self.params_vars["v0"].get()),
                omega0=float(self.params_vars["omega0"].get()),
                lane_length=float(self.params_vars["lane_length"].get()),
                launch_height=float(self.params_vars["launch_height"].get()),
                time_scale=float(self.params_vars["time_scale"].get()),
            )
        except ValueError:
            raise ValueError("Todos los parámetros deben ser numéricos.")

        if params.mass <= 0 or params.radius <= 0 or params.lane_length <= 0 or params.launch_height <= 0:
            raise ValueError("Masa, radio, longitud de pista y altura deben ser positivos.")
        if params.mu_contact_pybullet < 0 or params.mu_rolling < 0:
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