# curling_turnos_cenital_winner_fixed.py
#
# Mini-juego de curling en PyBullet (vista cenital) con:
# - 2 equipos (ROJO y AMARILLO), 2 piedras por equipo (4 tiros en total), alternando turnos
# - La cámara sigue a la piedra desde arriba
# - HUD que se mueve con la cámara (se posiciona relativo al objetivo de cámara) en la esquina "arriba-derecha"
# - Indicadores: marcador, turno/equipo, distancia al centro de la diana, fuerza
# - No se spawnea la siguiente piedra hasta que la anterior se para
# - Al terminar la última piedra, se calcula puntuación y se anuncia el ganador
#
# Nota:
# addUserDebugText/addUserDebugLine puede saturarse si se llama a 240 FPS.
# Por eso se actualiza el HUD cada N frames (throttle).

import math
import time
import pybullet as p
import pybullet_data


# ----------------- Helpers -----------------
def clamp(x, a, b):
    return max(a, min(b, x))


def dist2(a_xy, b_xy):
    return math.hypot(a_xy[0] - b_xy[0], a_xy[1] - b_xy[1])


def draw_house(center_xy, z=0.001):
    """
    Dibuja la diana (house) con líneas de depuración en el suelo.
    Esto es solo visual; no afecta a la física.
    """
    cx, cy = center_xy
    rings = [
        (0.50, (1, 0, 0)),
        (1.0, (0, 0, 1)),
        (1.50, (1, 1, 1)),
    ]
    segments = 96
    for r, col in rings:
        for i in range(segments):
            a0 = 2 * math.pi * (i / segments)
            a1 = 2 * math.pi * ((i + 1) / segments)
            x0, y0 = cx + r * math.cos(a0), cy + r * math.sin(a0)
            x1, y1 = cx + r * math.cos(a1), cy + r * math.sin(a1)
            p.addUserDebugLine([x0, y0, z], [x1, y1, z], col, 2)

    p.addUserDebugLine([cx - 0.05, cy, z], [cx + 0.05, cy, z], [0, 0, 0], 3)
    p.addUserDebugLine([cx, cy - 0.05, z], [cx, cy + 0.05, z], [0, 0, 0], 3)


def compute_curling_score(red_ds, yel_ds, house_radius):
    """
    Reglas básicas para un "end" único:
    - Solo cuentan piedras dentro del radio de la casa.
    - Puntúa el equipo con la piedra más cercana al centro.
    - Puntos = número de piedras del equipo ganador que están más cerca que la mejor del rival.
    """
    red_in = sorted([d for d in red_ds if d <= house_radius])
    yel_in = sorted([d for d in yel_ds if d <= house_radius])

    if not red_in and not yel_in:
        return 0, 0, "Sin piedras en la casa"

    if red_in and not yel_in:
        return len(red_in), 0, "Puntúa ROJO (rival fuera)"
    if yel_in and not red_in:
        return 0, len(yel_in), "Puntúa AMARILLO (rival fuera)"

    if red_in[0] < yel_in[0]:
        cutoff = yel_in[0]
        pts = sum(1 for d in red_in if d < cutoff)
        return pts, 0, f"Puntúa ROJO ({pts})"

    if yel_in[0] < red_in[0]:
        cutoff = red_in[0]
        pts = sum(1 for d in yel_in if d < cutoff)
        return 0, pts, f"Puntúa AMARILLO ({pts})"

    return 0, 0, "Empate exacto (0)"


def setup_world():
    """
    Devuelve: (lane_length, lane_half_width, house_center)
    La diana queda al final del pasillo (lo más cerca posible del back wall).
    """
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)

    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(
        plane_id, -1,
        lateralFriction=0.02,
        spinningFriction=0.001,
        rollingFriction=0.0008,
        restitution=0.0
    )

    lane_half_width = 2
    lane_length = 20.0

    wall_h = 0.18
    wall_th = 0.05

    # Paredes laterales sin huecos
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[lane_length / 2, wall_th / 2, wall_h / 2])
    wall_vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[lane_length / 2, wall_th / 2, wall_h / 2],
        rgbaColor=[0.15, 0.15, 0.15, 1]
    )

    x_center = lane_length / 2
    y_left = +(lane_half_width + wall_th / 2)
    y_right = -(lane_half_width + wall_th / 2)

    wallL_id = p.createMultiBody(0, wall_col, wall_vis, [x_center, y_left, wall_h / 2])
    wallR_id = p.createMultiBody(0, wall_col, wall_vis, [x_center, y_right, wall_h / 2])

    # Back wall cerrando el final (x = lane_length)
    back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_th / 2, lane_half_width + wall_th, wall_h / 2])
    back_vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[wall_th / 2, lane_half_width + wall_th, wall_h / 2],
        rgbaColor=[0.15, 0.15, 0.15, 1]
    )
    back_id = p.createMultiBody(0, back_col, back_vis, [lane_length + wall_th / 2, 0, wall_h / 2])

    # Rebote en paredes (arcade)
    p.changeDynamics(wallL_id, -1, restitution=1, lateralFriction=0.02)
    p.changeDynamics(wallR_id, -1, restitution=1, lateralFriction=0.02)
    p.changeDynamics(back_id,  -1, restitution=1, lateralFriction=0.02)

    # Líneas visuales del pasillo
    p.addUserDebugLine([0, -lane_half_width, 0.002], [lane_length, -lane_half_width, 0.002], [0.7, 0.7, 0.7], 2)
    p.addUserDebugLine([0, +lane_half_width, 0.002], [lane_length, +lane_half_width, 0.002], [0.7, 0.7, 0.7], 2)
    p.addUserDebugLine([lane_length, -lane_half_width, 0.002], [lane_length, +lane_half_width, 0.002], [0.7, 0.7, 0.7], 2)

    # Diana al final del pasillo (centrada en Y)
    HOUSE_R = 0.90
    EDGE_MARGIN = 2  # margen pequeño para no tocar la línea final
    house_x = lane_length - HOUSE_R - EDGE_MARGIN
    house_center = (house_x, 0.0)

    return lane_length, lane_half_width, house_center


def spawn_stone(color_rgba, start_pos=(1.0, 0.0, 0.08)):
    """
    Crea una "piedra" simplificada como cilindro.
    """
    radius = 0.14
    height = 0.10
    mass = 20.0

    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color_rgba)

    sid = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=list(start_pos),
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    )

    # Física de la piedra:
    # - lateralFriction: controla deslizamiento.
    # - rollingFriction/spinningFriction: ayudan a frenar rodadura/giro.
    # - restitution: rebote en choques (piedra-piedra/piedra-pared).
    # - damping: amortiguación para evitar vibraciones infinitas con dt discreto.
    p.changeDynamics(
        sid, -1,
        lateralFriction=0.2,
        rollingFriction=0.1,
        spinningFriction=0.1,
        restitution=0.02,
        linearDamping=0.2,
        angularDamping=0.15,
    )
    return sid


# ----------------- Debug/HUD utilities (compatibles) -----------------
def set_topdown_camera(target_pos, distance=3.2, yaw=0, pitch=-89.9):
    """
    Coloca la cámara cenital sobre target_pos.
    """
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=[target_pos[0], target_pos[1], 0.0]
    )


def safe_rm_user_debug(item_id):
    """
    Elimina un debug item si existe.
    """
    if item_id is None:
        return
    try:
        p.removeUserDebugItem(item_id)
    except Exception:
        pass


def rm(hud, key):
    """
    Elimina un elemento del HUD almacenado en hud[key].
    """
    safe_rm_user_debug(hud.get(key))
    hud[key] = None


def draw_hud_follow_camera(
    hud,
    cam_target_pos,
    score_text,
    info_text,
    status_text,
    ofs_x,
    ofs_y,
    z0,
    line
):
    """
    Textos persistentes (lifeTime por defecto infinito) actualizados con replaceItemUniqueId.
    Esto evita parpadeo porque no se crean objetos nuevos cada vez.
    """
    def repl(old_id):
        return old_id if isinstance(old_id, int) and old_id >= 0 else -1

    cx, cy, _ = cam_target_pos
    x = cx + ofs_x
    y = cy + ofs_y

    pos_score  = [x, y, z0]
    pos_info   = [x, y, z0 - line]
    pos_status = [x, y, z0 - 2 * line]

    hud["score"] = p.addUserDebugText(
        score_text, pos_score,
        textColorRGB=[0, 0, 0],
        textSize=1.30,
        replaceItemUniqueId=repl(hud.get("score", -1))
    )

    hud["info"] = p.addUserDebugText(
        info_text, pos_info,
        textColorRGB=[0, 0, 0],
        textSize=1.05,
        replaceItemUniqueId=repl(hud.get("info", -1))
    )

    hud["status"] = p.addUserDebugText(
        status_text, pos_status,
        textColorRGB=[0, 0, 0],
        textSize=1.00,
        replaceItemUniqueId=repl(hud.get("status", -1))
    )


def draw_power_bar(
    hud,
    power_value,
    cam_target_pos,
    ofs_x,
    ofs_y,
    z0,
    line,
    bar_gap,
    bar_len,
    pmin=2.0,
    pmax=50.0
):
    """
    Barra persistente:
    - lifeTime=0 (no desaparece)
    - solo se borran y recrean 2 líneas (base y fill), sin tocar el texto
    - colocada SIEMPRE debajo de la línea de status para evitar solapes
    """
    t = clamp((power_value - pmin) / (pmax - pmin), 0.0, 1.0)

    cx, cy, _ = cam_target_pos
    x0 = cx + ofs_x
    y0 = cy + ofs_y

    # Debajo del status (z0 - 2*line) y un poco más abajo
    z_bar = (z0 - 2 * line) - bar_gap

    # Quitamos solo las 2 líneas anteriores
    safe_rm_user_debug(hud.get("power_base"))
    safe_rm_user_debug(hud.get("power_fill"))
    hud["power_base"] = None
    hud["power_fill"] = None

    x1 = x0 + bar_len
    xf = x0 + bar_len * t

    hud["power_base"] = p.addUserDebugLine(
        [x0, y0, z_bar], [x1, y0, z_bar],
        [0.2, 0.2, 0.2],
        lineWidth=10,
        lifeTime=0
    )

    hud["power_fill"] = p.addUserDebugLine(
        [x0, y0, z_bar], [xf, y0, z_bar],
        [0.1, 0.9, 0.2],
        lineWidth=12,
        lifeTime=0
    )

# ----------------- Game -----------------
def main():
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("No se pudo conectar a PyBullet GUI")

    # Desactiva atajos del visor (por ejemplo W = wireframe)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

    # Evita dibujar mientras se inicializa el render
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    lane_length, lane_half_width, house_center = setup_world()

    HOUSE_R = 0.90
    draw_house(house_center)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Deja que el visor procese unos frames antes de dibujar debug
    for _ in range(10):
        p.stepSimulation()
        time.sleep(1 / 240)

    turns = [
        ("ROJO", (0.85, 0.1, 0.1, 1)),
        ("AMARILLO", (0.95, 0.85, 0.15, 1)),
        ("ROJO", (0.85, 0.1, 0.1, 1)),
        ("AMARILLO", (0.95, 0.85, 0.15, 1)),
        ("ROJO", (0.85, 0.1, 0.1, 1)),
        ("AMARILLO", (0.95, 0.85, 0.15, 1)),
        ("ROJO", (0.85, 0.1, 0.1, 1)),
        ("AMARILLO", (0.95, 0.85, 0.15, 1))
    ]
    turn_idx = 0

    score_red = 0
    score_yel = 0

    aim_y = 0.0
    power = 12.0
    launched = False

    # Detector de parada
    STOP_SPEED = 0.06
    STOP_FRAMES = 45
    stop_counter = 0

    stones = []  # [(team, id)]
    stone_id = spawn_stone(turns[turn_idx][1])

    hud = {
    "score": -1,
    "info": -1,
    "status": -1,
    "aim": None,
    "power_base": None,
    "power_fill": None
}

    # Throttle de HUD
    hud_frame = 0
    HUD_EVERY = 12

    HUD_OFS_X = 0
    HUD_OFS_Y = 1.6
    HUD_Z0    = 1.35    # altura base del HUD
    HUD_LINE  = 0.18    # separación vertical entre líneas
    BAR_GAP   = 0.10    # separación entre barra y texto
    BAR_LEN   = 1.25

    def cleanup_bodies_and_debug():
        try:
            p.removeAllUserDebugItems()
        except Exception:
            pass

        for _, sid in stones:
            try:
                p.removeBody(sid)
            except Exception:
                pass

        try:
            if stone_id is not None and all(sid != stone_id for _, sid in stones):
                p.removeBody(stone_id)
        except Exception:
            pass

    def reset_match():
        nonlocal lane_length, lane_half_width, house_center
        nonlocal turn_idx, score_red, score_yel, aim_y, power, launched, stop_counter, stones, stone_id
        nonlocal hud, hud_frame

        cleanup_bodies_and_debug()

        lane_length, lane_half_width, house_center = setup_world()
        draw_house(house_center)

        turn_idx = 0
        score_red = 0
        score_yel = 0
        aim_y = 0.0
        power = 12.0
        launched = False
        stop_counter = 0
        stones = []
        hud_frame = 0

        stone_id = spawn_stone(turns[turn_idx][1])

        hud["score"] = -1
        hud["info"] = -1
        hud["status"] = -1
        hud["aim"] = None
        hud["power_fill"] = None

    # --------------- Main Loop ---------------
    while True:
        if not p.isConnected():
            break

        hud_frame += 1
        keys = p.getKeyboardEvents()

        # ESC (ASCII 27)
        if 27 in keys and (keys[27] & p.KEY_WAS_TRIGGERED):
            break

        # Reset
        if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
            reset_match()
            continue

        # Estado de piedra
        stone_pos, _ = p.getBasePositionAndOrientation(stone_id)
        lin_vel, _ = p.getBaseVelocity(stone_id)
        speed = math.hypot(lin_vel[0], lin_vel[1])
        d = dist2((stone_pos[0], stone_pos[1]), house_center)

        team_name, _ = turns[turn_idx]
        score_text = f"MARCADOR  ROJO {score_red}  -  {score_yel} AMARILLO"
        info_text = f"Equipo: {team_name} | Tiro {turn_idx + 1}/4 | Dist: {d:.2f} m | Fuerza: {power:.1f}"
        base_controls = "A/D apuntar  W/S fuerza  SPACE lanzar  R reset  ESC salir"

        cam_target = [stone_pos[0], stone_pos[1], 0.0]

        if not launched:
            # Inputs de apuntado
            if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN):
                aim_y -= 0.01
            if ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN):
                aim_y += 0.01
            aim_y = clamp(aim_y, -lane_half_width + 0.10, lane_half_width - 0.10)

            # Inputs de fuerza
            if ord('w') in keys and (keys[ord('w')] & p.KEY_IS_DOWN):
                power += 0.08
            if ord('s') in keys and (keys[ord('s')] & p.KEY_IS_DOWN):
                power -= 0.08
            power = clamp(power, 2.0, 50.0)

            # Línea de apuntado
            rm(hud, "aim")
            start = [stone_pos[0], stone_pos[1], 0.02]
            end = [stone_pos[0] + 2.0, aim_y, 0.02]
            hud["aim"] = p.addUserDebugLine(start, end, [0, 1, 0], 3)

             # HUD y barra de fuerza con throttle
            if hud_frame % HUD_EVERY == 0:
                cam_target = [stone_pos[0], stone_pos[1], 0.0]

                draw_hud_follow_camera(
                    hud, cam_target,
                    score_text, info_text,
                    "Listo para lanzar | " + base_controls if not launched else f"En movimiento | vel: {speed:.2f} m/s",
                    HUD_OFS_X, HUD_OFS_Y, HUD_Z0, HUD_LINE
                )

                draw_power_bar(
                    hud, power, cam_target,
                    HUD_OFS_X, HUD_OFS_Y, HUD_Z0, HUD_LINE,
                    BAR_GAP, BAR_LEN
                )

            # Lanzamiento: fijamos velocidad inicial (v0)
            if p.B3G_SPACE in keys and (keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED):
                vx = power
                vy = (aim_y * 2.2)
                p.resetBaseVelocity(stone_id, linearVelocity=[vx, vy, 0], angularVelocity=[0, 0, 0])
                launched = True
                stop_counter = 0

        else:
            # En movimiento
            rm(hud, "aim")
            if speed < STOP_SPEED:
                stop_counter += 1
            else:
                stop_counter = 0

            if hud_frame % HUD_EVERY == 0:
                draw_hud_follow_camera(
                    hud, cam_target,
                    score_text, info_text,
                    f"En movimiento | vel: {speed:.2f} m/s",
                    ofs_x=HUD_OFS_X, ofs_y=HUD_OFS_Y, z0=HUD_Z0, line=HUD_LINE
                )

        # Cámara cenital siguiendo a la piedra
        set_topdown_camera(stone_pos)

        p.stepSimulation()
        time.sleep(1 / 240)

        # Si se paró de verdad
        if launched and stop_counter >= STOP_FRAMES:
            p.resetBaseVelocity(stone_id, [0, 0, 0], [0, 0, 0])
            stones.append((team_name, stone_id))

            launched = False
            stop_counter = 0
            aim_y = 0.0
            power = 12.0
            turn_idx += 1

            if turn_idx < len(turns):
                stone_id = spawn_stone(turns[turn_idx][1])
            else:
                # Última piedra: calcular puntuación
                red_ds, yel_ds = [], []
                for team, sid in stones:
                    pos, _ = p.getBasePositionAndOrientation(sid)
                    dd = dist2((pos[0], pos[1]), house_center)
                    (red_ds if team == "ROJO" else yel_ds).append(dd)

                pts_r, pts_y, msg = compute_curling_score(red_ds, yel_ds, house_radius=HOUSE_R)
                score_red += pts_r
                score_yel += pts_y

                if score_red > score_yel:
                    winner = f"GANADOR: ROJO ({score_red} - {score_yel})"
                elif score_yel > score_red:
                    winner = f"GANADOR: AMARILLO ({score_red} - {score_yel})"
                else:
                    winner = f"EMPATE ({score_red} - {score_yel})"

                final_anchor = [house_center[0], house_center[1], 0.0]
                set_topdown_camera(final_anchor)

                while p.isConnected():
                    hud_frame += 1
                    if hud_frame % HUD_EVERY == 0:
                        draw_hud_follow_camera(
                            hud, final_anchor,
                            f"MARCADOR FINAL  ROJO {score_red}  -  {score_yel} AMARILLO",
                            winner,
                            msg + " | R reiniciar | ESC salir",
                            ofs_x=HUD_OFS_X, ofs_y=HUD_OFS_Y, z0=HUD_Z0, line=HUD_LINE
                        )

                    p.stepSimulation()
                    time.sleep(1 / 240)

                    keys2 = p.getKeyboardEvents()
                    if 27 in keys2 and (keys2[27] & p.KEY_WAS_TRIGGERED):
                        p.disconnect()
                        return
                    if ord('r') in keys2 and (keys2[ord('r')] & p.KEY_WAS_TRIGGERED):
                        reset_match()
                        break

    if p.isConnected():
        p.disconnect()


if __name__ == "__main__":
    main()