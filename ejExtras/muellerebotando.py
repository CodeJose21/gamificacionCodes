import pymunk
import pygame
import math

# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================
WIDTH, HEIGHT = 600, 800
FPS = 60
BACKGROUND = (255, 255, 255)

BLACK = (0, 0, 0)
GRAY = (120, 120, 120)
BLUE = (50, 100, 255)

# =========================================================
# FUNCIÓN PARA DIBUJAR EL MUELLE
# =========================================================
def draw_zigzag_spring(screen, start, end, num_steps=25, width=15):
    """Dibuja un muelle en zigzag entre dos puntos."""
    start_x, start_y = start
    end_x, end_y = end

    dx = end_x - start_x
    dy = end_y - start_y
    length = math.sqrt(dx**2 + dy**2)

    if length == 0:
        return

    # Vector unitario en la dirección del muelle
    ux = dx / length
    uy = dy / length

    # Vector perpendicular
    px = -uy
    py = ux

    points = [start]

    for i in range(1, num_steps):
        tx = start_x + ux * (length * i / num_steps)
        ty = start_y + uy * (length * i / num_steps)

        offset = width if i % 2 == 0 else -width
        points.append((tx + px * offset, ty + py * offset))

    points.append(end)
    pygame.draw.lines(screen, GRAY, False, points, 2)

def calc_energia_cinetica(body):
    """Calcula la energía cinética de un cuerpo."""
    v = body.velocity.length
    return 0.5 * body.mass * v**2

def calc_energia_potencial(body, ceiling_pos, rest_length, kmuelle):
    """Calcula la energía potencial del sistema."""
    # Energía potencial gravitatoria
    y_equ = ceiling_pos[1] + rest_length + body.mass * 900 / kmuelle
    ep_grav = body.mass * 900 * (y_equ - body.position.y)

    # Energía potencial elástica del muelle
    elongacion = body.position.y - (ceiling_pos[1] + rest_length)
    ep_elastica = 0.5 * kmuelle * elongacion**2

    return ep_grav + ep_elastica

# =========================================================
# PLANTILLA DE LA SIMULACIÓN
# =========================================================
def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Plantilla - muelle amortiguado")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 22)

    # -----------------------------------------------------
    # ESPACIO FÍSICO
    # -----------------------------------------------------
    space = pymunk.Space()

    # AJUSTA TÚ ESTO
    space.gravity = (0, 900)

    # IMPORTANTE:
    # damping = 1.0  -> sin amortiguamiento global
    # damping < 1.0  -> sí hay amortiguamiento global
    space.damping = 1

    # -----------------------------------------------------
    # PUNTO FIJO DEL TECHO
    # -----------------------------------------------------
    ceiling_pos = (300, 50)

    # -----------------------------------------------------
    # PARÁMETROS DEL SISTEMA
    # -----------------------------------------------------
    mass = 8.0
    radius = 40

    # AJUSTA TÚ ESTO
    kmuelle = 150

    # Ejemplo basado en amortiguamiento crítico:
    bmuelle = 0 #math.sqrt(4 * mass * kmuelle) * 0.007

    rest_length = 350

    # -----------------------------------------------------
    # CUERPO MÓVIL
    # -----------------------------------------------------
    moment = pymunk.moment_for_circle(mass, 0, radius)
    body = pymunk.Body(mass, moment)

    # Posición inicial
    body.position = (300, 350)

    # Velocidad inicial
    body.velocity = (0, -80)

    shape = pymunk.Circle(body, radius)
    space.add(body, shape)

    # -----------------------------------------------------
    # MUELLE AMORTIGUADO
    # -----------------------------------------------------
    spring = pymunk.DampedSpring(
        space.static_body,   # cuerpo fijo
        body,                # cuerpo móvil
        ceiling_pos,         # anclaje en el cuerpo fijo
        (0, 0),              # anclaje en el centro de la esfera
        rest_length=rest_length,
        stiffness=kmuelle,
        damping=bmuelle
    )
    space.add(spring)

    # -----------------------------------------------------
    # OPCIONES DE SIMULACIÓN
    # -----------------------------------------------------
    camara_lenta = 1
    running = True

    while running:
        # -------------------------------------------------
        # EVENTOS
        # -------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # -------------------------------------------------
        # PASO DE SIMULACIÓN
        # -------------------------------------------------
        dt = 1.0 / (FPS * camara_lenta)
        space.step(dt)

        # -------------------------------------------------
        # DIBUJO
        # -------------------------------------------------
        screen.fill(BACKGROUND)

        # Techo
        pygame.draw.line(screen, BLACK, (150, 50), (450, 50), 5)

        # Muelle dibujado
        draw_zigzag_spring(screen, ceiling_pos, body.position, num_steps=30, width=15)

        # Esfera
        pos = (int(body.position.x), int(body.position.y))
        pygame.draw.circle(screen, BLUE, pos, radius)
        pygame.draw.circle(screen, BLACK, pos, radius, 2)

        # -------------------------------------------------
        # LÍNEA DE EQUILIBRIO TEÓRICA
        # -------------------------------------------------
        # y_equ = y_techo + longitud_reposo + mg/k
        y_equ = ceiling_pos[1] + rest_length + mass * space.gravity[1] / kmuelle
        pygame.draw.line(screen, GRAY, (0, int(y_equ)), (WIDTH, int(y_equ)), 1)

        # -------------------------------------------------
        # TEXTO EN PANTALLA
        # -------------------------------------------------
        texto1 = font.render(f"y = {body.position.y:.2f}", True, BLACK)
        texto2 = font.render(f"vy = {body.velocity.y:.2f}", True, BLACK)
        texto3 = font.render(f"y_equ = {y_equ:.2f}", True, BLACK)
        ec = calc_energia_cinetica(body)
        ep = calc_energia_potencial(body, ceiling_pos, rest_length, kmuelle)
        em = ec + ep
        texto4 = font.render(f"Ec = {ec:.2f}", True, BLACK)
        texto5 = font.render(f"Ep = {ep:.2f}", True, BLACK)
        texto6 = font.render(f"Em = {em:.2f}", True, BLACK)
        

        screen.blit(texto1, (20, 20))
        screen.blit(texto2, (20, 50))
        screen.blit(texto3, (20, 80))
        screen.blit(texto4, (20, 110))
        screen.blit(texto5, (20, 140))
        screen.blit(texto6, (20, 170))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run_simulation()