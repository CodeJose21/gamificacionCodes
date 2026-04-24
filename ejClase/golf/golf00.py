import math
import pygame
import pymunk
import pymunk.pygame_util
import rozamiento_aire as roz
import numpy as np

# =========================================================
# CONFIGURACIÓN
# =========================================================
WIDTH, HEIGHT = 1000, 600
FPS = 60
NIVEL_DEL_SUELO = HEIGHT - 60

PX_M = 5
M_PX = 1 / PX_M
RADIO_M = 0.0213

# Tamaño visual de la bola
RADIO_BOLA = int(RADIO_M * PX_M * 60)

# =========================================================
# SUELO
# =========================================================
def crearSuelo(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)

    suelo = pymunk.Segment(
        body,
        (0, NIVEL_DEL_SUELO),
        (WIDTH, NIVEL_DEL_SUELO),
        2
    )
    suelo.friction = 0.8
    suelo.elasticity = 0.0

    space.add(body, suelo)
    return suelo


# =========================================================
# BOLA FÍSICA
# =========================================================
def crearBola(space, x, y):
    masa = 1
    momento = pymunk.moment_for_circle(masa, 0, RADIO_BOLA)

    body = pymunk.Body(masa, momento)
    body.position = (x, y)

    shape = pymunk.Circle(body, RADIO_BOLA)
    shape.friction = 0.8
    shape.elasticity = 0.8

    space.add(body, shape)
    return body, shape


# =========================================================
# PALO FÍSICO
# =========================================================
def crearPalo(space, pivot_x, pivot_y):
    """
    El palo será un cuerpo cinemático:
    - mango: segmento
    - cabeza: triángulo
    El pivote visual es la parte superior del mango.
    """
    body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
    body.position = (pivot_x, pivot_y)

    largo_mango = 105

    mango = pymunk.Segment(body, (0, 0), (0, largo_mango), 3)
    mango.friction = 0.8
    mango.elasticity = 0.7

    cabeza = pymunk.Poly(body, [
        (0, largo_mango),
        (0, largo_mango + 22),
        (32, largo_mango + 22)
    ])
    cabeza.friction = 0.8
    cabeza.elasticity = 0.7

    space.add(body, mango, cabeza)
    return body, mango, cabeza


# =========================================================
# DIBUJO DEL PALO
# =========================================================
def dibujar_palo(screen, palo_body):
    largo_mango = 105

    p0 = palo_body.local_to_world((0, 0))
    p1 = palo_body.local_to_world((0, largo_mango))

    pygame.draw.line(
        screen,
        (60, 60, 60),
        (int(p0.x), int(p0.y)),
        (int(p1.x), int(p1.y)),
        6
    )

    tri_local = [
        (0, largo_mango),
        (0, largo_mango + 22),
        (32, largo_mango + 22)
    ]
    tri_world = [palo_body.local_to_world(p) for p in tri_local]
    tri_points = [(int(p.x), int(p.y)) for p in tri_world]

    pygame.draw.polygon(screen, (40, 40, 40), tri_points)


# =========================================================
# RUN
# =========================================================
def run():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Golf")
    clock = pygame.time.Clock()

    try:
        bg_image = pygame.image.load("ejClase/golf/calle_golf.png")
        bg_width, bg_height = bg_image.get_size()

        bg_width = int(bg_width * 0.7)
        bg_height = int(bg_height * 0.6)

        aspect_ratio = bg_height / bg_width
        new_height = int(WIDTH * aspect_ratio)

        bg_image = pygame.transform.scale(bg_image, (WIDTH, new_height))
        bg_pos = (0, HEIGHT - new_height)

    except pygame.error:
        print("No se pudo cargar la imagen calle_golf.png. Se usará fondo azul.")
        bg_image = None

    space = pymunk.Space()
    space.gravity = (0, 900)

    crearSuelo(space)

    # Bola física
    x_bola = 100
    y_bola = NIVEL_DEL_SUELO - RADIO_BOLA
    bola_body, bola_shape = crearBola(space, x_bola, y_bola)

    # Palo físico
    pivot_x = x_bola - 40
    pivot_y = y_bola - 120
    palo_body, mango_shape, cabeza_shape = crearPalo(space, pivot_x, pivot_y)

    # Swing
    angulo_inicial = 45
    angulo_final = -35
    velocidad_angular = 300  # grados/s
    swing_activo = False

    palo_body.angle = math.radians(angulo_inicial)
    palo_body.angular_velocity = 0

    running = True
    while running:
        dt = 1.0 / FPS

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not swing_activo:
                    swing_activo = True
                    palo_body.angular_velocity = -math.radians(velocidad_angular)

                if event.key == pygame.K_r:
                    swing_activo = False
                    palo_body.angle = math.radians(angulo_inicial)
                    palo_body.angular_velocity = 0

                    bola_body.position = (x_bola, y_bola)
                    bola_body.velocity = (0, 0)
                    bola_body.angular_velocity = 0
                    bola_body.angle = 0
                    bola_body.activate()

        if swing_activo:
            if palo_body.angle <= math.radians(angulo_final):
                palo_body.angle = math.radians(angulo_final)
                palo_body.angular_velocity = 0
                swing_activo = False

        # =================================================
        # ROZAMIENTO DEL AIRE
        # OJO: estas funciones esperan magnitudes en metros
        # =================================================
        v_m = bola_body.velocity.length * M_PX
        diametro_m = 2 * RADIO_M
        area_m2 = np.pi * (RADIO_M ** 2)

        cd = roz.get_Cd(v=v_m, D=diametro_m, crisis=True, golf=True)
        roz.aplicar_newton(bola_body, area_m2, M_PX=M_PX, Cd=cd)

        space.step(dt)

        screen.fill((135, 206, 235))

        if bg_image:
            screen.blit(bg_image, bg_pos)

        # Dibujar bola de forma segura
        x_draw = bola_body.position.x
        y_draw = bola_body.position.y

        if math.isfinite(x_draw) and math.isfinite(y_draw):
            pygame.draw.circle(
                screen,
                (255, 255, 255),
                (round(x_draw), round(y_draw)),
                RADIO_BOLA
            )
        else:
            print("Posición inválida de la bola:", bola_body.position)

        # Dibujar palo
        dibujar_palo(screen, palo_body)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run()