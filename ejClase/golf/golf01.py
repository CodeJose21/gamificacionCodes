import pygame
import pymunk
import pymunk.pygame_util

WIDTH, HEIGHT = 1000, 600
FPS = 60
NIVEL_DEL_SUELO = HEIGHT - 60

# Escala
PX_M = 1000
M_PX = 1.0 / PX_M

# Bola y hoyo
RADIO_M = 0.0213
RADIO_PX = RADIO_M * PX_M

X_HOYO_PX = 500

ANCHO_HOYO_M = 4 * RADIO_M
ANCHO_HOYO_PX = ANCHO_HOYO_M * PX_M

X_INICIO_HOYO = X_HOYO_PX - ANCHO_HOYO_PX / 2
X_FIN_HOYO = X_HOYO_PX + ANCHO_HOYO_PX / 2


def crearSueloAntes(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    shape = pymunk.Segment(
        body,
        (0, NIVEL_DEL_SUELO),
        (X_INICIO_HOYO, NIVEL_DEL_SUELO),
        2
    )
    shape.elasticity = 0.5
    shape.friction = 0.7
    space.add(body, shape)


def crearSueloDespues(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    shape = pymunk.Segment(
        body,
        (X_FIN_HOYO, NIVEL_DEL_SUELO),
        (WIDTH, NIVEL_DEL_SUELO),
        2
    )
    shape.elasticity = 0.5
    shape.friction = 0.7
    space.add(body, shape)


def crear_bola(space, posicion):
    masa = 0.04593
    momento = pymunk.moment_for_circle(masa, 0, RADIO_PX)

    body = pymunk.Body(masa, momento)
    body.position = posicion

    shape = pymunk.Circle(body, RADIO_PX)
    shape.elasticity = 0.7
    shape.friction = 0.7

    space.add(body, shape)
    return body, shape


def run():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    space = pymunk.Space()
    space.gravity = (0, 900)

    crearSueloAntes(space)
    crearSueloDespues(space)

    # Bola justo encima del hoyo
    bola_body, bola_shape = crear_bola(
        space,
        (X_HOYO_PX, NIVEL_DEL_SUELO - RADIO_PX - 80)
    )

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((135, 206, 235))

        dt = 1.0 / FPS
        space.step(dt)

        space.debug_draw(draw_options)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run()