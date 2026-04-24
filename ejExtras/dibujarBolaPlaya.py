import math
import pygame
import pymunk
import pymunk.pygame_util


# =========================================================
# CONFIGURACIÓN
# =========================================================
WIDTH, HEIGHT = 900, 600
FPS = 60

BACKGROUND = (245, 245, 245)
BLACK = (20, 20, 20)

RADIO = 35
MASA = 1.0


# =========================================================
# PELOTA DE PLAYA
# =========================================================
def draw_beach_ball(screen, body, radius_px):
    # Definimos una función llamada draw_beach_ball.
    # Recibe:
    # - screen: la ventana o superficie de Pygame donde vamos a dibujar.
    # - body: el cuerpo físico de Pymunk que representa la pelota.
    # - radius_px: el radio de la pelota en píxeles.

    sx = int(body.position.x)
    # Guardamos en sx la coordenada X de la pelota.
    # body.position.x viene de Pymunk.
    # int(...) convierte la posición a número entero porque Pygame dibuja con píxeles.

    sy = int(body.position.y)
    # Guardamos en sy la coordenada Y de la pelota.
    # Igual que antes, la convertimos a entero para poder dibujar en pantalla.

    r_px = max(8, int(radius_px))
    # Calculamos el radio final en píxeles.
    # int(radius_px) asegura que sea un número entero.
    # max(8, ...) evita que el radio sea menor de 8 píxeles.
    # Así la pelota siempre será visible aunque se pase un radio muy pequeño.

    colors = [
        (235, 70, 70),
        (255, 210, 60),
        (70, 170, 255),
        (70, 210, 140),
        (255, 140, 60),
        (255, 105, 180),
    ]
    # Creamos una lista de colores RGB.
    # Cada tupla representa un color: (rojo, verde, azul).
    # Estos colores se usarán para pintar las franjas de la pelota.

    pygame.draw.circle(screen, (245, 245, 245), (sx, sy), r_px)
    # Dibujamos un círculo base de color casi blanco.
    # screen: superficie donde se dibuja.
    # (245, 245, 245): color del círculo.
    # (sx, sy): centro del círculo.
    # r_px: radio del círculo.

    angle = body.angle
    # Guardamos el ángulo actual del cuerpo de Pymunk.
    # body.angle cambia cuando la pelota rota.
    # Usamos este ángulo para que las franjas giren con la pelota.

    for i, color in enumerate(colors):
        # Recorremos la lista de colores.
        # enumerate(colors) nos da:
        # - i: índice del color, 0, 1, 2, ...
        # - color: el color actual.

        a0 = angle + i * math.tau / len(colors)
        # Calculamos el ángulo inicial de la franja.
        # math.tau equivale a 2π radianes, es decir, una vuelta completa.
        # Dividimos la vuelta completa entre el número de colores.
        # Multiplicamos por i para colocar cada franja en una zona distinta.
        # Sumamos angle para que la franja rote junto con la pelota.

        a1 = angle + (i + 1) * math.tau / len(colors)
        # Calculamos el ángulo final de la franja.
        # Es el límite angular siguiente al ángulo inicial a0.

        p0 = (sx, sy)
        # Primer punto del triángulo: el centro de la pelota.

        p1 = (
            sx + int(math.cos(a0) * r_px),
            sy + int(math.sin(a0) * r_px),
        )
        # Segundo punto del triángulo.
        # math.cos(a0) calcula la posición horizontal sobre el borde del círculo.
        # math.sin(a0) calcula la posición vertical sobre el borde del círculo.
        # Multiplicamos por r_px para llegar hasta el borde de la pelota.
        # Sumamos sx y sy para desplazarlo desde el centro real de la pelota.

        p2 = (
            sx + int(math.cos(a1) * r_px),
            sy + int(math.sin(a1) * r_px),
        )
        # Tercer punto del triángulo.
        # Es igual que p1, pero usando el ángulo final de la franja.

        pygame.draw.polygon(screen, color, [p0, p1, p2])
        # Dibujamos un triángulo coloreado.
        # Este triángulo representa una franja de la pelota.
        # [p0, p1, p2] son los tres puntos del triángulo.

    pygame.draw.circle(screen, BLACK, (sx, sy), r_px, 2)
    # Dibujamos el contorno exterior de la pelota.
    # BLACK es el color del borde.
    # El último valor, 2, indica que solo queremos el borde con grosor 2.
    # Si no pusieras el 2, se rellenaría el círculo entero.

    pygame.draw.circle(screen, (250, 250, 250), (sx, sy), max(4, r_px // 5))
    # Dibujamos un círculo blanco pequeño en el centro.
    # r_px // 5 hace que su radio sea una quinta parte del radio total.
    # max(4, ...) evita que sea demasiado pequeño.

    pygame.draw.circle(screen, BLACK, (sx, sy), max(4, r_px // 5), 1)
    # Dibujamos el borde negro del círculo central.
    # El último valor, 1, indica grosor 1.

    marker_angle = body.angle
    # Guardamos el ángulo de la pelota para colocar un marcador.
    # Este marcador permite ver claramente que la pelota está rotando.

    mx = sx + int(math.cos(marker_angle) * r_px * 0.65)
    # Calculamos la coordenada X del marcador.
    # math.cos(marker_angle) da la dirección horizontal.
    # r_px * 0.65 coloca el marcador dentro de la pelota, no justo en el borde.

    my = sy + int(math.sin(marker_angle) * r_px * 0.65)
    # Calculamos la coordenada Y del marcador.
    # math.sin(marker_angle) da la dirección vertical.
    # También usamos r_px * 0.65 para que quede dentro de la pelota.

    pygame.draw.circle(screen, (255, 255, 255), (mx, my), max(3, r_px // 8))
    # Dibujamos el marcador como un círculo blanco pequeño.

    pygame.draw.circle(screen, BLACK, (mx, my), max(3, r_px // 8), 1)
    # Dibujamos el borde negro del marcador.


# =========================================================
# CREAR BOLA
# =========================================================
def crear_bola(space, x, y):
    momento = pymunk.moment_for_circle(MASA, 0, RADIO)

    body = pymunk.Body(MASA, momento)
    body.position = (x, y)

    shape = pymunk.Circle(body, RADIO)
    shape.friction = 1.0
    shape.elasticity = 0.0

    space.add(body, shape)

    return body, shape


# =========================================================
# CREAR PLANO INCLINADO
# =========================================================
def crear_plano_inclinado(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)

    # Plano inclinado desde arriba izquierda hacia abajo derecha
    plano = pymunk.Segment(
        body,
        (120, 170),
        (780, 470),
        4
    )

    plano.friction = 1.0
    plano.elasticity = 0.0

    space.add(body, plano)

    return plano


# =========================================================
# MAIN
# =========================================================
def main():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pelota de playa rodando por plano inclinado")

    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0, 900)

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    crear_plano_inclinado(space)
    bola_body, bola_shape = crear_bola(space, 150, 50)

    running = True

    while running:
        dt = 1 / FPS

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Reset con tecla R
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    bola_body.position = (180, 100)
                    bola_body.velocity = (0, 0)
                    bola_body.angular_velocity = 0

        screen.fill(BACKGROUND)

        # Dibuja las formas físicas de Pymunk, incluido el plano inclinado
        space.debug_draw(draw_options)

        # Dibuja encima la pelota personalizada
        draw_beach_ball(screen, bola_body, RADIO)

        pygame.display.flip()

        space.step(dt)

        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()