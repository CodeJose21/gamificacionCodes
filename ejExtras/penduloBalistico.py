import math
import sys
import pygame
import pymunk
import pymunk.pygame_util

WIDTH, HEIGHT = 1200, 700
FPS = 60

BACKGROUND = (245, 245, 245)
BLACK = (20, 20, 20, 255)
RED = (200, 40, 40, 255)
BLUE = (40, 80, 200, 255)
GREEN = (30, 150, 80, 255)
GRAY = (120, 120, 120, 255)

PIXELS_PER_METER = 220
SUBSTEPS = 100
DT = 1 / FPS
SUB_DT = DT / SUBSTEPS
G = 9.81*PIXELS_PER_METER

MASA_PROYECTIL = 0.03
RADIO_PROYECTIL = 10
VELOCIDAD_INICIAL_M_S = 500.0
VELOCIDAD_INICIAL_PX_S = VELOCIDAD_INICIAL_M_S * PIXELS_PER_METER

MASA_BLOQUE = 0.50
RADIO_BLOQUE = 22

LONGITUD_PENDULO_M = 1.0
LONGITUD_PENDULO_PX = LONGITUD_PENDULO_M * PIXELS_PER_METER

PIVOT_X = 850
PIVOT_Y = 120

PROYECTIL_X0 = 120
PROYECTIL_Y0 = PIVOT_Y + LONGITUD_PENDULO_PX


class Proyectil:
    def __init__(self, space, x, y, masa, radio):
        self.space = space
        self.masa = masa
        self.radio = radio
        self.disparada = False
        self.velocidad_reconstruida_m_s = None

        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = (x, y)
        self.body.velocity = (0, 0)

        self.shape = pymunk.Circle(self.body, radio)
        self.shape.color = RED
        self.shape.elasticity = 0.0
        self.shape.friction = 0.0
        self.shape.collision_type = 1

        space.add(self.body, self.shape)

    def disparar(self, velocidad_x, velocidad_y=0):
        if self.disparada:
            return

        posicion_actual = self.body.position

        self.space.remove(self.body, self.shape)

        moment = pymunk.moment_for_circle(self.masa, 0, self.radio)
        self.body = pymunk.Body(self.masa, moment, body_type=pymunk.Body.DYNAMIC)
        self.body.position = posicion_actual
        self.body.velocity = (velocidad_x, velocidad_y)

        self.shape = pymunk.Circle(self.body, self.radio)
        self.shape.color = RED
        self.shape.elasticity = 0.0
        self.shape.friction = 0.0
        self.shape.collision_type = 1

        self.space.add(self.body, self.shape)
        self.disparada = True

    def comprobar_velocidad_inicial(self, pendulo):
        altura = pendulo.obtener_altura_max_m()
        if altura <= 0:
            return 0.0

        V = math.sqrt(2 * G * altura)
        v = ((self.masa + pendulo.masa_bloque) * V) / self.masa
        return v


class Pendulo:
    def __init__(self, space, pivot_x, pivot_y, masa_bloque, radio_bloque, longitud_px):
        self.space = space
        self.pivot = (pivot_x, pivot_y)
        self.longitud_px = longitud_px

        x_bloque = pivot_x
        y_bloque = pivot_y + longitud_px

        self.masa_bloque = masa_bloque
        self.radio_bloque = radio_bloque

        self.body = pymunk.Body(
            masa_bloque,
            pymunk.moment_for_circle(masa_bloque, 0, radio_bloque)
        )
        self.body.position = (x_bloque, y_bloque)

        self.shape = pymunk.Circle(self.body, radio_bloque)
        self.shape.color = BLUE
        self.shape.elasticity = 0.0
        self.shape.friction = 0.6
        self.shape.collision_type = 2

        self.joint = pymunk.PinJoint(
            self.body,
            space.static_body,
            (0, 0),
            self.pivot
        )

        space.add(self.body, self.shape, self.joint)

        self.impactado = False
        self.y_inicial = y_bloque
        self.altura_max_px = 0.0
        self.altura_max_m_confirmada = False

    def dibujar_cuerda(self, screen):
        x1, y1 = self.pivot
        x2, y2 = self.body.position
        pygame.draw.line(screen, (20, 20, 20), (x1, y1), (x2, y2), 3)

    def actualizar_altura_maxima(self):
        altura_actual = self.y_inicial - self.body.position.y
        if altura_actual > self.altura_max_px:
            self.altura_max_px = altura_actual

    def obtener_altura_actual_m(self):
        altura_actual_px = max(0, self.y_inicial - self.body.position.y)
        return altura_actual_px / PIXELS_PER_METER

    def obtener_altura_max_m(self):
        return self.altura_max_px / PIXELS_PER_METER


def crear_espacio():
    space = pymunk.Space()
    space.gravity = (0, G * PIXELS_PER_METER)
    return space


def dibujar_pivote(screen, x, y):
    pygame.draw.circle(screen, (20, 20, 20), (int(x), int(y)), 6)


def dibujar_texto(screen, texto, x, y, font, color=(20, 20, 20)):
    superficie = font.render(texto, True, color)
    screen.blit(superficie, (x, y))

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


def fusionar_proyectil_con_pendulo(space, proyectil, pendulo):
    if pendulo.impactado:
        return

    m1 = proyectil.masa
    m2 = pendulo.masa_bloque

    space.remove(proyectil.body, proyectil.shape)

    pendulo.body.mass = m1 + m2
    pendulo.body.moment = pymunk.moment_for_circle(m1 + m2, 0, pendulo.radio_bloque)
    pendulo.impactado = True


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Péndulo balístico")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 22)

    draw_options = pymunk.pygame_util.DrawOptions(screen)
    space = crear_espacio()

    proyectil = Proyectil(
        space,
        PROYECTIL_X0,
        PROYECTIL_Y0,
        MASA_PROYECTIL,
        RADIO_PROYECTIL
    )

    pendulo = Pendulo(
        space,
        PIVOT_X,
        PIVOT_Y,
        MASA_BLOQUE,
        RADIO_BLOQUE,
        LONGITUD_PENDULO_PX
    )

    def colision(arbiter, space_interno, data):
        nonlocal proyectil, pendulo
        if not pendulo.impactado and proyectil.disparada:
            fusionar_proyectil_con_pendulo(space_interno, proyectil, pendulo)
        return False

    space.on_collision(1, 2, separate=colision)

    running = True
    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    proyectil.disparar(VELOCIDAD_INICIAL_PX_S, 0)

        for _ in range(SUBSTEPS):
            space.step(SUB_DT)

        if pendulo.impactado:
            pendulo.actualizar_altura_maxima()

            if (
                proyectil.velocidad_reconstruida_m_s is None
                and pendulo.body.velocity.y >= 0
                and pendulo.obtener_altura_max_m() > 0
            ):
                proyectil.velocidad_reconstruida_m_s = proyectil.comprobar_velocidad_inicial(pendulo)

        screen.fill(BACKGROUND)

        pendulo.dibujar_cuerda(screen)
        dibujar_pivote(screen, PIVOT_X, PIVOT_Y)
        space.debug_draw(draw_options)
        if not pendulo.impactado:
                draw_beach_ball(screen,proyectil.body,RADIO_PROYECTIL)

        velocidad_bala_px_s = proyectil.body.velocity.length if not pendulo.impactado else 0
        velocidad_bala_m_s = velocidad_bala_px_s / PIXELS_PER_METER
        velocidad_pendulo_px_s = pendulo.body.velocity.length
        velocidad_pendulo_m_s = velocidad_pendulo_px_s / PIXELS_PER_METER

        angulo = math.degrees(math.atan2(
            pendulo.body.position.x - PIVOT_X,
            pendulo.body.position.y - PIVOT_Y
        ))

        altura_actual_m = pendulo.obtener_altura_actual_m()
        altura_max_m = pendulo.obtener_altura_max_m()

        dibujar_texto(screen, f"Velocidad inicial real programada: {VELOCIDAD_INICIAL_M_S:.3f} m/s", 20, 20, font)
        dibujar_texto(screen, f"Velocidad bala actual: {velocidad_bala_m_s:.3f} m/s", 20, 50, font)
        dibujar_texto(screen, f"Masa proyectil: {MASA_PROYECTIL:.3f} kg", 20, 80, font)
        dibujar_texto(screen, f"Masa bloque: {MASA_BLOQUE:.3f} kg", 20, 110, font)
        dibujar_texto(screen, f"Velocidad actual del pendulo: {velocidad_pendulo_m_s:.3f} m/s", 20, 140, font)
        dibujar_texto(screen, f"Angulo actual: {angulo:.2f} grados", 20, 170, font)
        dibujar_texto(screen, f"Altura actual del pendulo: {altura_actual_m:.4f} m", 20, 200, font)
        dibujar_texto(screen, f"Altura maxima alcanzada: {altura_max_m:.4f} m", 20, 230, font)

        if proyectil.velocidad_reconstruida_m_s is not None:
            error_abs = abs(proyectil.velocidad_reconstruida_m_s - VELOCIDAD_INICIAL_M_S)
            error_rel = 100 * error_abs / VELOCIDAD_INICIAL_M_S if VELOCIDAD_INICIAL_M_S != 0 else 0.0

            dibujar_texto(
                screen,
                f"Velocidad inicial deducida por pendulo balistico: {proyectil.velocidad_reconstruida_m_s:.3f} m/s",
                20, 270, font, GREEN
            )
            dibujar_texto(
                screen,
                f"Error absoluto: {error_abs:.3f} m/s",
                20, 300, font
            )
            dibujar_texto(
                screen,
                f"Error relativo: {error_rel:.2f} %",
                20, 330, font
            )

        if pendulo.impactado:
            dibujar_texto(screen, "Estado: impacto realizado", 20, 380, font, (30, 150, 80))
        elif proyectil.disparada:
            dibujar_texto(screen, "Estado: bala disparada", 20, 380, font, (200, 40, 40))
        else:
            dibujar_texto(screen, "Estado: bala en reposo (pulsa ESPACIO)", 20, 380, font, (200, 40, 40))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()