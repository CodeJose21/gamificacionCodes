import math
import pygame
import pymunk

# =========================================================
# CONFIGURACIÓN
# =========================================================
WIDTH, HEIGHT = 1200, 650
FPS = 60
SUBSTEPS = 5

GRAVEDAD = 900

COLOR_FONDO = (235, 245, 255)
COLOR_RAMPA = (80, 80, 80)
COLOR_COCHE_1 = (40, 120, 220)
COLOR_COCHE_2 = (220, 70, 70)
COLOR_RUEDA = (30, 30, 30)
COLOR_TEXTO = (20, 20, 20)

# =========================================================
# TIPOS DE COLISIÓN
# =========================================================
COLISION_COCHE_1 = 1
COLISION_COCHE_2 = 2

# =========================================================
# FRICCIÓN Y RESTITUCIÓN
# =========================================================
FRICCION_RUEDAS = 0.2
FRICCION_SUELO = 0.3
RESTITUCION_CHOQUE_COCHES = 0.8

# =========================================================
# ESCENARIO
# =========================================================
TRAMO_IZQ_INICIO = (80, 340)
TRAMO_IZQ_FIN = (300, 430)

TRAMO_CENTRO_INICIO = TRAMO_IZQ_FIN
TRAMO_CENTRO_FIN = (900, 430)

TRAMO_DER_INICIO = TRAMO_CENTRO_FIN
TRAMO_DER_FIN = (1120, 340)

# =========================================================
# COCHE
# =========================================================
MASA_CHASIS = 3
ANCHO_CHASIS = 80
ALTO_CHASIS = 25

MASA_RUEDA = 0.7
RADIO_RUEDA = 12

ALTURA_COCHE_SOBRE_PLANO = RADIO_RUEDA + ALTO_CHASIS / 2 + 4

# =========================================================
# MOTOR
# =========================================================
TORQUE_MOTOR = 220000
VELOCIDAD_MAX_RUEDA = 180
VELOCIDAD_MINIMA = 2


# =========================================================
# UTILIDADES
# =========================================================
def dibujar_texto(screen, font, texto, x, y, color=COLOR_TEXTO):
    img = font.render(texto, True, color)
    screen.blit(img, (x, y))


def rotar_vector(vector, angulo):
    x, y = vector
    cos_a = math.cos(angulo)
    sin_a = math.sin(angulo)

    return pymunk.Vec2d(
        x * cos_a - y * sin_a,
        x * sin_a + y * cos_a
    )


def obtener_vector_adelante(chasis):
    return pymunk.Vec2d(
        math.cos(chasis.angle),
        math.sin(chasis.angle)
    )


def obtener_velocidad_adelante(chasis):
    adelante = obtener_vector_adelante(chasis)
    return chasis.velocity.dot(adelante)


def modulo_velocidad(body):
    return body.velocity.length


def punto_sobre_segmento(p1, p2, t):
    x1, y1 = p1
    x2, y2 = p2

    return pymunk.Vec2d(
        x1 + (x2 - x1) * t,
        y1 + (y2 - y1) * t
    )


def angulo_segmento(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return math.atan2(y2 - y1, x2 - x1)


def normal_arriba_segmento(angulo):
    return pymunk.Vec2d(
        math.sin(angulo),
        -math.cos(angulo)
    )


def posicion_inicial_coche_en_tramo(p1, p2, t):
    angulo = angulo_segmento(p1, p2)
    punto = punto_sobre_segmento(p1, p2, t)
    normal_arriba = normal_arriba_segmento(angulo)

    posicion = punto + normal_arriba * ALTURA_COCHE_SOBRE_PLANO

    return posicion, angulo


# =========================================================
# ESCENARIO
# =========================================================
def crear_escenario(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)

    tramo_izq = pymunk.Segment(
        body,
        TRAMO_IZQ_INICIO,
        TRAMO_IZQ_FIN,
        5
    )
    tramo_izq.friction = FRICCION_SUELO
    tramo_izq.elasticity = 0.0

    tramo_centro = pymunk.Segment(
        body,
        TRAMO_CENTRO_INICIO,
        TRAMO_CENTRO_FIN,
        5
    )
    tramo_centro.friction = FRICCION_SUELO
    tramo_centro.elasticity = 0.0

    tramo_der = pymunk.Segment(
        body,
        TRAMO_DER_INICIO,
        TRAMO_DER_FIN,
        5
    )
    tramo_der.friction = FRICCION_SUELO
    tramo_der.elasticity = 0.0

    space.add(body, tramo_izq, tramo_centro, tramo_der)

    return tramo_izq, tramo_centro, tramo_der


def dibujar_escenario(screen):
    pygame.draw.line(
        screen,
        COLOR_RAMPA,
        TRAMO_IZQ_INICIO,
        TRAMO_IZQ_FIN,
        10
    )

    pygame.draw.line(
        screen,
        COLOR_RAMPA,
        TRAMO_CENTRO_INICIO,
        TRAMO_CENTRO_FIN,
        10
    )

    pygame.draw.line(
        screen,
        COLOR_RAMPA,
        TRAMO_DER_INICIO,
        TRAMO_DER_FIN,
        10
    )


# =========================================================
# CREAR COCHE
# =========================================================
def crear_coche(space, posicion_chasis, angulo_chasis, collision_type):
    momento_chasis = pymunk.moment_for_box(
        MASA_CHASIS,
        (ANCHO_CHASIS, ALTO_CHASIS)
    )

    chasis = pymunk.Body(MASA_CHASIS, momento_chasis)
    chasis.position = posicion_chasis
    chasis.angle = angulo_chasis

    shape_chasis = pymunk.Poly.create_box(
        chasis,
        (ANCHO_CHASIS, ALTO_CHASIS)
    )

    shape_chasis.friction = 0.2
    shape_chasis.elasticity = RESTITUCION_CHOQUE_COCHES
    shape_chasis.collision_type = collision_type

    space.add(chasis, shape_chasis)

    ruedas = []

    posiciones_ruedas_locales = [
        (-28, 18),
        (28, 18)
    ]

    for pos_local in posiciones_ruedas_locales:
        pos_rueda = chasis.position + rotar_vector(pos_local, chasis.angle)

        momento_rueda = pymunk.moment_for_circle(
            MASA_RUEDA,
            0,
            RADIO_RUEDA
        )

        rueda = pymunk.Body(MASA_RUEDA, momento_rueda)
        rueda.position = pos_rueda
        rueda.angle = chasis.angle

        shape_rueda = pymunk.Circle(rueda, RADIO_RUEDA)
        shape_rueda.friction = FRICCION_RUEDAS
        shape_rueda.elasticity = RESTITUCION_CHOQUE_COCHES
        shape_rueda.collision_type = collision_type

        space.add(rueda, shape_rueda)

        eje = pymunk.PivotJoint(
            chasis,
            rueda,
            pos_rueda
        )
        eje.collide_bodies = False

        space.add(eje)

        ruedas.append(rueda)

    return {
        "chasis": chasis,
        "shape_chasis": shape_chasis,
        "ruedas": ruedas,
        "collision_type": collision_type
    }


# =========================================================
# MOTOR
# =========================================================
def aplicar_torque_ruedas(ruedas, torque):
    for rueda in ruedas:
        rueda.torque += torque


def limitar_velocidad_ruedas(ruedas, velocidad_maxima):
    for rueda in ruedas:
        if rueda.angular_velocity > velocidad_maxima:
            rueda.angular_velocity = velocidad_maxima

        elif rueda.angular_velocity < -velocidad_maxima:
            rueda.angular_velocity = -velocidad_maxima


def controlar_coche(chasis, ruedas, avanzar, retroceder):
    """
    Sin frenado artificial.

    Si se pulsa avanzar:
        aplica torque positivo.

    Si se pulsa retroceder:
        aplica torque negativo.

    Si no se pulsa nada:
        no se aplica torque.
        El coche se frena únicamente por rozamiento rueda-suelo.
    """
    if avanzar and not retroceder:
        aplicar_torque_ruedas(ruedas, TORQUE_MOTOR)

    elif retroceder and not avanzar:
        aplicar_torque_ruedas(ruedas, -TORQUE_MOTOR)

    limitar_velocidad_ruedas(ruedas, VELOCIDAD_MAX_RUEDA)


# =========================================================
# COLLISION HANDLER
# =========================================================
def configurar_collision_handler(space, coche_1, coche_2, estado_colision):
    def comienza_colision(arbiter, space, data):
        frame_actual = estado_colision["frame"]

        if frame_actual - estado_colision["ultimo_print"] > 20:
            chasis_1 = coche_1["chasis"]
            chasis_2 = coche_2["chasis"]

            v1 = chasis_1.velocity
            v2 = chasis_2.velocity

            print("====================================")
            print("COLISIÓN ENTRE COCHES")
            print(
                f"Coche azul | "
                f"vx = {v1.x:.2f} px/s | "
                f"vy = {v1.y:.2f} px/s | "
                f"|v| = {v1.length:.2f} px/s"
            )
            print(
                f"Coche rojo | "
                f"vx = {v2.x:.2f} px/s | "
                f"vy = {v2.y:.2f} px/s | "
                f"|v| = {v2.length:.2f} px/s"
            )
            print("====================================")

            estado_colision["ultimo_print"] = frame_actual

        return True

    space.on_collision(
        COLISION_COCHE_1,
        COLISION_COCHE_2,
        begin=comienza_colision
    )


# =========================================================
# DIBUJO
# =========================================================
def dibujar_coche(screen, coche, color_coche):
    chasis = coche["chasis"]
    shape_chasis = coche["shape_chasis"]
    ruedas = coche["ruedas"]

    vertices = [
        chasis.local_to_world(v)
        for v in shape_chasis.get_vertices()
    ]

    puntos = [
        (int(v.x), int(v.y))
        for v in vertices
    ]

    pygame.draw.polygon(screen, color_coche, puntos)
    pygame.draw.polygon(screen, COLOR_TEXTO, puntos, 2)

    for rueda in ruedas:
        x = int(rueda.position.x)
        y = int(rueda.position.y)

        pygame.draw.circle(
            screen,
            COLOR_RUEDA,
            (x, y),
            RADIO_RUEDA
        )

        pygame.draw.circle(
            screen,
            (220, 220, 220),
            (x, y),
            RADIO_RUEDA,
            2
        )

        marca_x = x + int(math.cos(rueda.angle) * RADIO_RUEDA)
        marca_y = y + int(math.sin(rueda.angle) * RADIO_RUEDA)

        pygame.draw.line(
            screen,
            (255, 255, 255),
            (x, y),
            (marca_x, marca_y),
            3
        )


# =========================================================
# RUN
# =========================================================
def run():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Dos coches con rozamiento y choque elástico")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 20)

    space = pymunk.Space()
    space.gravity = (0, GRAVEDAD)

    crear_escenario(space)

    pos_1, ang_1 = posicion_inicial_coche_en_tramo(
        TRAMO_IZQ_INICIO,
        TRAMO_IZQ_FIN,
        0.25
    )

    coche_1 = crear_coche(
        space,
        posicion_chasis=pos_1,
        angulo_chasis=ang_1,
        collision_type=COLISION_COCHE_1
    )

    pos_2, ang_2 = posicion_inicial_coche_en_tramo(
        TRAMO_DER_INICIO,
        TRAMO_DER_FIN,
        0.75
    )

    coche_2 = crear_coche(
        space,
        posicion_chasis=pos_2,
        angulo_chasis=ang_2,
        collision_type=COLISION_COCHE_2
    )

    estado_colision = {
        "frame": 0,
        "ultimo_print": -1000
    }

    configurar_collision_handler(
        space,
        coche_1,
        coche_2,
        estado_colision
    )

    frame = 0
    running = True

    while running:
        dt = 1.0 / FPS
        frame += 1
        estado_colision["frame"] = frame

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # Coche azul: W / S
        controlar_coche(
            coche_1["chasis"],
            coche_1["ruedas"],
            avanzar=keys[pygame.K_w],
            retroceder=keys[pygame.K_s]
        )

        # Coche rojo: flecha arriba / abajo
        # Se invierte para que flecha arriba lo lleve hacia el centro.
        controlar_coche(
            coche_2["chasis"],
            coche_2["ruedas"],
            avanzar=keys[pygame.K_DOWN],
            retroceder=keys[pygame.K_UP]
        )

        for _ in range(SUBSTEPS):
            space.step(dt / SUBSTEPS)

        v1 = obtener_velocidad_adelante(coche_1["chasis"])
        v2 = obtener_velocidad_adelante(coche_2["chasis"])

        screen.fill(COLOR_FONDO)

        dibujar_escenario(screen)
        dibujar_coche(screen, coche_1, COLOR_COCHE_1)
        dibujar_coche(screen, coche_2, COLOR_COCHE_2)

        dibujar_texto(screen, font, "Azul: W / S", 20, 20)
        dibujar_texto(screen, font, "Rojo: Flecha arriba / abajo", 20, 45)

        dibujar_texto(
            screen,
            font,
            f"Rozamiento ruedas: {FRICCION_RUEDAS}",
            20,
            85
        )

        dibujar_texto(
            screen,
            font,
            f"Rozamiento suelo: {FRICCION_SUELO}",
            20,
            110
        )

        dibujar_texto(
            screen,
            font,
            f"Restitucion choque: {RESTITUCION_CHOQUE_COCHES}",
            20,
            135
        )

        dibujar_texto(
            screen,
            font,
            f"Velocidad azul adelante: {v1:.2f} px/s",
            20,
            175
        )

        dibujar_texto(
            screen,
            font,
            f"Velocidad rojo adelante: {v2:.2f} px/s",
            20,
            200
        )

        dibujar_texto(
            screen,
            font,
            f"|v| azul: {modulo_velocidad(coche_1['chasis']):.2f} px/s",
            20,
            240
        )

        dibujar_texto(
            screen,
            font,
            f"|v| rojo: {modulo_velocidad(coche_2['chasis']):.2f} px/s",
            20,
            265
        )

        dibujar_texto(
            screen,
            font,
            "Collision handler imprime velocidades al chocar",
            20,
            310
        )

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run()