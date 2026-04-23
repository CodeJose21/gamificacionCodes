import sys
import pygame
import pymunk
import pymunk.pygame_util

# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================
ANCHO = 1200
ALTO = 700
FPS = 120

COLOR_FONDO = (25, 25, 35)
COLOR_MOVIL = (220, 80, 80)
COLOR_ESTATICO = (80, 160, 220)
COLOR_TEXTO = (240, 240, 240)

# =========================================================
# PARÁMETROS DEL CUERPO CINEMÁTICO
# =========================================================
RADIO_MOVIL = 20
POS_INICIAL_MOVIL = (100, ALTO // 2)
VELOCIDAD_INICIAL = (2200.0, 0.0)   # velocidad muy elevada

ELASTICIDAD_MOVIL = 1.0
FRICCION_MOVIL = 0.0

# =========================================================
# PARÁMETROS DEL ELEMENTO ESTÁTICO
# =========================================================
POS_BLOQUE = (900, ALTO // 2)
TAM_BLOQUE = (40, 220)

ELASTICIDAD_ESTATICO = 1.0
FRICCION_ESTATICO = 0.0

# =========================================================
# FUNCIONES AUXILIARES
# =========================================================
def dibujar_texto(screen, font, texto, x, y, color=COLOR_TEXTO):
    superficie = font.render(texto, True, color)
    screen.blit(superficie, (x, y))


def crear_cuerpo_cinematico(space, posicion, velocidad):
    """
    Crea un cuerpo cinemático (KINEMATIC).
    Un cuerpo cinemático no responde a fuerzas ni a impulsos.
    Su movimiento lo decides tú asignando velocidad o posición.
    """
    body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
    body.position = posicion
    body.velocity = velocidad

    shape = pymunk.Circle(body, RADIO_MOVIL)
    shape.elasticity = ELASTICIDAD_MOVIL
    shape.friction = FRICCION_MOVIL

    space.add(body, shape)
    return body, shape


def crear_bloque_estatico(space, posicion, tam):
    """
    Crea un bloque estático.
    """
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = posicion

    shape = pymunk.Poly.create_box(body, tam)
    shape.elasticity = ELASTICIDAD_ESTATICO
    shape.friction = FRICCION_ESTATICO

    space.add(body, shape)
    return body, shape


def crear_paredes(space, ancho, alto, grosor=10):
    """
    Paredes estáticas para evitar que el cuerpo salga de la pantalla.
    """
    segmentos = [
        pymunk.Segment(space.static_body, (0, grosor), (ancho, grosor), grosor),
        pymunk.Segment(space.static_body, (0, alto - grosor), (ancho, alto - grosor), grosor),
        pymunk.Segment(space.static_body, (grosor, 0), (grosor, alto), grosor),
        pymunk.Segment(space.static_body, (ancho - grosor, 0), (ancho - grosor, alto), grosor),
    ]

    for s in segmentos:
        s.elasticity = 1.0
        s.friction = 0.0

    space.add(*segmentos)


def imprimir_datos(body):
    print("----- CUERPO CINEMÁTICO -----")
    print(f"Posición: ({body.position.x:.2f}, {body.position.y:.2f})")
    print(f"Velocidad: ({body.velocity.x:.2f}, {body.velocity.y:.2f})")
    print()


# =========================================================
# PROGRAMA PRINCIPAL
# =========================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Pymunk - Cuerpo cinemático vs elemento estático")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 20, bold=True)

    # Espacio físico
    space = pymunk.Space()
    space.gravity = (0, 0)

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    # Crear objetos
    cuerpo_movil, shape_movil = crear_cuerpo_cinematico(
        space,
        POS_INICIAL_MOVIL,
        VELOCIDAD_INICIAL
    )

    bloque_estatico, shape_bloque = crear_bloque_estatico(
        space,
        POS_BLOQUE,
        TAM_BLOQUE
    )

    crear_paredes(space, ANCHO, ALTO)

   # Handler de colisión
    def on_collision(arbiter, space, data):
        print(">>> COLISIÓN DETECTADA")
        imprimir_datos(cuerpo_movil)
        return True

    space.on_collision(None, None, begin=on_collision)

    running = True

    while running:
        dt = 1 / FPS

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    cuerpo_movil.position = POS_INICIAL_MOVIL
                    cuerpo_movil.velocity = VELOCIDAD_INICIAL
                    print(">>> REINICIO")

                elif event.key == pygame.K_SPACE:
                    if cuerpo_movil.velocity.length > 0:
                        cuerpo_movil.velocity = (0, 0)
                        print(">>> VELOCIDAD PUESTA A 0")
                    else:
                        cuerpo_movil.velocity = VELOCIDAD_INICIAL
                        print(">>> VELOCIDAD RESTAURADA")

        # Paso de simulación
        space.step(dt)

        # Dibujado
        screen.fill(COLOR_FONDO)
        space.debug_draw(draw_options)

        dibujar_texto(screen, font, "Cuerpo móvil: KINEMATIC", 20, 20)
        dibujar_texto(screen, font, f"Posición: ({cuerpo_movil.position.x:.1f}, {cuerpo_movil.position.y:.1f})", 20, 55)
        dibujar_texto(screen, font, f"Velocidad: ({cuerpo_movil.velocity.x:.1f}, {cuerpo_movil.velocity.y:.1f})", 20, 90)
        dibujar_texto(screen, font, "R = reiniciar | ESPACIO = parar/reanudar | ESC = salir", 20, 125)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()