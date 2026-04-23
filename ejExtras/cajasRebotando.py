import sys

import pymunk
import pygame
import random

# 1. Configuración de la ventana (Pygame)
pygame.init()
pantalla = pygame.display.set_mode((700, 600))
pygame.display.set_caption("Cajas: colores + separación + rebote + reset (R)")
reloj = pygame.time.Clock()

# 2. Configuración del mundo físico (Pymunk)
espacio = pymunk.Space()

# Más tiempo en el aire + rebotes largos
espacio.gravity = (0, 400)   # menos gravedad => suben/bajan más lento
espacio.damping = 0.997      # cerca de 1 => pierden poca energía con el tiempo

# --- CREAR EL SUELO ---
suelo = pymunk.Segment(espacio.static_body, (0, 560), (1000, 560), 6)
suelo.elasticity = 0.95
suelo.friction = 0.2
espacio.add(suelo)

FUENTE = pygame.font.SysFont("consolas", 16)

# --- FUNCIÓN PARA CREAR CAJAS ---
def crear_caja(x, y, lado, masa, elasticidad, color):
    momento = pymunk.moment_for_box(masa, (lado, lado))
    cuerpo = pymunk.Body(masa, momento)
    cuerpo.position = (x, y)

    forma = pymunk.Poly.create_box(cuerpo, (lado, lado))
    forma.elasticity = elasticidad
    forma.friction = 0.2

    espacio.add(cuerpo, forma)
    return cuerpo, forma, color, lado, masa, elasticidad


# --- CAJAS INICIALES (más espacio + colores distintos) ---
config_cajas = [
    (120, 80, 50, 1, 0.98, (235, 129, 27)),  # naranja
    (260, 80, 50, 1, 0.90, (52, 152, 219)),  # azul
    (400, 80, 50, 1, 0.75, (46, 204, 113)),  # verde
    (540, 80, 50, 1, 0.60, (155, 89, 182)),  # morado
]

def crear_cajas_iniciales():
    return [crear_caja(*cfg) for cfg in config_cajas]

cajas = crear_cajas_iniciales()

# Bonus: crear nuevas cajas con ESPACIO (color aleatorio)
def nueva_caja_aleatoria():
    x = random.randint(80, 620)
    y = 30
    lado = random.choice([30, 40, 50, 60])
    masa = random.choice([0.5, 1, 2, 5])
    elasticidad = random.choice([0.6, 0.8, 0.9, 0.98])
    color = random.choice([
        (231, 76, 60),   # rojo
        (241, 196, 15),  # amarillo
        (26, 188, 156),  # turquesa
        (52, 73, 94),    # azul oscuro
        (230, 126, 34),  # naranja
    ])
    cajas.append(crear_caja(x, y, lado, masa, elasticidad, color))

def resetear():
    """Borra todas las cajas del espacio y recrea las iniciales."""
    global cajas
    # quitar del motor físico
    for cuerpo, forma, *_ in cajas:
        espacio.remove(cuerpo, forma)
    # recrear
    cajas = crear_cajas_iniciales()

# 3. Bucle Principal
ejecutando = True
while ejecutando:
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            ejecutando = False

        if evento.type == pygame.KEYDOWN:
            if evento.key == pygame.K_SPACE:
                nueva_caja_aleatoria()

            if evento.key == pygame.K_r:
                resetear()

    # Física
    dt = 1 / 60.0
    espacio.step(dt)

    # Dibujo
    pantalla.fill((255, 255, 255))

    # Suelo visible
    pygame.draw.line(pantalla, (0, 0, 0), (20, 560), (680, 560), 6)

    # Cajas rotadas con colores
    for cuerpo, forma, color, lado, masa, e in cajas:
        puntos = forma.get_vertices()
        puntos = [p.rotated(cuerpo.angle) + cuerpo.position for p in puntos]
        puntos = [(int(p.x), int(p.y)) for p in puntos]
        pygame.draw.polygon(pantalla, color, puntos)

        x, y = cuerpo.position
        txt = FUENTE.render(f"m={masa} e={e}", True, (20, 20, 20))
        pantalla.blit(txt, (int(x - lado), int(y - lado - 18)))

    info = FUENTE.render("ESPACIO: nueva caja | R: reset | gravity=400 | damping=0.997", True, (20, 20, 20))
    pantalla.blit(info, (10, 10))

    pygame.display.flip()
    reloj.tick(60)
    print(f"TAMAÑO RAM: {sys.getsizeof(espacio.bodies) + sys.getsizeof(espacio.shapes)} bytes")

pygame.quit()