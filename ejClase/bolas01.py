import pygame
import pymunk
import pymunk.pygame_util
import sys

# Configuración
WIDTH, HEIGHT = 1200, 700
GREEN_TABLE = (34, 139, 34)
WHITE = (255, 255, 255)
RED = (200, 0, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
FPS = 60
V0 = 250.0
RADIO = 25

class BolaPymunk:
    def __init__(self, space, x, y, color, e, masa=1.0):
        self.masa = masa

        # 1. Crear el cuerpo (Body)
        moment = pymunk.moment_for_circle(masa, 0, RADIO)
        self.body = pymunk.Body(masa, moment)
        self.body.position = (x, y)

        # 2. Crear la forma (Shape)
        self.shape = pymunk.Circle(self.body, RADIO)
        self.shape.elasticity = e
        self.shape.friction = 0.0  # Para simular colisión pura en 1D
        self.color = color

        # 3. Añadir al espacio
        space.add(self.body, self.shape)

    def dibujar(self, pantalla):
        pos = self.body.position
        pygame.draw.circle(pantalla, self.color, (int(pos.x), int(pos.y)), RADIO)

    def energia_cinetica(self):
        vx, vy = self.body.velocity
        v2 = vx**2 + vy**2
        return 0.5 * self.masa * v2

    def momento_lineal_x(self):
        vx, _ = self.body.velocity
        return self.masa * vx


def dibujar_datos_bola(pantalla, fuente, bola, nombre, x, y, color_texto):
    ec = bola.energia_cinetica()
    p = bola.momento_lineal_x()
    vx = bola.body.velocity.x

    texto1 = fuente.render(f"{nombre}", True, color_texto)
    texto2 = fuente.render(f"vx = {vx:.2f} px/s", True, color_texto)
    texto3 = fuente.render(f"Ec = {ec:.2f} J", True, color_texto)
    texto4 = fuente.render(f"p = {p:.2f} kg·px/s", True, color_texto)

    pantalla.blit(texto1, (x, y))
    pantalla.blit(texto2, (x, y + 20))
    pantalla.blit(texto3, (x, y + 40))
    pantalla.blit(texto4, (x, y + 60))


def dibujar_datos_fila(pantalla, fuente, bola_blanca, bola_roja, titulo, x, y):
    ec_total = bola_blanca.energia_cinetica() + bola_roja.energia_cinetica()
    p_total = bola_blanca.momento_lineal_x() + bola_roja.momento_lineal_x()

    texto_titulo = fuente.render(titulo, True, YELLOW)
    texto_total1 = fuente.render(f"Ec total = {ec_total:.2f} J", True, CYAN)
    texto_total2 = fuente.render(f"p total = {p_total:.2f} kg·px/s", True, CYAN)

    pantalla.blit(texto_titulo, (x, y))
    pantalla.blit(texto_total1, (x, y + 20))
    pantalla.blit(texto_total2, (x, y + 40))


def main():
    pygame.init()
    pantalla = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pymunk: Energía cinética y momento lineal")
    reloj = pygame.time.Clock()
    fuente = pygame.font.SysFont("Arial", 18)

    # Inicializar Espacio de Pymunk
    space = pymunk.Space()
    space.gravity = (0, 0)

    # Creación de pares de bolas
    # Fila 1: Elástica (1.0 * 1.0 = 1.0)
    b1_blanca = BolaPymunk(space, 50, 150, WHITE, 1.0)
    b1_roja = BolaPymunk(space, 500, 150, RED, 1.0)

    # Fila 2: Inelástica (1.0 * 0.0 = 0.0)
    b2_blanca = BolaPymunk(space, 50, 300, WHITE, 1.0)
    b2_roja = BolaPymunk(space, 500, 300, RED, 0.0)

    # Fila 3: Parcial (1.0 * 0.5 = 0.5)
    b3_blanca = BolaPymunk(space, 50, 450, WHITE, 1.0)
    b3_roja = BolaPymunk(space, 500, 450, RED, 0.5)

    bolas = [b1_blanca, b1_roja, b2_blanca, b2_roja, b3_blanca, b3_roja]
    bolas_lanzadas = 0

    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if evento.type == pygame.KEYDOWN:
                if evento.key == pygame.K_SPACE:
                    if bolas_lanzadas == 0:
                        b1_blanca.body.velocity = (V0, 0)
                        bolas_lanzadas += 1
                    elif bolas_lanzadas == 1:
                        b2_blanca.body.velocity = (V0, 0)
                        bolas_lanzadas += 1
                    elif bolas_lanzadas == 2:
                        b3_blanca.body.velocity = (V0, 0)
                        bolas_lanzadas += 1

        # Actualizar el motor físico
        dt = 1.0 / FPS
        space.step(dt)

        # Dibujo
        pantalla.fill(GREEN_TABLE)

        pantalla.blit(fuente.render("Pulsa ESPACIO para lanzar", True, WHITE), (20, 20))
        pantalla.blit(fuente.render("Elástica (e=1.0)", True, WHITE), (550, 90))
        pantalla.blit(fuente.render("Inelástica (e=0.0)", True, WHITE), (550, 240))
        pantalla.blit(fuente.render("Parcial (e=0.5)", True, WHITE), (550, 390))

        for b in bolas:
            b.dibujar(pantalla)

        # -----------------------------
        # DATOS FILA 1
        # -----------------------------
        dibujar_datos_bola(pantalla, fuente, b1_blanca, "Blanca 1", 700, 120, WHITE)
        dibujar_datos_bola(pantalla, fuente, b1_roja, "Roja 1", 950, 120, RED)
        dibujar_datos_fila(pantalla, fuente, b1_blanca, b1_roja, "Totales fila 1", 700, 205)

        # -----------------------------
        # DATOS FILA 2
        # -----------------------------
        dibujar_datos_bola(pantalla, fuente, b2_blanca, "Blanca 2", 700, 270, WHITE)
        dibujar_datos_bola(pantalla, fuente, b2_roja, "Roja 2", 950, 270, RED)
        dibujar_datos_fila(pantalla, fuente, b2_blanca, b2_roja, "Totales fila 2", 700, 355)

        # -----------------------------
        # DATOS FILA 3
        # -----------------------------
        dibujar_datos_bola(pantalla, fuente, b3_blanca, "Blanca 3", 700, 420, WHITE)
        dibujar_datos_bola(pantalla, fuente, b3_roja, "Roja 3", 950, 420, RED)
        dibujar_datos_fila(pantalla, fuente, b3_blanca, b3_roja, "Totales fila 3", 700, 505)

        pygame.display.flip()
        reloj.tick(FPS)

if __name__ == "__main__":
    main()