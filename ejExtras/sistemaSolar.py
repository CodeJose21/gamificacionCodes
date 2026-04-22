import pygame
import pymunk
import pymunk.pygame_util
import math
import sys

# =========================================================
# CONFIGURACIÓN
# =========================================================
WIDTH, HEIGHT = 1400, 900
FPS = 60
BACKGROUND = (10, 10, 25)

# Escala visual en pantalla
DRAW_SCALE = 1 / 8_000_000_000  # 8e9 m -> 1 px aprox

# Paso temporal del motor físico (1 dia por frame)
DT = 60*60*24

# Constante de gravitación universal
G = 6.67430e-11


# =========================================================
# CLASE CUERPO CELESTE
# =========================================================
class CuerpoCeleste:
    def __init__(self, space, nombre, masa, x, y, vx, vy, radio_px, color, es_estatico = False):
        self.nombre = nombre
        self.masa = masa
        self.color = color
        self.radio_px = radio_px

        if(es_estatico):
            self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        else:
            momento = pymunk.moment_for_circle(masa, 0, radio_px)
            self.body = pymunk.Body(masa, momento)

        self.body.position = (x, y)
        self.body.velocity = (vx, vy)

        self.shape = pymunk.Circle(self.body, radio_px)
        self.shape.elasticity = 0.0
        self.shape.friction = 0.0

        space.add(self.body, self.shape)
# =====================================
#    FÍSICA
# =====================================
    def aplicar_gravedad_del_sol(self, sol):
       

        vector = sol.body.position - self.body.position
        direccion = vector.normalized()
        fuerza_modulo = G*sol.masa*self.masa/(vector.length)**2
        fuerza_vector = direccion * fuerza_modulo
        self.body.apply_force_at_world_point(fuerza_vector, self.body.position)

    def calcular_energia_cinetica(self):
        vx = self.body.velocity.x
        vy = self.body.velocity.y
        v2 = vx**2 + vy**2
        ec = 0.5 * self.masa * v2
        return ec
    
    def calcular_energia_potencial(self,sol):
        vector = sol.body.position - self.body.position
        ep = G*sol.masa*self.masa/(vector.length)
        return ep
    
    def calcular_energia_mecanica(self, sol):
        return self.calcular_energia_cinetica() + self.calcular_energia_potencial(sol)

    def draw(self, screen, center_x, center_y):
        x = int(center_x + self.body.position.x * DRAW_SCALE)
        y = int(center_y + self.body.position.y * DRAW_SCALE)

        pygame.draw.circle(screen, self.color, (x, y), self.radio_px)

    def draw_label(self, screen, font, center_x, center_y):
        x = int(center_x + self.body.position.x * DRAW_SCALE)
        y = int(center_y + self.body.position.y * DRAW_SCALE)

        text = font.render(self.nombre, True, (230, 230, 230))
        screen.blit(text, (x + 8, y + 8))


# =========================================================
# CREACIÓN DEL SISTEMA SOLAR
# =========================================================
def crear_sistema_solar(space):
    sol = CuerpoCeleste(
        space=space,
        nombre="Sol",
        masa=1.98847e30,
        x=0,
        y=0,
        vx=0,
        vy=0,
        radio_px=10,
        color=(255, 220, 80), 
        es_estatico=True
    )

    planetas = [
        CuerpoCeleste(space, "Mercurio", 3.3011e23,   57.9e9,   0, 0, 47870, 2, (170, 170, 170)),
        CuerpoCeleste(space, "Venus",    4.8675e24,  108.2e9,   0, 0, 35020, 3, (220, 180, 80)),
        CuerpoCeleste(space, "Tierra",   5.97237e24, 149.6e9,   0, 0, 29780, 3, (100, 149, 237)),
        CuerpoCeleste(space, "Marte",    6.4171e23,  227.9e9,   0, 0, 24077, 2, (210, 100, 70)),
        CuerpoCeleste(space, "Júpiter",  1.8982e27,  778.5e9,   0, 0, 13070, 5, (210, 170, 120)),
        CuerpoCeleste(space, "Saturno",  5.6834e26, 1433.5e9,   0, 0,  9680, 4, (220, 200, 130)),
        CuerpoCeleste(space, "Urano",    8.6810e25, 2872.5e9,   0, 0,  6800, 4, (150, 220, 220)),
        CuerpoCeleste(space, "Neptuno",  1.02413e26,4495.1e9,   0, 0,  5430, 4, (90, 120, 255)),
    ]

    return sol, planetas


# =========================================================
# HUD
# =========================================================
def draw_hud(screen, font, planetas, sol):
    lines = [
        "Sistema solar 2D en Pymunk",
        "ESC para salir"
    ]

    y = 20

    for line in lines:
        text = font.render(line, True, (255, 255, 255))
        screen.blit(text, (20, y))
        y += 28

    # Velocidad del Sol
    vx_sol = sol.body.velocity.x
    vy_sol = sol.body.velocity.y
    v_sol = math.sqrt(vx_sol**2 + vy_sol**2)

    texto_sol = font.render(
        f"Sol: vx={vx_sol:.3e} m/s | vy={vy_sol:.3e} m/s | |v|={v_sol:.3e} m/s",
        True,
        (255, 255, 0)
    )
    screen.blit(texto_sol, (20, y))
    y += 36

    # Energías de los planetas
    for planeta in planetas:
        ec = planeta.calcular_energia_cinetica()
        ep = planeta.calcular_energia_potencial(sol)
        em = ec + ep

        texto = font.render(
            f"{planeta.nombre}: Ec={ec:.3e} J | Ep={ep:.3e} J | Em={em:.3e} J",
            True,
            (255, 255, 255)
        )
        screen.blit(texto, (20, y))
        y += 28

# =========================================================
# MAIN
# =========================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Plantilla Sistema Solar 2D - Pymunk")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)

    center_x = WIDTH // 2
    center_y = HEIGHT // 2

    space = pymunk.Space()
    space.gravity = (0, 0)

    sol, planetas = crear_sistema_solar(space)

    running = True
    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        # Aplicar gravedad del Sol sobre cada planeta
        for planeta in planetas:
            planeta.aplicar_gravedad_del_sol(sol)

        # Avanzar simulación
        space.step(DT)

        # Dibujado
        screen.fill(BACKGROUND)

        sol.draw(screen, center_x, center_y)
        sol.draw_label(screen, font, center_x, center_y)

        for planeta in planetas:
            planeta.draw(screen, center_x, center_y)
            planeta.draw_label(screen, font, center_x, center_y)

        draw_hud(screen, font, planetas, sol)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()