import pygame
import pymunk
import pymunk.pygame_util



# Configuración de la ventana
WIDTH, HEIGHT = 1000, 600
FPS = 60
NIVEL_DEL_SUELO=HEIGHT-60 #PIXELES 


#escala
PX_M = 1000
RADIO_M = 0.0213
RADIO_PX = RADIO_M * PX_M

X_HOYO_PX = 500
ANCHO_HOYO_M = 4 * RADIO_M
ANCHO_HOYO_PX = ANCHO_HOYO_M * PX_M

X_INICIO_HOYO = X_HOYO_PX - ANCHO_HOYO_PX / 2
X_FIN_HOYO = X_HOYO_PX + ANCHO_HOYO_PX / 2

def crearSueloAntes(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = (0, 0)

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
    body.position = (0, 0)

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
    body = pymunk.Body()
    body.position = posicion
    body.mass = 0.04593
    body.moment = pymunk.moment_for_circle(body.mass, 0, RADIO_PX)
    shape = pymunk.Circle(body, RADIO_PX)
    shape.elasticity = 0.7
    shape.friction = 0.7

    space.add(body, shape)
    return body, shape

def crear_palo(space, posicion):
    return True

def golpear_bola(body, fuerza):
	body.apply_impulse_at_local_point(fuerza)


def run():
	# 1. Inicializar Pygame
	pygame.init()
	screen = pygame.display.set_mode((WIDTH, HEIGHT))
	clock = pygame.time.Clock()
	draw_options = pymunk.pygame_util.DrawOptions(screen)

	# 2. Cargar y ajustar imagen de fondo
	# Cargamos la imagen y la escalamos para que cubra los 1000px de ancho
	try:
		bg_image = pygame.image.load("golf/calle_golf.png")
		bg_width, bg_height = bg_image.get_size()
		bg_width*=0.7
		bg_height*=0.6
		# Ajuste proporcional: el ancho es 1000, calculamos el alto correspondiente
		aspect_ratio = bg_height / bg_width
		new_height = int(WIDTH * aspect_ratio)
		bg_image = pygame.transform.scale(bg_image, (WIDTH, new_height))
		# Posición: pegado a la parte inferior
		bg_pos = (0, HEIGHT - new_height)
	except pygame.error:
		print("No se pudo cargar la imagen calle_golf.png. Se usará fondo negro.")
		bg_image = None

	# 3. Inicializar Pymunk (Espacio físico)
	space = pymunk.Space()
	space.gravity = (0, 900)  # Gravedad hacia abajo (ajusta el valor según tu escala)
	crearSueloAntes(space)
	crearSueloDespues(space)
	bola_body, bola_shape = crear_bola(space, (X_HOYO_PX, NIVEL_DEL_SUELO - RADIO_PX - 80) #Encima del hoyo
)

	# 4. Bucle principal de simulación
	running = True
	while running:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
			if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
				print("golpe")
				golpear_bola(bola_body, (1000, 0))

		# Limpiar pantalla
		screen.fill((135, 206, 235))  # Color cielo por defecto

		

		# Dibujar fondo si existe
		if bg_image:
			screen.blit(bg_image, bg_pos)

		# Paso de tiempo de la física (fijo para estabilidad)
		dt = 1.0 / FPS
		space.step(dt)

		# Dibujar debug de Pymunk (para ver los cuerpos físicos sobre el dibujo)
		# space.debug_draw(draw_options)


		radio=RADIO_M*PX_M*30
		pygame.draw.circle(screen, (255,255,255), (100,NIVEL_DEL_SUELO-radio), radio)
		
		pygame.draw.circle(screen, (255,255,255), (HOYO_M*PX_M,NIVEL_DEL_SUELO-radio), radio)
		

		pygame.display.flip()
		clock.tick(FPS)
		
		
		

	pygame.quit()

if __name__ == "__main__":
	run()
