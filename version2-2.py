import math
import numpy as np
from PIL import Image

# Definir constantes
WIDTH = 800  # Ancho de la imagen
HEIGHT = 600  # Alto de la imagen
FOV = math.pi / 3  # Ángulo de campo de visión de la cámara
MAX_DEPTH = 4  # Profundidad máxima de recursión para los rayos
BACKGROUND_COLOR = (0.2, 0.7, 0.8)  # Color del fondo

# Definir clases

class Vector:
    # Clase para representar un vector tridimensional
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __neg__(self):
        # Negación unaria
        return Vector(-self.x, -self.y, -self.z)

    def __add__(self, other):
        # Suma de vectores
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        # Resta de vectores
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        # Producto por un escalar
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar):
        # División por un escalar
        return Vector(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other):
        # Producto escalar
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        # Producto vectorial
        return Vector(self.y * other.z - self.z * other.y,
                      self.z * other.x - self.x * other.z,
                      self.x * other.y - self.y * other.x)

    def norm(self):
        # Norma del vector
        return math.sqrt(self.dot(self))

    def normalize(self):
        # Normalización del vector
        return self / self.norm()


class Ray:
    # Clase para representar un rayo de luz
    def __init__(self, origin, direction):
        self.origin = origin  # Punto de origen del rayo
        self.direction = direction.normalize()  # Dirección del rayo (normalizada)


class Sphere:
    # Clase para representar una esfera
    def __init__(self, center, radius, color, specular=0.5, reflection=0.5):
        self.center = center  # Centro de la esfera
        self.radius = radius  # Radio de la esfera
        self.color = color  # Color de la esfera
        self.specular = specular  # Factor de especularidad (0 a 1)
        self.reflection = reflection  # Factor de reflexión (0 a 1)

    def intersect(self, ray):
        # Función para calcular la intersección del rayo con la esfera
        oc = ray.origin - self.center
        a = ray.direction.dot(ray.direction)
        b = 2 * oc.dot(ray.direction)
        c = oc.dot(oc) - self.radius ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            return None
        else:
            t1 = (-b - math.sqrt(discriminant)) / (2 * a)
            t2 = (-b + math.sqrt(discriminant)) / (2 * a)
            if t1 > 0:
                return t1
            elif t2 > 0:
                return t2
            else:
                return None


class Plane:
    # Clase para representar un plano
    def __init__(self, position, normal, color, specular=0.3, reflection=0.2):
        self.position = position  # Punto en el plano
        self.normal = normal.normalize()  # Vector normal al plano (normalizado)
        self.color = color  # Color del plano
        self.specular = specular  # Factor de especularidad (0 a 1)
        self.reflection = reflection  # Factor de reflexión (0 a 1)

    def intersect(self, ray):
        # Función para calcular la intersección del rayo con el plano
        denominator = ray.direction.dot(self.normal)
        if abs(denominator) > 1e-6:
            t = (self.position - ray.origin).dot(self.normal) / denominator
            if t > 0:
                return t
        return None


class Light:
    # Clase para representar una fuente de luz
    def __init__(self, position, intensity):
        self.position = position  # Posición de la luz
        self.intensity = intensity  # Intensidad de la luz


# Definir funciones

def clamp(x, min_value, max_value):
    # Función para limitar un valor entre un mínimo y un máximo
    return np.clip(x, min_value, max_value)


def cast_ray(ray, objects, lights, depth):
    if depth > MAX_DEPTH:
        return BACKGROUND_COLOR

    closest_t = math.inf
    closest_object = None
    t = math.inf
    for obj in objects:
        t = obj.intersect(ray)
        if t and t < closest_t:
            closest_t = t
            closest_object = obj

    if closest_object is None:
        return BACKGROUND_COLOR

    point = ray.origin + ray.direction * closest_t

    if isinstance(closest_object, Sphere):
        normal = (point - closest_object.center).normalize()
    elif isinstance(closest_object, Plane):
        normal = closest_object.normal

    view = -ray.direction
    color = np.array([0.0, 0.0, 0.0])

    for light in lights:
        light_dir = (light.position - point).normalize()
        light_dist = (light.position - point).norm()

        shadow_ray = Ray(point + normal * 1e-3, light_dir)
        shadow_object = None

        for obj in objects:
            t = obj.intersect(shadow_ray)
            if t and t < light_dist:
                shadow_object = obj
                break

        if shadow_object is None:
            diffuse = np.array(closest_object.color) * max(0, light_dir.dot(normal)) * light.intensity
            specular = np.array([1.0, 1.0, 1.0]) * max(0, -reflect(light_dir, normal).dot(view)) ** 50 * light.intensity
            color += diffuse + specular

    if isinstance(closest_object, Plane):
        reflection_ray = Ray(point + normal * 1e-3, reflect(ray.direction, normal))
        reflection_color = np.array(cast_ray(reflection_ray, objects, lights, depth + 1))
        if reflection_color is not None:  # Comprobar si se encuentra un color válido
            color += closest_object.reflection * reflection_color

    color = clamp(color, 0, 1)  # Asegurar que los valores están en el rango [0, 1]
    return tuple(color)



def reflect(vector, normal):
    # Función para calcular la reflexión de un vector respecto a una normal
    return vector - normal * 2 * vector.dot(normal)

def render(objects, lights):
    image = Image.new("RGB", (WIDTH, HEIGHT))
    pixels = image.load()

    for x in range(WIDTH):
        for y in range(HEIGHT):
            # Convertir coordenadas de píxel a coordenadas normalizadas en el rango [-1, 1]
            px = (2 * x - WIDTH) / HEIGHT
            py = 1 - 2 * y / HEIGHT

            # Calcular dirección del rayo en función de las coordenadas normalizadas
            ray_direction = Vector(px * math.tan(FOV / 2), py * math.tan(FOV / 2), -1).normalize()

            ray = Ray(Vector(0, 0, 0), ray_direction)
            pixel_color = cast_ray(ray, objects, lights, 0)

            # Convertir el color del píxel de flotante a entero en el rango [0, 255]
            r = int(clamp(pixel_color[0], 0, 1) * 255)
            g = int(clamp(pixel_color[1], 0, 1) * 255)
            b = int(clamp(pixel_color[2], 0, 1) * 255)

            pixels[x, y] = (r, g, b)

    image.show()

# Definir objetos y luces

sphere1 = Sphere(Vector(-1, 0, -5), 1, (1, 0, 0), specular=0.7, reflection=0.5)
sphere2 = Sphere(Vector(1, 0, -7), 2, (0, 1, 0), specular=0.9, reflection=0.3)
plane = Plane(Vector(0, -3, 0), Vector(0, 1, 0), (0.5, 0.5, 0.5), specular=0.3, reflection=0.2)
light = Light(Vector(5, 5, -10), intensity=0.8)

# Definir lista de objetos y luces
objects = [sphere1, sphere2,plane]
lights = [light]

# Renderizar la escena
render(objects, lights)


