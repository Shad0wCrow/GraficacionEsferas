# Importar librerías
import math
import numpy as np
from PIL import Image
#importacion interfaz

import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication,QTextEdit


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
    def __init__(self, center, radius, color):
        self.center = center  # Centro de la esfera
        self.radius = radius  # Radio de la esfera
        self.color = color  # Color de la esfera

    def intersect(self, ray):
        # Función para calcular la intersección del rayo con la esfera
        # Vector desde el centro de la esfera al origen del rayo
        oc = ray.origin - self.center
        a = ray.direction.dot(ray.direction)  # Coeficiente cuadrático
        b = 2 * oc.dot(ray.direction)  # Coeficiente lineal
        c = oc.dot(oc) - self.radius ** 2  # Coeficiente constante
        discriminant = b ** 2 - 4 * a * c  # Discriminante de la ecuación cuadrática
        if discriminant < 0:
            return None  # No hay solución real (no hay intersección)
        else:
            t1 = (-b - math.sqrt(discriminant)) / (2 * a)  # Primera solución
            t2 = (-b + math.sqrt(discriminant)) / (2 * a)  # Segunda solución
            if t1 > 0:
                # La primera solución es positiva (el punto de intersección está adelante del rayo)
                return t1
            elif t2 > 0:
                # La segunda solución es positiva (el punto de intersección está detrás del rayo)
                return t2
            else:
                # Ambas soluciones son negativas (no hay intersección)
                return None


class Light:
    # Clase para representar una fuente de luz
    def __init__(self, position, intensity):
        self.position = position  # Posición de la luz
        self.intensity = intensity  # Intensidad de la luz

# Definir funciones


def clamp(x, min_value, max_value):
    # Función para limitar un valor entre un mínimo y un máximo
    return max(min(x, max_value), min_value)


def cast_ray(ray, spheres, lights, depth):
    # Función para lanzar un rayo y calcular el color resultante
    if depth > MAX_DEPTH:
        return BACKGROUND_COLOR  # Se alcanzó la profundidad máxima de recursión
    closest_t = math.inf  # Distancia mínima al punto de intersección más cercano
    closest_sphere = None  # Esfera más cercana al rayo
    for sphere in spheres:
        # Calcular la intersección del rayo con la esfera
        t = sphere.intersect(ray)
        if t and t < closest_t:
            closest_t = t  # Actualizar la distancia mínima
            closest_sphere = sphere  # Actualizar la esfera más cercana
    if closest_sphere is None:
        return BACKGROUND_COLOR  # No hay intersección con ninguna esfera
    else:
        # Punto de intersección más cercano
        point = ray.origin + ray.direction * closest_t
        # Vector normal a la superficie de la esfera en el punto de intersección
        normal = (point - closest_sphere.center).normalize()
        view = -ray.direction  # Vector desde el punto de intersección al observador
        color = np.array([0.0, 0.0, 0.0])  # Color inicial del píxel
        for light in lights:
            # Vector desde el punto de intersección a la luz
            light_dir = (light.position - point).normalize()
            # Distancia desde el punto de intersección a la luz
            light_dist = (light.position - point).norm()
            # Rayo desde el punto de intersección hacia la luz (con un pequeño desplazamiento para evitar autointersecciones)
            shadow_ray = Ray(point + normal * 1e-3, light_dir)
            shadow_sphere = None  # Esfera que hace sombra al punto de intersección
            for sphere in spheres:
                # Calcular la intersección del rayo de sombra con la esfera
                t = sphere.intersect(shadow_ray)
                if t and t < light_dist:
                    shadow_sphere = sphere  # Encontrar la esfera más cercana que bloquea la luz
                    break
            if shadow_sphere is None:
                # No hay sombra en el punto de intersección
                # Componente difusa de la iluminación (modelo de Lambert)
                diffuse = np.array(closest_sphere.color) * \
                    max(0, light_dir.dot(normal)) * light.intensity
                # Componente especular de la iluminación (modelo de Phong)
                specular = np.array(
                    [1.0, 1.0, 1.0]) * max(0, -reflect(light_dir, normal).dot(view)) ** 50 * light.intensity
                # Sumar las componentes difusa y especular al color del píxel
                color += diffuse + specular
        # Rayo reflejado desde el punto de intersección (con un pequeño desplazamiento para evitar autointersecciones)
        reflection_ray = Ray(point + normal * 1e-3,
                             reflect(ray.direction, normal))
        # Color resultante del rayo reflejado (llamada recursiva)
        reflection_color = np.array(
            cast_ray(reflection_ray, spheres, lights, depth + 1))
        # Sumar el color reflejado al color del píxel (con un factor de atenuación)
        color += 0.2 * reflection_color
        # Devolver el color del píxel limitado entre 0 y 1
        return tuple(clamp(c, 0, 1) for c in color)


def reflect(direction, normal):
    # Función para calcular el vector reflejado a partir de un vector incidente y un vector normal
    return direction - normal * 2 * direction.dot(normal)


def render(spheres, lights):
    # Función para renderizar una escena compuesta por esferas y luces
    # Crear una imagen vacía
    image = Image.new("RGB", (WIDTH, HEIGHT))
    pixels = image.load()  # Obtener los píxeles de la imagen
    # Recorrer los píxeles de la imagen
    for i in range(WIDTH):
        for j in range(HEIGHT):
            # Calcular el vector dirección del rayo desde la cámara al píxel
            x = (2 * (i + 0.5) / WIDTH - 1) * \
                math.tan(FOV / 2) * WIDTH / HEIGHT
            y = -(2 * (j + 0.5) / HEIGHT - 1) * math.tan(FOV / 2)
            z = -1
            direction = Vector(x, y, z)
            # Crear el rayo desde la cámara al píxel
            ray = Ray(Vector(0, 0, 0), direction)
            # Calcular el color resultante del rayo
            color = cast_ray(ray, spheres, lights, 0)
            # Asignar el color al píxel
            pixels[i, j] = tuple(int(c * 255) for c in color)
    # Guardar la imagen
    image.save("primeraVersion.png")
    # Mostrar la imagen
    image.show()


spheres = [ Sphere(Vector(-1, 0, -4) , 1 , (0.6, 0.7, 0.8)),  # Esfera azulada a la izquierda
        Sphere(Vector(1.5, -0.5, -6), 2, (0.9, 0.8, 0.2))]  # Esfera amarillenta a la derecha
lights = [Light(Vector(10, 10, 10), 1.5)]  # Luz blanca arriba a la derecha

render(spheres, lights)
