import math
import numpy as np
from PIL import Image
from vector import Vector
from ray import Ray
from sphere import Sphere
from plane import Plane
from light import Light
from cylinder import Cylinder

#importacion para la interfaz
import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication,QTextEdit

# Definir constantes
WIDTH = 800  # Ancho de la imagen
HEIGHT = 600  # Alto de la imagen
FOV = math.pi / 3  # Ángulo de campo de visión de la cámara
MAX_DEPTH = 4  # Profundidad máxima de recursión para los rayos
BACKGROUND_COLOR = (0.2, 0.7, 0.8)  # Color del fondo

# Definir funciones

def clamp(x, min_value, max_value):
    # Función para limitar un valor entre un mínimo y un máximo
    return np.clip(x, min_value, max_value)

def reflect(vector, normal):
    # Función para calcular la reflexión de un vector respecto a una normal
    return vector - normal * 2 * vector.dot(normal)

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
        surface_color = closest_object.get_surface_color(point)
    elif isinstance(closest_object, Plane):
        normal = closest_object.normal
        surface_color = closest_object.color
    elif isinstance(closest_object, Cylinder):
        normal = closest_object.get_normal(point)
        surface_color = closest_object.get_surface_color(point)

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
                diffuse = np.array(surface_color) * max(0, light_dir.dot(normal)) * light.intensity
                specular = np.array([1.0, 1.0, 1.0]) * max(0, -reflect(light_dir, normal).dot(view)) ** 50 * light.intensity
                color += diffuse + specular

        reflection_ray = Ray(point + normal * 1e-3, reflect(ray.direction, normal))
        reflection_color = np.array(cast_ray(reflection_ray, objects, lights, depth + 1))
        if reflection_color is not None:
            color += closest_object.reflection * reflection_color

        color = clamp(color, 0, 1)
        return tuple(color)


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
            diffuse = np.array(surface_color) * max(0, light_dir.dot(normal)) * light.intensity
            specular = np.array([1.0, 1.0, 1.0]) * max(0, -reflect(light_dir, normal).dot(view)) ** 50 * light.intensity
            color += diffuse + specular

    if isinstance(closest_object, Plane):
        reflection_ray = Ray(point + normal * 1e-3, reflect(ray.direction, normal))
        reflection_color = np.array(cast_ray(reflection_ray, objects, lights, depth + 1))
        if reflection_color is not None:  # Comprobar si se encuentra un color válido
            color += closest_object.reflection * reflection_color

    color = clamp(color, 0, 1)  # Asegurar que los valores están en el rango [0, 1]
    return tuple(color)

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

def escogerTextura(valor):
    if valor == "Cuadros":
        textura = "C:/Users/mairon/Desktop/GraficacionEsferas/ProyectoGrafiFinal/textura1.jpg"
    elif valor == "Piedras":
        textura = "C:/Users/mairon/Desktop/GraficacionEsferas/ProyectoGrafiFinal/textura2.jpg"
    elif valor == "Abstract":
        textura = "C:/Users/mairon/Desktop/GraficacionEsferas/ProyectoGrafiFinal/textura3.jpg"
    else:
        textura = "C:/Users/mairon/Desktop/GraficacionEsferas/ProyectoGrafiFinal/textura4.jpg" 
    
    return textura

def mostrarImagen(instancia_ventana):
    #inicializacion esfera 1
    xEsferaUno = int(instancia_ventana.xEsfera1.toPlainText())
    yEsferaUno = int(instancia_ventana.yEsfera1.toPlainText())
    zEsferaUno = int(instancia_ventana.zEsfera1.toPlainText())
    rEsferaUno = int(instancia_ventana.radioE1.value())
    texturaE1 = escogerTextura(instancia_ventana.texturaE1.currentText())
    
    #inicializacion esfera 2
    xEsferaDos = int(instancia_ventana.xEsfera2.toPlainText())
    yEsferaDos = int(instancia_ventana.yEsfera2.toPlainText())
    zEsferaDos = int(instancia_ventana.zEsfera2.toPlainText())
    rEsferaDos = int(instancia_ventana.radioE2.value())
    texturaE2 = escogerTextura(instancia_ventana.texturaE2.currentText())
    
    #inicializacion cilindro 3
    xCilindro = int(instancia_ventana.xCilindro.toPlainText())
    yCilindro = int(instancia_ventana.yCilindro.toPlainText())
    zCilindro = int(instancia_ventana.zCilindro.toPlainText())
    rCilindro = int(instancia_ventana.radioCilindro.value())
    aCilindro = int(instancia_ventana.alturaCilindro.value())
    texturaC = escogerTextura(instancia_ventana.texturaC.currentText())
    
    #inicializacion plano 4
    xPlano1 = int(instancia_ventana.xPlano1.toPlainText())
    yPlano1 = int(instancia_ventana.yPlano1.toPlainText())
    zPlano1 = int(instancia_ventana.zPlano1.toPlainText())
    xPlano2 = int(instancia_ventana.xPlano2.toPlainText())
    yPlano2 = int(instancia_ventana.yPlano2.toPlainText())
    zPlano2 = int(instancia_ventana.zPlano2.toPlainText())

    # Creación de objetos y luces
    
    #texture1   # Ruta de la imagen de textura para la esfera
    #texture2 =   # Ruta de la imagen de textura para la esfera
    #texture3 =   # Ruta de la imagen de textura para la esfera
    #texture4 =  # Ruta de la imagen de textura para la esfera
    
    #sphere1 = Sphere(Vector(-2, 0, -5), 1, texture2, specular=0.9, reflection=0.8)
    #sphere2 = Sphere(Vector(2, 0, -5), 1, texture3, specular=0.9, reflection=0.8)
    #plane = Plane(Vector(0, -1, 0), Vector(0, 1, 0), (0.5, 0.5, 0.5), specular=0.5, reflection=0.4)
    #cylinder = Cylinder(Vector(0, -1, -4), 0.5, 2, texture4, specular=0.7, reflection=0.3)
    #light = Light(Vector(3, 2, 10), (1, 1, 1))
    
    sphere1 = Sphere(Vector(xEsferaUno, yEsferaUno, zEsferaUno), rEsferaUno, texturaE1, specular=0.9, reflection=0.8)
    sphere2 = Sphere(Vector(xEsferaDos, yEsferaDos, zEsferaDos), rEsferaDos, texturaE2, specular=0.9, reflection=0.8)
    plane = Plane(Vector(xPlano1, yPlano1, zPlano1), Vector(xPlano2, yPlano2, zPlano2), (0.5, 0.5, 0.5), specular=0.5, reflection=0.4)
    cylinder = Cylinder(Vector(xCilindro, yCilindro, zCilindro), rCilindro, aCilindro, texturaC, specular=0.7, reflection=0.3)
    light = Light(Vector(3, 2, 10), (1, 1, 1))
    # Definir lista de objetos y luces
    objects = [sphere1, sphere2, cylinder,plane]
    lights = [light]

    # Renderizar la escena
    render(objects, lights)



#Inicio codigo interfaz
class ventana(QMainWindow):
    def __init__(self) :
        super().__init__()
        uic.loadUi("InterfazFinal.ui",self)
        self.boton_renderizar.clicked.connect(self.fn_renderizar) #asignamos un metodo de accion al boton
        
    def fn_renderizar(self):
        #llamado a la renderizacion
        print("Renderizando....")
        mostrarImagen(self)


if __name__=='__main__':
    app = QApplication(sys.argv)
    GUI = ventana()
    GUI.show()
    sys.exit(app.exec_())