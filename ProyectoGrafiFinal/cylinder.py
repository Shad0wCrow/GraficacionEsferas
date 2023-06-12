import math
from PIL import Image
from vector import Vector

class Cylinder:
    def __init__(self, center, radius, height, texture_path, specular=0.5, reflection=0.5):
        self.center = center
        self.radius = radius
        self.height = height
        self.specular = specular
        self.reflection = reflection
        self.texture = Image.open(texture_path)

    def intersect(self, ray):
        oc = ray.origin - self.center
        a = ray.direction.x**2 + ray.direction.z**2
        b = 2 * (oc.x * ray.direction.x + oc.z * ray.direction.z)
        c = oc.x**2 + oc.z**2 - self.radius**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None

        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)

        y1 = ray.origin.y + t1 * ray.direction.y
        y2 = ray.origin.y + t2 * ray.direction.y

        if t1 > 0 and self.center.y < y1 < self.center.y + self.height:
            return t1
        elif t2 > 0 and self.center.y < y2 < self.center.y + self.height:
            return t2
        else:
            return None

    def get_normal(self, point):
        direction = (point - self.center).normalize()
        normal = Vector(direction.x, 0, direction.z)
        return normal

    def get_surface_color(self, point):
        distance = math.sqrt((point.x - self.center.x) ** 2 + (point.z - self.center.z) ** 2)

        if distance <= self.radius:
            y_diff = point.y - self.center.y
            y_normalized = y_diff / self.height

            if y_normalized > 1:
                y_normalized = 1
            elif y_normalized < -1:
                y_normalized = -1

            phi = math.acos(-y_normalized)
        else:
            phi = 0.0

        theta = math.atan2(-(point.z - self.center.z), point.x - self.center.x)
        u = (theta + math.pi) / (2 * math.pi)
        v = phi / math.pi

        tex_width, tex_height = self.texture.size
        pixel_x = int(u * tex_width) % tex_width
        pixel_y = int(v * tex_height) % tex_height

        tex_color = self.texture.getpixel((pixel_x, pixel_y))
        surface_color = (tex_color[0] / 255, tex_color[1] / 255, tex_color[2] / 255)

        return surface_color


