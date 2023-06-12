import math
from PIL import Image


class Sphere:
    def __init__(self, center, radius, texture_path, specular=0.5, reflection=0.5):
        self.center = center
        self.radius = radius
        self.specular = specular
        self.reflection = reflection
        self.texture = Image.open(texture_path)

    def intersect(self, ray):
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

    def get_surface_color(self, point):
        phi = math.acos(-(point.y - self.center.y) / self.radius)
        theta = math.atan2(-(point.z - self.center.z), point.x - self.center.x)
        u = (theta + math.pi) / (2 * math.pi)
        v = phi / math.pi

        tex_width, tex_height = self.texture.size
        pixel_x = int(u * tex_width) % tex_width
        pixel_y = int(v * tex_height) % tex_height

        tex_color = self.texture.getpixel((pixel_x, pixel_y))
        surface_color = (tex_color[0] / 255,
                         tex_color[1] / 255, tex_color[2] / 255)

        return surface_color
