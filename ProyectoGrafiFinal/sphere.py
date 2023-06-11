import math
class Sphere:
    def __init__(self, center, radius, color, specular=0.5, reflection=0.5):
        self.center = center
        self.radius = radius
        self.color = color
        self.specular = specular
        self.reflection = reflection

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
