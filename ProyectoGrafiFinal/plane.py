import math
class Plane:
    def __init__(self, position, normal, color, specular=0.3, reflection=0.2):
        self.position = position
        self.normal = normal.normalize()
        self.color = color
        self.specular = specular
        self.reflection = reflection

    def intersect(self, ray):
        denominator = ray.direction.dot(self.normal)
        if abs(denominator) > 1e-6:
            t = (self.position - ray.origin).dot(self.normal) / denominator
            if t > 0:
                return t
        return None
