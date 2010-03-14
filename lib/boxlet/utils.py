import math
import pyglet

def clamp(x, min_x, max_x):
    return min(max(x, min_x), max_x)

def float_to_ubyte(f):
    return clamp(int(f * 256.0), 0, 255)

def float_to_byte(f):
    return clamp(int(math.floor(f * 128.0)), -128, 127)

def char_to_float(c):
    assert isinstance(c, str) and len(c) == 1
    return float(ord(c)) / 255.0

def get_box_vertices(half_width=1.0, half_height=1.0):
    return [(-half_width, -half_height), (half_width, -half_height),
            (half_width, half_height), (-half_width, half_height)]

def get_circle_vertices(center=(0.0, 0.0), radius=1.0, angle=0.0, count=8):
    vertices = []
    x, y = center
    for i in xrange(count):
        a = angle + 2.0 * math.pi * float(i) / float(count)
        vertex = x + radius * math.cos(a), y + radius * math.sin(a)
        vertices.append(vertex)
    return vertices

def get_circle_triangles(center=(0.0, 0.0), radius=1.0, angle=0.0, count=8):
    vertices = get_circle_vertices(center, radius, angle, count)
    triangles = []
    for i in xrange(count):
        j = (i + 1) % count
        triangle = center, vertices[i], vertices[j]
        triangles.append(triangle)
    return triangles

def get_polygon_triangles(vertices):
    centroid = get_polygon_centroid(vertices)
    triangles = []
    for i in xrange(len(vertices)):
        j = (i + 1) % len(vertices)
        triangle = centroid, vertices[i], vertices[j]
        triangles.append(triangle)
    return triangles

# TODO: calculate actual centroid
def get_polygon_centroid(vertices):
    total_x = sum(x for x, y in vertices)
    total_y = sum(y for x, y in vertices)
    return total_x / float(len(vertices)), total_y / float(len(vertices))

def get_vertex_normals(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    n1 = 0.0, 0.0, 1.0
    n2 = normalize((x2 - x1, y2 - y1, 0.0))
    n3 = normalize((x3 - x1, y3 - y1, 0.0))
    return [n1, n2, n3]

def get_face_normal(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x = (x2 + x3) / 2.0
    y = (y2 + y3) / 2.0
    nx, ny = x - x1, y - y1
    nz = abs((nx + ny) / 2.0)
    return normalize((nx, ny, nz))

def normalize(v):
    x, y, z = v
    if x or y or z:
        length = math.sqrt(x * x + y * y + z * z)
        assert length
        return x / length, y / length, z / length
    else:
        return 0.0, 0.0, 0.0

# TODO: linear interpolation for float coordinates
def sample_image(image_data, x, y):
    assert isinstance(image_data, pyglet.image.ImageData)
    assert image_data.format in ('RGB', 'RGBA')
    i = int(y) * image_data.pitch + int(x) * len(image_data.format)
    pixel = image_data.data[i:i + len(image_data.format)]
    return tuple(map(char_to_float, pixel))
