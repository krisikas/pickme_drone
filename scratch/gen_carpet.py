from PIL import Image
import random
width, height = 512, 512
img = Image.new('RGB', (width, height))
pixels = img.load()
for y in range(height):
    for x in range(width):
        r = 100 + random.randint(-40, 40)
        g = 50 + random.randint(-20, 20)
        b = 50 + random.randint(-20, 20)
        pixels[x, y] = (r, g, b)
img.save('sim/worlds/materials/textures/carpet.png')
print("Generated sim/worlds/materials/textures/carpet.png")
