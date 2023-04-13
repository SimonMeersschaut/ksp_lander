import math

r = 70000 + 72
phi = -0.0957306 #Lattitude
theta = -74.5534983 #Longitude


z = r*math.sin(theta)*math.cos(phi) #diepte
x = r*math.sin(theta)*math.sin(phi) #breedte
y = r*math.cos(theta)               #hoogte

print(x, y, z)