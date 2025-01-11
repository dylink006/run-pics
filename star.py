import folium
from math import sin, cos, radians

# Define the coordinates for Gainesville, Florida
latitude, longitude = 29.6516, -82.3248

# Function to calculate star points
def generate_star_coordinates(lat, lon, size, points=5):
    coords = []
    for i in range(2 * points):
        angle = radians(360 * i / (2 * points))
        r = size if i % 2 == 0 else size / 2  # Alternate between long and short points
        x = lat + r * cos(angle) * 0.1  # Adjust scale for latitude
        y = lon + r * sin(angle) * 0.1  # Adjust scale for longitude
        coords.append((x, y))
    coords.append(coords[0])  # Close the star
    return coords

# Generate larger star coordinates
star_coords = generate_star_coordinates(latitude, longitude, size=0.05)  # Increase size

# Create a map centered on Gainesville
m = folium.Map(location=[latitude, longitude], zoom_start=12)

# Add a red-outlined star
folium.PolyLine(star_coords, color="red", weight=3).add_to(m)

# Save map to an HTML file
m.save("gainesville_large_star.html")
print("Map saved as 'gainesville_large_star.html'. Open this file to view the map.")
