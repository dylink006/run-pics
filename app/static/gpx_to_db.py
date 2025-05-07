import sqlite3
import os
import gpxpy

# Path to SQLite database
DB_PATH = 'routes.db'

# Folder containing GPX files
GPX_FOLDER = './routes'

def init_db():
    """Create the routes table if it doesn't exist."""
    with sqlite3.connect(DB_PATH) as conn:
        c = conn.cursor()
        c.execute('''
            CREATE TABLE IF NOT EXISTS routes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT,
                distance_km REAL,
                city TEXT,
                latitude REAL,
                longitude REAL,
                shape TEXT,
                color TEXT,
                gpx_data BLOB
            )
        ''')
        conn.commit()

def parse_gpx_metadata(file_path):
    """Extract name, distance, lat/lon, GPX data—and auto-detect star vs heart."""
    with open(file_path, 'r') as f:
        gpx = gpxpy.parse(f)

    name = os.path.splitext(os.path.basename(file_path))[0]
    distance_km = gpx.length_2d() / 1000.0 if gpx.length_2d() else 0

    lat = lon = None
    for track in gpx.tracks:
        for segment in track.segments:
            if segment.points:
                lat = segment.points[0].latitude
                lon = segment.points[0].longitude
                break
        if lat is not None:
            break

    lower = name.lower()
    if 'heart' in lower:
        shape = 'heart'
        color = 'red'
    elif 'star' in lower:
        shape = 'star'
        color = 'yellow'
    else:
        shape = 'unknown'
        color = 'blue'

    return {
        'name': name,
        'distance_km': round(distance_km, 2),
        'city': 'Unknown',   # or infer via reverse geocoding
        'latitude': lat,
        'longitude': lon,
        'shape': shape,
        'color': color,
        'gpx_data': open(file_path, 'rb').read()
    }

def insert_route_to_db(metadata):
    """Upsert: if name exists update it, otherwise insert new."""
    with sqlite3.connect(DB_PATH) as conn:
        c = conn.cursor()
        # see if this route is already in the table
        c.execute('SELECT id FROM routes WHERE name = ?', (metadata['name'],))
        row = c.fetchone()
        if row:
            # update existing
            c.execute('''
                UPDATE routes
                SET distance_km = ?,
                    city        = ?,
                    latitude    = ?,
                    longitude   = ?,
                    shape       = ?,
                    color       = ?,
                    gpx_data    = ?
                WHERE id = ?
            ''', (
                metadata['distance_km'],
                metadata['city'],
                metadata['latitude'],
                metadata['longitude'],
                metadata['shape'],
                metadata['color'],
                metadata['gpx_data'],
                row[0]
            ))
        else:
            # insert new
            c.execute('''
                INSERT INTO routes
                (name, distance_km, city, latitude, longitude, shape, color, gpx_data)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                metadata['name'],
                metadata['distance_km'],
                metadata['city'],
                metadata['latitude'],
                metadata['longitude'],
                metadata['shape'],
                metadata['color'],
                metadata['gpx_data']
            ))
        conn.commit()

def main():
    init_db()
    if not os.path.exists(GPX_FOLDER):
        print(f"Error: Folder '{GPX_FOLDER}' not found.")
        return

    for file in os.listdir(GPX_FOLDER):
        if file.lower().endswith('.gpx'):
            file_path = os.path.join(GPX_FOLDER, file)
            print(f"Processing {file_path}…")
            meta = parse_gpx_metadata(file_path)
            insert_route_to_db(meta)

if __name__ == '__main__':
    main()
