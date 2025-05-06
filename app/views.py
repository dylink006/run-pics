from app import app, db, login_manager
from flask import render_template, redirect, url_for, flash, request, Response, abort
from flask_login import UserMixin, login_user, login_required, logout_user, current_user
from werkzeug.security import generate_password_hash, check_password_hash
import os
import sqlite3

auth_db_path = 'app/instance/app.db'
if not os.path.exists(auth_db_path):
    open(auth_db_path, 'w').close()

class User(UserMixin, db.Model):
    id = db.Column(db.Integer, primary_key=True)
    email = db.Column(db.String(150), unique=True, nullable=False)
    password = db.Column(db.String(150), nullable=False)

@login_manager.user_loader
def load_user(user_id):
    return User.query.get(int(user_id))

@app.route("/")
def index():
    routes_db_path = os.path.join(app.static_folder, 'routes.db')
    conn = sqlite3.connect(routes_db_path)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()
    cur.execute("SELECT id, name, distance_km FROM routes ORDER BY name")
    rows = cur.fetchall()
    conn.close()

    # Build list of route dicts for template
    routes = []
    for row in rows:
        routes.append({
            'id': row['id'],
            'name': row['name'].replace('_', ' ').title(),
            'distance_km': row['distance_km'],
            'image_name': f"{row['name']}.png",
            'gpx_url': url_for('get_gpx', route_id=row['id'])
        })

    if current_user.is_authenticated:
        return render_template('user/home.html', user=current_user, routes=routes)
    return render_template('public/home.html', routes=routes)

@app.route("/about")
def about():
    all_routes = Route.query.order_by(Route.name).all()
    routes_data = []
    for r in all_routes:
        routes_data.append({
            'id': r.id,
            'name': r.name.replace('_', ' ').title(),
            'distance_km': r.distance_km,
            'image_name': f"{r.name}.png"
        })
    if current_user.is_authenticated:
        return render_template("user/home.html", user=current_user, routes=routes_data)
    return render_template("public/home.html", routes=routes_data)

@app.route("/signup", methods=["GET", "POST"])
def signup():
    if request.method == "POST":
        email = request.form.get('email')
        password = request.form.get('password')
        hashed_password = generate_password_hash(password, method='pbkdf2:sha256')

        if User.query.filter_by(email=email).first():
            flash('Email already exists. Please log in.', 'warning')
            return redirect(url_for('login'))

        new_user = User(email=email, password=hashed_password)
        db.session.add(new_user)
        db.session.commit()
        flash('Account created! You can now log in.', 'success')
        return redirect(url_for('login'))

    return render_template("/signup.html")

@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        email = request.form.get('email')
        password = request.form.get('password')
        user = User.query.filter_by(email=email).first()

        if user and check_password_hash(user.password, password):
            login_user(user)
            flash('Welcome back!', 'success')
            return redirect(url_for('index'))
        else:
            flash('Invalid email or password', 'danger')

    return render_template("/login.html")

@app.route("/logout")
@login_required
def logout():
    logout_user()
    flash('You have been logged out.', 'info')
    return redirect(url_for('login'))

@app.route("/gpx/<int:route_id>")
def get_gpx(route_id):
    # connect to the same DB where you stored the blobs
    routes_db_path = os.path.join(app.static_folder, 'routes.db')
    conn = sqlite3.connect(routes_db_path)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()
    cur.execute("SELECT gpx_data FROM routes WHERE id = ?", (route_id,))
    row = cur.fetchone()
    conn.close()

    if row is None:
        abort(404)

    # row['gpx_data'] is a bytes BLOB
    return Response(row['gpx_data'], mimetype='application/gpx+xml')

with app.app_context():
    db.create_all()