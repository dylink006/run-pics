from app import app

from flask import render_template

@app.route("/")
def index():
    return render_template("/public/home-public.html")

@app.route("/about")
def about():
    return render_template("/about.html")