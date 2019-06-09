from flask import Flask, request,send_from_directory,send_file
from flask_restful import Resource, Api
import flask


app = Flask(__name__)


api = Api(app)


todos = {}


@app.route("/")
def hello():
    return "Hello World!"

@app.route("/512",methods=["GET"])
def send512():
    return send_file("a512")

@app.route("/1024",methods=["GET"])
def send1024():
    return send_file("a1024")

@app.route("/4096",methods=["GET"])
def send4096():
    return send_file("a4096")

@app.route("/8192",methods=["GET"])
def send8192():
    return send_file("a8192")

@app.route("/send",methods=["POST"])
def rec():
    img_file = request.files.get('file')

    print(img_file.filename)
    return "ok "+img_file.filename



if __name__ == '__main__':
    app.run(host="0.0.0.0")
    

