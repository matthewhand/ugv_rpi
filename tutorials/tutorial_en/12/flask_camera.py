from flask import Flask, render_template, Response, send_from_directory
import threading
import cv_ctrl
import os

app = Flask(__name__)

cvf = cv_ctrl.OpencvFuncs()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory('templates', filename)

@app.route('/settings/<path:filename>')
def serve_static_settings(filename):
    return send_from_directory('templates', filename)
        
if __name__ == '__main__':
    cam_thread = threading.Thread(target=cvf.frame_process, daemon=True)
    cam_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False)
