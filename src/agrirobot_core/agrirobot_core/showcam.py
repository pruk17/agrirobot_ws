import cv2
from flask import Flask, Response, render_template_string
#flask => create a small web server to show camera feed
#Response => used to send response to client SSH
#render_template_string => used to render HTML template directly from a string
#cv2 => OpenCV library for computer vision tasks

#app => Flask application instance
app = Flask(__name__)
#cap => VideoCapture object to access the camera
cap = cv2.VideoCapture(0)#VideoCapture(0) => opens the default camera (0 is usually the default camera index

if not cap.isOpened():
    print("Cannot open camera")
    exit()

#generate_frames => function to capture frames from the camera and yield them as byte streams
def generate_frames():
    while True:
        #success => boolean indicating if the frame was captured successfully
        #frame => the captured frame from the camera (numpy array)
        success, frame = cap.read()
        frame = cv2.flip(frame, 1)  # Flip the frame horizontally for a mirror effect
        if not success:
            break
        else:
            #ret => boolean indicating if the frame was encoded successfully
            #buffer => byte buffer containing the encoded frame (bytes)
            #frame_bytes => byte representation of the frame for transmission 
            #yield => used to return a generator that produces frames for the video feed
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n' #-- frame boundary for multipart response
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

#@app.route => decorator to define a route for the web server
#@app.route('/') => defines the root route of the web server
@app.route('/')
def index():
    #render_template_string => renders a simple HTML template to display the camera feed
    return render_template_string('''
        <html>
            <head>
                <title>Live Camera</title>
            </head>
            <body>
                <h1>Live Camera Feed</h1>
                <img src="/video_feed" width="800" height="600">
            </body>
        </html>
    ''')
#@app.route('/video_feed') => defines the route for the video feed
@app.route('/video_feed')
def video_feed():
    #Response(generate_frames()) => returns a response that streams the video feed
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    #app.run => starts the Flask web server
    app.run(host='0.0.0.0', port=5000) 
