import cv2
import time
import depthai as dai
from djitellopy import Tello
import subprocess
import socket
import jetson.inference
import jetson.utils
################################################################################
# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(640, 480)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)

# Connect to device and start pipeline
device = dai.Device(pipeline)
# Output queue will be used to get the rgb frames from the output defined above
qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
################################################################################
tello = None
frame_read = None
################################################################################
def get_jetson_camframe():
    inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
    
    # Retrieve 'bgr' (opencv format) frame
    frame = inRgb.getCvFrame()
    # Convert to RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #cv2.imwrite("jetson.jpeg", frame)
    return frame

def get_tello_camframe():
    frame = frame_read.frame
    cv2.imwrite("telloframe.jpg", frame)
    time.sleep(0.1)  # Delay to get different frames
    return frame

def connect_tello():
    global tello
    global frame_read
    tello = Tello()
    tello.connect()
    print(f"Battery: {tello.get_battery()}%")
    
    # Start video stream
    tello.streamon()
    frame_read = tello.get_frame_read()
    # Wait a moment to ensure stream starts
    time.sleep(4)
    #tello.streamoff()

def connect_tello_wifi(ssid: str):
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:
        command = f"./drone_connect.sh {ssid}"
        # Run the command
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        print("Connection output:")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("Failed to connect to Wi-Fi:")
        print(e.stderr)

def move_turtle(x, z, ip='192.168.50.129', port=5025):
    message = f"{x},{z}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (ip, port))
    print(f"Sent command: {message} to {ip}:{port}")

# Initialize Jetson's detection model
detectnet = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

########connect_tello_wifi("TELLO-5C28D8")
#######connect_tello()
for i in range(5):
    frame = get_jetson_camframe()
    #frame = get_tello_camframe()
    print("got frame")
    print(frame.shape)
    cuda_frame = jetson.utils.cudaFromNumpy(frame)
    detections = detectnet.Detect(cuda_frame)
    # Print detections
    for d in detections:
        print(f"{detectnet.GetClassDesc(d.ClassID)} @ {d.Left},{d.Top} to {d.Right},{d.Bottom} (confidence={d.Confidence:.2f})")
    # Filter for "person" class
    person_detections = [d for d in detections if detectnet.GetClassDesc(d.ClassID) == "person"]
    if person_detections:
        # Find the largest one by area
        largest = max(person_detections, key=lambda d: (d.Right - d.Left) * (d.Bottom - d.Top))

        # Compute the center of the bounding box
        center_x = (largest.Left + largest.Right) / 2
        center_y = (largest.Top + largest.Bottom) / 2
        area = (largest.Right - largest.Left) * (largest.Bottom - largest.Top)
        print(f"Largest person: Center=({center_x:.1f}, {center_y:.1f}), Area={area:.1f}, Confidence={largest.Confidence:.2f}")
        
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        print(f"frame >>>> {frame_width} x {frame_height}")
        norm_x = center_x / frame_width
        norm_y = center_y / frame_height
        z_movement = (0.5 - norm_x) * 2
        print(f" normx  {norm_x}, x_movement {z_movement}")
        move_turtle(0.3, z_movement)
    else:
        print("No person detected.")
        
    # Save annotated frame for review (optional)
    jetson.utils.saveImage(f"detected_{i}.jpg", cuda_frame)

