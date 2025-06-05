import cv2
import time
import depthai as dai
from djitellopy import Tello
import subprocess
import socket
import jetson.inference
import jetson.utils
import threading
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

# Initialize Jetson's detection model
detectnet = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

detectnet_lock = threading.Lock()
################################################################################
tello = None
frame_read = None
bTakeoff = False
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

#x is forward speed, z > 0 turn left, z < 0 turn right, range 0-1
def move_turtle(x, z, ip='192.168.50.129', port=5025):
    message = f"{x},{z}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (ip, port))
    print(f"Sent command: {message} to {ip}:{port}")

# x forward distance in cm, z is angle
def move_tello(x, z):
    print(f">>move tello {x} {z}")
    global bTakeoff
    if not bTakeoff:
        tello.takeoff()
        time.sleep(3)
        tello.move_up(30)
        bTakeoff = True
    if x != 0.0:
        int_value = int(x)
        if int_value > 0:
            tello.move_forward(int_value)
        else:
            tello.move_back(abs(int_value))
    if z != 0.0:
        int_value = int(z)
        if int_value > 0:
            tello.rotate_counter_clockwise(int_value)
        else:
            tello.rotate_clockwise(abs(int_value))

#Use jetson Net to detect human and get the result
def get_detectNet_result(frame):
    cuda_frame = jetson.utils.cudaFromNumpy(frame)
    with detectnet_lock:
        detections = detectnet.Detect(cuda_frame)
        #jetson.utils.saveImage(f"detected_.jpg", cuda_frame)
    
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
        print(f" norm_x  {norm_x}, z_movement {z_movement}")
        hori_parti = (largest.Right - largest.Left) / frame_width
        #move_turtle(0.3, z_movement)
        
        return norm_x, z_movement, hori_parti
    else:
        print("No person detected.")
        return None

###############################################################################
##y is forward distance, x > 0 is right forward distance, x < 0 is left forward distance
destination_x = -1.0 #left in meter
destination_y = 2.0 #forward in meter
dest_x_cm = -1.0
dest_y_cm = 200

def drone_move_onestep():
    global destination_x
    global destination_y
    
    if destination_y != 0.0:
        move_tello(destination_y*100, 0)
        destination_y = 0.0
        return False
    elif destination_x != 0.0:
        if destination_x > 0:
            move_tello(0, -90.0)
        elif destination_x < 0:
            move_tello(0, 90.0)
        move_tello(abs(destination_x*100), 0)
        destination_x = 0.0    
        return True
    else:
        return True

#move turtle to the predefined destinaiton
def turtle_move_onestep():
    global dest_x_cm
    global dest_y_cm
    if dest_y_cm != 0.0:
        for i in range(6):
            move_turtle(1.0, 0)
            dest_y_cm = dest_y_cm - 20 if dest_y_cm > 20 else 0
            print(f">>>>>>>>{dest_y_cm}")
            time.sleep(0.5)
            if dest_y_cm == 0:
                break
        return False
    elif dest_x_cm != 0.0:
        if dest_x_cm > 0:
            move_turtle(0, -0.8)
            time.sleep(0.5)
            move_turtle(0, -0.8)
            time.sleep(0.5)
            move_turtle(0, -0.8)
            time.sleep(0.5)
            move_turtle(0, -0.8)
            time.sleep(0.5)
        elif dest_x_cm < 0:
            move_turtle(0, 0.8)
            time.sleep(0.5)
            move_turtle(0, 0.8)
            time.sleep(0.5)
            move_turtle(0, 0.8)
            time.sleep(0.5)
            move_turtle(0, 0.8)
            time.sleep(0.5)
        dest_y_cm = dest_x_cm
        dest_x_cm = 0
        return False
    else:
        return True

def drone_follow_onestep():
    frame = get_tello_camframe()
    results = get_detectNet_result(frame)
    if results is not None:
        norm_x, z_movement, hori_parti = results
        print(f">>>>norm_x {norm_x}   z_movement {z_movement} hori_parti {hori_parti}")
        if hori_parti > 0.69: #move backward
            move_tello(-30, 0)
            ####################
            #tello.land()
            #break
        elif hori_parti < 0.6: #move forward
            if hori_parti < 0.3:
                move_tello(80, 0)
            elif hori_parti < 0.4:
                move_tello(40, 0)
            else:
                move_tello(20, 0)
        if z_movement > 0.5: #counter_clockwise
            move_tello(0, 20)
        elif z_movement < -0.25:
            move_tello(0, -20)

def turtle_follow_onestep():
    frame = get_jetson_camframe()
    print("got frame turtle")
    cuda_frame = jetson.utils.cudaFromNumpy(frame)
    with detectnet_lock:
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
        print(f"################{largest.Right - largest.Left}")
        if largest.Right - largest.Left < 400:
            move_turtle(0.3, z_movement)
    else:
        print("No person detected.")

def drone_loop():
    while True:
        print(">> Drone Loop")
        if drone_move_onestep():
            drone_follow_onestep()
        time.sleep(0.1)  # Optional, to reduce CPU usage

# Control loop for the turtlebot
def turtle_loop():
    while True:
        print(">> Turtle Loop")
        if turtle_move_onestep():
            print(">> Turtle Loop follow")
            turtle_follow_onestep()
            turtle_follow_onestep()
            turtle_follow_onestep()
            turtle_follow_onestep()
        time.sleep(0.1)  # Optional, to reduce CPU usage

connect_tello_wifi("TELLO-5C28D8")
connect_tello()

t1 = threading.Thread(target=drone_loop)
t2 = threading.Thread(target=turtle_loop)

t1.start()
t2.start()    
