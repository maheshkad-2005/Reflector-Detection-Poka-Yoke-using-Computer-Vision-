import time
import cv2
from ultralytics import YOLO
import serial
import keyboard

# Load the YOLOv8 moel
model = YOLO('best.pt')
try:
    ser =serial.Serial('COM5', 9600, timeout=1)
    print("Serial port opened successfully.")

except serial.SerialException as e:
    print(f"Serial port error: {e}")        

def signal(commands):
    
    def send_command(command):
        ser.write(command.encode())
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"Arduino response: {response}")

    time.sleep(0.1)
    user_input = commands
    if user_input.lower() == 'q':
        print("hello")
    elif user_input == "1":
        send_command(user_input)
    elif user_input == "2":
        send_command(user_input)  
    elif user_input == "3":
        send_command(user_input)
    elif user_input == "4":
        send_command(user_input)

    
   

    

    

total_reflectors = [0, 0]  # List to store reflector counts per camera
front = 0
rear = 0 
midpoint=0
# Open the video files for multiple cameras
video_paths = [0, 1]  # Add paths for additional cameras if needed
caps = [cv2.VideoCapture(path) for path in video_paths]

# Check if the video files were successfully opened
for i, cap in enumerate(caps):
    if not cap.isOpened():
        print(f"Error: Unable to open video source for camera {i}")
        exit()

# Initialize variables
detected_ids = set()
skip_frames = 0

# Initialize serial connection to Arduino


# Define the coordinates of the predefined area (ROI)
roi_top_left = (1, 70)
roi_bottom_right = (550, 550)

# Loop through the video frames of all cameras
while True:
    data = ser.readline().decode().strip()
    print("My Serial Data",data)
    if "sensor" in data:
        midpoint=0 
        front=0 
        print("Reset")

    
    for i, cap in enumerate(caps):
        reflector_count = 0  # Initialize reflector count per frame

        # Read a frame from the video
        success, frame = cap.read()

        # Check if frame reading was successful
        if not success:
            print(f"Error: Failed to read frame from camera {i}")
            break

        # If frames need to be skipped, decrement skip_frames and continue
        if skip_frames > 0:
            skip_frames -= 1
            continue

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True ,conf=0.82)

        # Extract IDs from the tracked boxes if availab
        if "result" =="result":
        

            # Initialize a list to store the IDs of boxes within the ROI555

            # Loop through the detected boxes
            for box in results[0].boxes.xyxy:
                # Extract coordinates of the box
                x1, y1, x2, y2 = box

                # Check if the box is within the ROI
                if x1 >= roi_top_left[0] and y1 >= roi_top_left[1] and x2 <= roi_bottom_right[0] and y2 <= roi_bottom_right[1]:
                    # Add the ID to the list
                    reflector_count += 1
                    time.sleep(1)
            
            # Print the IDs within the ROI
        total_reflectors[i] = reflector_count

            # Add the detected IDs within the ROI to the set
        total_detected = sum(total_reflectors)
        print(total_detected)
        print(front)
        if total_detected == 2 and front !=3 :
            front = 1
            print ("Front reflectors are detected")
            signal("1")
            time.sleep(7)
            front = 3
        if total_detected ==1 and front != 3:
            print("one reflector front ")
            signal("2")
            time.sleep(0.2)
        if front == 3 and total_detected==0:
            midpoint = 1

        if midpoint ==1:
            if total_detected == 2:
                rear = 1          
                front=0
                print ("rear reflectors are detected")
                signal("3")

                time.sleep(7)
                midpoint=0  
            if total_detected == 1:
                print("one reflector rear")
                signal("4")
                time.sleep(0.2)


            

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        # Draw ROI on the frame
        cv2.rectangle(annotated_frame, roi_top_left, roi_bottom_right, (0, 255, 0), 2)
        # Display the annotated frame
        cv2.imshow(f"Camera {i} - YOLOv8 Tracking", annotated_frame)

       
        
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the video capture objects and close the display windows
for cap in caps:
    cap.release()
cv2.destroyAllWindows()

# Close serial connection to Arduino
