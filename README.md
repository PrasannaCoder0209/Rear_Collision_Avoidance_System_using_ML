# Rear_Collision_Avoidance_System_using_ML
This repository contains the code for a **Rear Collision Avoidance System** that combines computer vision with hardware alerts. The system uses a camera and the YOLOv8 object detection model to identify potential threats and a connected Arduino to provide real-time alerts.
-----
### Key Features
  * **Object Detection:** Utilizes the YOLOv8 model to detect different objects, such as vehicles, people, animals, and barriers.
  * **Audio Alerts:** Plays specific audio alerts for different detected objects.
  * **Hardware Integration:** Communicates with an Arduino, which controls a buzzer and an LCD display to provide a secondary layer of alerts.
  * **Ultrasonic Sensor:** The Arduino code includes logic for an ultrasonic sensor to measure distance, providing a separate alert mechanism for proximity.
  * **Visual Interface:** Displays a live camera feed with object detection bounding boxes, along with system information and timestamps.
-----
### File Descriptions
  * `app.py`: The main Python application that handles video capture, YOLOv8 object detection, audio alerts, and serial communication with the Arduino.
  * `al.py`: A Python script used to generate the audio alert `.mp3` files using the `gTTS` (Google Text-to-Speech) library.
  * `ad.txt`: The Arduino sketch that runs on the hardware. It reads data from an ultrasonic sensor, controls an LCD screen, and activates a buzzer based on commands received from the Python application via serial communication.
  * `yolov8n.pt`: The pre-trained YOLOv8 model file used for object detection.
  * `*.mp3` files: Audio files (`alert.mp3`, `vehicle_alert.mp3`, `person_alert.mp3`, `animal_alert.mp3`, `barrier_alert.mp3`) used by the Python application for audible alerts.
-----
### Getting Started
#### Prerequisites
  * Python 3.x
  * Arduino IDE
  * A physical Arduino board (e.g., Uno) and components (ultrasonic sensor, buzzer, I2C LCD)
  * Required Python libraries: `ultralytics`, `opencv-python`, `pyserial`, `pygame`, `numpy`, and `gTTS`.
You can install the necessary Python libraries by running:
```bash
pip install ultralytics opencv-python pyserial pygame numpy gTTS
```
#### Setup and Usage
1.  **Arduino Setup:**
      * Open `ad.txt` in the Arduino IDE.
      * Connect the ultrasonic sensor, buzzer, and I2C LCD to your Arduino board as defined in the code.
      * Upload the sketch to your Arduino.
2.  **Python Setup:**
      * Ensure all necessary Python libraries are installed.
      * Place all the files (`app.py`, `al.py`, `yolov8n.pt`, and all `.mp3` files) in the same directory.
      * You can run `al.py` to regenerate the alert audio files if needed.
3.  **Running the System:**
      * Connect the Arduino to your computer via a USB cable.
      * Make sure you know the correct COM port for your Arduino. You may need to edit the `ser = serial.Serial('COMX', 9600)` line in `app.py` to match your port (e.g., `'COM3'` on Windows or `'/dev/ttyACM0'` on Linux).
    * Run the main application from your terminal:
    <!-- end list -->
    ```bash
    python app.py
    ```
4.  **How it Works:**
      * The `app.py` script will start the video feed from your camera.
      * It will analyze each frame for objects using the YOLOv8 model.
      * If an object is detected, the system will play the corresponding audio alert.
      * If an object is detected at a distance, `app.py` will send a command to the Arduino via the serial port.
      * The Arduino will receive the command and activate the buzzer and update the LCD display.
