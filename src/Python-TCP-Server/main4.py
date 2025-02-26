from flask import Flask, render_template, jsonify, request, redirect, url_for
from flask_socketio import SocketIO, emit
import socket, threading, struct, datetime, requests
# import sys, re

app = Flask(__name__)
socketio = SocketIO(app)

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 48569

# Communication protocol
BUFFER_SIZE_HELLO_MSG = 50  # 1 + 49 = 50 bytes
MESSAGE_TYPE_HELLO = 0
BUFFER_SIZE_SENSORS_UPDATE = 25 # 1 + 4*6 = 25 bytes
MESSAGE_TYPE_SENSORS_UPDATE = 1
MESSAGE_TYPE_WARNING_TEMP = 5
MESSAGE_TYPE_WARNING_HUM = 6
MESSAGE_TYPE_WARNING_LIGHT = 7
# BUFFER_SIZE_TIMER_UPDATE 5 # 1 + 4 = 5 bytes
MESSAGE_TYPE_TIMER_UPDATE = 2
BUFFER_SIZE_THRESHOLD_UPDATE = 13 # 1 + 4*3 = 13 bytes
MESSAGE_TYPE_THRESHOLD_UPDATE = 3
BUFFER_SIZE_REQUEST_UPDATE = 50  # 1 + 49 = 50 bytes
MESSAGE_TYPE_SENSORS_REQUEST = 4

conn = None


def pack_hello_message(message):
    """Packs a HelloMessage into bytes."""
    message_bytes = message.encode('utf-8')
    truncated_message_bytes = message_bytes[:BUFFER_SIZE_HELLO_MSG]  # Truncate the message
    format_string = "<B{}s".format(len(truncated_message_bytes))
    return struct.pack(format_string, MESSAGE_TYPE_HELLO, truncated_message_bytes)

def pack_timer_update_message(timer_value):
    """Packs a TimerUpdateMessage into bytes."""
    return struct.pack("<BI", MESSAGE_TYPE_TIMER_UPDATE, timer_value)  # I = uint32_t

def pack_threshold_update_message(temperature, humidity, light):
    """Packs a ThresholdUpdateMessage into bytes."""
    return struct.pack("<Bfff", MESSAGE_TYPE_THRESHOLD_UPDATE, temperature, humidity, light)

def pack_update_request(message):
    message_bytes = message.encode('utf-8')
    truncated_message_bytes = message_bytes[:BUFFER_SIZE_REQUEST_UPDATE]  # Truncate the message
    format_string = "<B{}s".format(len(truncated_message_bytes))
    return struct.pack(format_string, MESSAGE_TYPE_SENSORS_REQUEST, truncated_message_bytes)

def send_message(conn, message_type, data):
    """Sends a message to the IOT01 board."""
    if message_type == MESSAGE_TYPE_HELLO:
        packed_message = pack_hello_message(data)  # data is the string message
    elif message_type == MESSAGE_TYPE_TIMER_UPDATE:
        packed_message = pack_timer_update_message(int(data))  # data is the timer value
    elif message_type == MESSAGE_TYPE_THRESHOLD_UPDATE:
        packed_message = pack_threshold_update_message(data[0], data[1], data[2]) #data is the threshold
    elif message_type == MESSAGE_TYPE_SENSORS_REQUEST:
        packed_message = pack_update_request(data)  # data is the message
    else:
        print("Invalid message type")
        return
    print(f"Sending message: {packed_message.hex()}")

    conn.sendall(packed_message)


# Sensor variables initialization
temperature = None
humidity = None
light = None
last_message = "No message received yet."  # for the general message
timestamp = "No timestamp yet."
alarmMessage = None

# Global thresholds initialization
temperature_threshold = None
humidity_threshold = None
light_threshold = None

def tcp_server():
    """Creates a TCP server to receive data and updates sensor data."""
    global temperature, humidity, light, last_message, timestamp, last_message, temperature_threshold, humidity_threshold, light_threshold, conn, alarmMessage

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server_socket.bind((HOST, PORT))
    except socket.error as e:
        print(f"Bind failed. Error: {e}")
        return

    server_socket.listen(1)
    print(f"TCP Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()  # Store the connection here
    print(f"Connected by {addr}")

    while True:
        try:
            # Expecting: uint8_t, float, float, float  (1 + 4 + 4 + 4 = 13 bytes)
            data = conn.recv(BUFFER_SIZE_SENSORS_UPDATE)  # Adjust buffer size to match the struct size
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            if not data:
                print("Client disconnected.")
                break
            if len(data) != BUFFER_SIZE_SENSORS_UPDATE: #Verify if you received all the data
                print("Incomplete data received.  Skipping...")
                continue

            try:
                # Unpack the received data, assuming network byte order
                unpacked_data = struct.unpack("<Bffffff", data)  # Little-endian order

                if unpacked_data[0] == MESSAGE_TYPE_SENSORS_UPDATE or unpacked_data[0] == 5 or unpacked_data[0] == 6 or unpacked_data[0] == 7:
                    message_type, temperature_val, humidity_val, light_val, temperature_threshold, humidity_threshold, light_threshold = unpacked_data
                
                    temperature = str(round(temperature_val, 2))  # Round to 2 decimal places
                    humidity = str(round(humidity_val, 2))        # Round to 2 decimal places
                    light = str(round(light_val, 2))          # Round to 2 decimal places
                    
                    if(message_type == 5):
                        alarmMessage = "WARNING! THE TEMPERATURE IS TOO HIGH"
                    elif(message_type == 6):
                        alarmMessage = "WARNING! THE HUMIDITY IS TOO HIGH"
                    elif(message_type == 7):
                        alarmMessage = "WARNING! THE LIGHT IS TOO HIGH - CLOSING THE SHUTTERS"
                    else:
                        alarmMessage = "Every value is OK"

                    last_message = f"Received at {timestamp}: Type: {message_type}, Temp: {temperature}, Hum: {humidity}, Light: {light}, Temp Threshold: {temperature_threshold}, Hum Threshold: {humidity_threshold}, Light Threshold: {light_threshold}"  # For general Message
                

                    """ if message_type == 1:
                        socketio.emit('show_hidden_text')
                    elif message_type == 2:
                        socketio.emit('hide_hidden_text') """

            except struct.error as e:
                print(f"Struct unpack error: {e}")
                print(f"Received data: {data.hex()}") # Print the raw bytes to debug.
                continue  # Skip to the next iteration if unpacking fails.
        except ConnectionResetError:
            print("Client disconnected unexpectedly.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

    conn.close()
    server_socket.close()
    print("TCP Server closed.")

@app.route("/", methods=['GET', 'POST'])
def index():
    """Renders the main page with sensor data and handles button clicks."""
    global timestamp, conn

    if request.method == 'POST':
        if 'update_now' in request.form:
            if conn:  # Check if there's a valid connection
                try:
                    send_message(conn, MESSAGE_TYPE_SENSORS_REQUEST, "Update Now")
                except Exception as e:
                        print(f"Error sending data: {e}")
            else:
                print("No connection to send 'Hello' message!")
        elif 'set_up_timer' in request.form:
            return redirect(url_for('timer_setup'))
        elif 'set_up_thresholds' in request.form:
            return redirect(url_for('threshold_setup'))
        elif 'send_hello' in request.form:  # to mantain the function
            if conn:  # Check if there's a valid connection
                try:
                    hello_message = "Hello from Web!"
                    # conn.sendall(hello_message.encode('utf-8'))
                    send_message(conn, MESSAGE_TYPE_HELLO, hello_message)
                    # print(f"Sent: {hello_message}")
                except Exception as e:
                    print(f"Error sending data: {e}")
            else:
                print("No connection to send 'Hello' message!")

    return render_template(
        'index.html',
        temperature=temperature,
        humidity=humidity,
        light=light,
        timestamp=timestamp,
        message=last_message,
        alarm=alarmMessage
    )


@app.route("/api/sensors")
def get_sensors_data():
    """API endpoint to get sensor data as JSON."""
    global temperature, humidity, light, timestamp, last_message, alarmMessage
    return jsonify(temperature=temperature, humidity=humidity, light=light, timestamp=timestamp, message=last_message, alarm=alarmMessage)


@app.route("/timer_setup", methods=['GET', 'POST'])
def timer_setup():
    """Renders the timer setup page and handles saving the timer value."""
    if request.method == 'POST':
        if 'save_timer' in request.form:
            timer_value = request.form['timer_value']
            if conn:
                try:
                    send_message(conn, MESSAGE_TYPE_TIMER_UPDATE, timer_value)
                except Exception as e:
                    print(f"Error sending timer setup message: {e}")
            return redirect(url_for('index'))  # Return to the main page
        elif 'cancel_timer' in request.form:
            return redirect(url_for('index'))

    return render_template('timer_setup.html')


@app.route("/threshold_setup", methods=['GET', 'POST'])
def threshold_setup():
    """Renders the threshold setup page and handles saving the threshold values."""
    global humidity_threshold, temperature_threshold, light_threshold

    if request.method == 'POST':
        if 'save_thresholds' in request.form:
            try:
                humidity_threshold = float(request.form['humidity'])
                temperature_threshold = float(request.form['temperature'])
                light_threshold = float(request.form['light'])
                if conn:
                    send_message(conn, MESSAGE_TYPE_THRESHOLD_UPDATE, (temperature_threshold, humidity_threshold, light_threshold))
            except ValueError:
                print("Invalid threshold values.")
            return redirect(url_for('index'))
        elif 'cancel_thresholds' in request.form:
            return redirect(url_for('index'))

    return render_template(
        'threshold_setup.html',
        humidity_threshold=humidity_threshold,
        temperature_threshold=temperature_threshold,
        light_threshold=light_threshold
    )


""" @app.route('/show_hidden_text')
def show_hidden_text():
    return jsonify({'show': True})

@app.route('/hide_hidden_text')
def hide_hidden_text():
    return jsonify({'hide': True}) """


if __name__ == "__main__":
    # Start the TCP server in a separate thread
    tcp_thread = threading.Thread(target=tcp_server)
    tcp_thread.daemon = True
    tcp_thread.start()

    # Start the Flask app
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)