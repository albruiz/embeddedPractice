from flask import Flask, render_template, jsonify, request, redirect, url_for
import socket
import threading
import struct  # Import the struct module
import socket as socketlib
import sys # For checking byte order
import re

app = Flask(__name__)

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 48569

# Global variables to store sensor data
temperature = "N/A"
humidity = "N/A"
light = "N/A"
last_message = "No message received yet."  # for the general message
conn = None

# Global thresholds (initial values)
humidity_threshold = 40.0
temperature_threshold = 20.0
light_threshold = 100.0

def tcp_server():
    """Creates a TCP server to receive data and updates sensor data."""
    global temperature, humidity, light, last_message, conn

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
            data = conn.recv(100)  # Adjust buffer size to match the struct size

            if not data:
                print("Client disconnected.")
                break

            else:
                # Unpack the received data, assuming network byte order
                """ print(data,'-------------')
                unpacked_data = struct.unpack("!Bfff", data)  # Network byte order
                print(f"Unpacked data: {unpacked_data}")
                print(f"Received data (raw bytes): {data.hex()}") 

                message_type, temperature, humidity, light = unpacked_data"""


                data_str = data.decode('utf-8')  # or 'ascii' if you know it's only ASCII

                # Use regular expressions to extract the values
                match = re.match(r"Temperature: (\d+\.\d+), Humidity: (\d+\.\d+), Light: (\d+\.\d+)", data_str)

                temperature = float(match.group(1))
                humidity = float(match.group(2))
                light = float(match.group(3))
                print(f"Temp = {temperature}, Hum = {humidity}, Light = {light}")
                """ print(f"Received: Type = {message_type}, Temp = {temperature}, Hum = {humidity}, Light = {light}") """

                # Update global variables (converting floats to strings for display)
                temperature = str(temperature)
                humidity = str(humidity)
                light = str(light)
                #last_message = f"Type: {message_type}, Temp: {temperature}, Hum: {humidity}, Light: {light}"  # For general Message

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
    global conn

    if request.method == 'POST':
        if 'update_now' in request.form:
            # Future functionality for "Update Now" button
            print("Update Now button pressed.")
            pass  # Add your functionality here
        elif 'set_up_timer' in request.form:
            return redirect(url_for('timer_setup'))
        elif 'set_up_thresholds' in request.form:
            return redirect(url_for('threshold_setup'))
        elif 'send_hello' in request.form:  # to mantain the function
            if conn:  # Check if there's a valid connection
                try:
                    hello_message = "settings_timer_30"
                    conn.sendall(hello_message.encode('utf-8'))
                    print(f"Sent: {hello_message}")
                except Exception as e:
                    print(f"Error sending data: {e}")
            else:
                print("No connection to send 'Hello' message!")

    return render_template(
        'index.html',
        temperature=temperature,
        humidity=humidity,
        light=light,
        message=last_message
    )


@app.route("/api/sensors")
def get_sensors_data():
    """API endpoint to get sensor data as JSON."""
    global temperature, humidity, light
    return jsonify(temperature=temperature, humidity=humidity, light=light)


@app.route("/timer_setup", methods=['GET', 'POST'])
def timer_setup():
    """Renders the timer setup page and handles saving the timer value."""
    if request.method == 'POST':
        if 'save_timer' in request.form:
            timer_value = request.form['timer_value']
            if conn:
                try:
                    message = f"new timing set up: {timer_value}"
                    conn.sendall(message.encode('utf-8'))
                    print(f"Sent: {message}")
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
                    message = f"new thresholds set up: Humidity={humidity_threshold}, Temperature={temperature_threshold}, Light={light_threshold}"
                    conn.sendall(message.encode('utf-8'))
                    print(f"Sent: {message}")
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


if __name__ == "__main__":
    # Start the TCP server in a separate thread
    tcp_thread = threading.Thread(target=tcp_server)
    tcp_thread.daemon = True
    tcp_thread.start()

    # Start the Flask app
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)