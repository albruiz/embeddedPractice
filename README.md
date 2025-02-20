<!-- ABOUT THE PROJECT -->
## About The Project
State variables:
- Humidity Level
- Light
- Temperature

Future state variables:
- Floor's pH
- Electrical conductivity

Monitoring:
- Based on default values:
	- Temperature:
		- Range: 22-28ºC
        - Action above the range --> Send a notification
	- Humidity
        - Range: 50/60%
		- Action -> When below watered automatically (Led ON, which led?)
	- Light -> white/blue -> 4000-6500K
		- Action -> When over threshold -> close slowly the shutters till the light's level is reached (Put some less below the threshold to avoid hysteresis)

- Based on user values -> Through WiFi User specifies the values.

Monitoring: Choose by the user --> 30s, 60s, 300s, 1800s

User can:
- Request current values
- Change threshold
- Show values of monitoring

## Functions

- int readTemperature(): Reads the current Temperature value and return it (Translate decimal to integer value)
- int readHumidity(): Read the current Temperature value and return it (Translate decimal percentage)
- int readLight(): Read the current light level value and return it

- void sendNotification(char message): Whenever the level of temperature is out of range, sends a notification to user's app

- void setValues(int temp, int humidity, int light): set the new threshold values by the user.

43.3 volts reference 409 steps
