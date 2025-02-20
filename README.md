# Flower smart pot with Wi-Fi connection

Minimal specifications: 
1. The system shall control the status of the plant at all times (i.e., humidity level, light, temperature).
2. When the plant needs attention, the pot will send a message to the smartphone.
3. The user shall be able to set up a threshold for each of the parameters, so as to ask for attention.
4. When the humidity is below the threshold, the plant will be watered automatically. Use a LED so as to turn on or off this functionality.
5. If the light is over the threshold, the shutters of the room will be closed as much as indicated by the excess of light.
6. The sensing frequency of each of the magnitudes shall be edited by the user through the smartphone (30 seconds, 1 minute, 5 minutes or 30 minutes).
7. At any time, the user can ask for a new measurement of the parameters through the smartphone.
8. The smartphone will show the values of the last measurement available.

## Implementation
State variables:
- Humidity Level
- Light
- Temperature

Monitoring:
- Based on default values:
	- Temperature:
		- Range: Definido y por default
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
MQTT
%cambiartemperature46%
%46% -> Set values

Álvar -> Luz
Carlos -> Humedad y temperatura
Jairo -> Interrupciones
Uti --> WiFi chuparla

- int readTemperature(): Reads the current Temperature value and return it (Translate decimal to integer value)
- int readHumidity(): Read the current Temperature value and return it (Translate decimal percentage)
- int readLight(): Read the current light level value and return it

- Interrupciones -> 2 interrupciones, ciclo de lectura y peticion usuario
	- Para leer valores cuando acabe el timer
	- Para leer valores cuando el usuario lo pida 
	- Para cambiar thresholds cuando el usario lo pida
	- Para cambiar el timer cuando el usuario lo pida
	- Cuando alguno de los parametros supera el threshold al leer el timer
   		- Temperature = Notificacion
		- Humidity = Encender led
    		- Luz = bajar persiana hasta que este por debajo del valor threshold
        		- Imprimir "Bajando persiana" y leer valor luz; Si luz>threshold -> seguir bajando; else -> parar persiana y volver a low-battery mode

- void sendNotification(char message): Whenever the level of temperature is out of range, sends a notification to user's app

- void setValues(int temp, int humidity, int light): set the new threshold values by the user.
- array getValues();

43.3 volts reference 409 steps
