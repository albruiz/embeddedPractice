<!-- ABOUT THE PROJECT -->
## About The Project
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
