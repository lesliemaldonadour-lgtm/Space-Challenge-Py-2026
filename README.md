Cansat ArasySat - RAS / UCAP

Misión Principal: 
1.Registrar la posición GPS (longitud, latitud y altura). (check)
2.Transmisión de datos de telemetría o datos de sensores. (check)
3.Registrar como mínimo tres variables atmosféricas, magnéticas o inerciales. (check) 
------Variables a medir: Temperatura ambiental, presión atmosférica, porcentaje de humedad, aceleración escalar, orientacion, campo magnético.

Misión Secundaria: 
-Fusión de sensores / Filtro de Kalman
-Detección de fases de vuelo (Experimentos científicos o tecnológicos propios.)


Flujo de trabajo: 

Sensores → ESP32 → Procesamiento → LoRa → Estación tierra → Análisis

TELEMETRÍA

-Tranmisión en tiempo real.
-Registro en CSV de los datos que recibe el ESP32.
-Analisis y comparación posterior.

Componentes: enlace a Componentes.md

Codigos del proyecto: enlace a Codigos.md

Montaje del proyecto: enlace a Diseño del circuito.md / enlace a Diseño de la lata.md
