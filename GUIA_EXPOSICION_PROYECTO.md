# Guía de Exposición Sencilla — Péndulo de Furuta

> **Objetivo de esta guía:** Que puedas explicar el proyecto con total confianza, usando palabras claras, ejemplos sencillos del mundo real y sin enredarte en tecnicismos innecesarios.

---

## 1. El Proyecto en 30 Segundos (Ficha Rápida)

* **¿Qué es?** Es un brazo que gira impulsado por un motor, y en su punta lleva una varilla libre. El objetivo es que el brazo balancee la varilla para **pararla de cabeza en el aire y mantenerla en equilibrio**, tal como balancearías una escoba en la palma de tu mano.
* **El Cerebro:** Tarjeta **STM32 Nucleo-F446RE** que calcula qué hacer **1.000 veces cada segundo** ($1\text{ kHz}$).
* **Los Ojos:** 2 sensores magnéticos ultra-precisos (**AS5600**) que miden el ángulo del motor y del péndulo sin rozar ni frenar nada.
* **El Músculo:** Un motor DC con caja de piñones (**reductora 80:1**) y un driver de potencia (**L298N**).
* **El Cuerpo:** Estructura completa diseñada y fabricada en **impresión 3D**.
* **El Resultado Real:** Se levanta solo en **menos de 1 segundo** ($0.98\text{ s}$), se queda quieto arriba por **más de 14 segundos continuos**, y si alguien le pega un golpe fuerte y se cae, **se frena, regresa al centro y se vuelve a parar solo**.

---

## 2. ¿Por qué es tan difícil? (Los 3 retos en cristiano)

Imagínate balancear un palo de escoba sobre la palma de tu mano. Es exactamente el mismo problema, pero con 3 complicaciones:

1. **Un solo motor para dos cosas (Subactuado):**  
   Solo tenemos el motor de la base. No hay ningún motor en la varilla del péndulo. Para que la varilla no se caiga, el motor tiene que adivinar cómo mover la base.
2. **Si te duermes una décima de segundo, se cae (Inestable):**  
   La gravedad no perdona. Si la varilla se inclina un poquito, en menos de $120\text{ milisegundos}$ ya está en el suelo. El procesador no puede tener pausas ni demoras.
3. **El efecto patineta (Fase no mínima):**  
   Si estás patinando o balanceando una escoba y quieres moverte hacia la derecha, **primero tienes que dar un pequeño paso hacia la izquierda** para que la escoba se incline a la derecha, y luego perseguirla. Si intentas correr directo a la derecha, la escoba se queda atrás y se cae. Nuestro controlador hace exactamente esa jugada inteligente.

---

## 3. Las 3 Fases del Sistema (¿Qué pasa al darle "Iniciar"?)

```text
[ En reposo colgado ] ──▶ 1. EL COLUMPIO (Swing-Up) ──▶ 2. EL EQUILIBRIO (Balance)
                               ▲                                  │
                               │                                  ▼
                               └── Si le pegan un golpe ◀── [ Auto-recuperación ]
```

### Fase 1: El Columpio (Swing-Up)
* **¿Qué hace?** La varilla empieza colgando quieta hacia abajo. El motor empieza a mover el brazo adelante y atrás, como quien empuja a un niño en un columpio justo a tiempo.
* **El truco inteligente:** A medida que la varilla sube y gana energía, el motor va haciendo los movimientos **más cortitos y suaves**. Así la varilla no llega como un proyectil, sino que frena justo al llegar a la cima ($180^\circ$) en apenas **$0.98\text{ segundos}$**.

### Fase 2: El Equilibrio Invertido (Balance)
* **¿Qué hace?** En cuanto la varilla llega arriba, el sistema cambia de chip al instante. El motor da empujoncitos milimétricos hacia adelante y hacia atrás para no dejarla caer, mientras regresa suavemente el brazo al centro ($0^\circ$).
* **Libertad total de giro:** Si alguien empuja el péndulo fuerte, el brazo puede dar las vueltas que necesite para salvar la varilla; no se frena ni se bloquea por límites artificiales.

### Fase 3: La Auto-recuperación (A prueba de caídas)
* **¿Qué hace?** Si alguien le mete un golpe tan duro que es físicamente imposible sostenerlo:
  1. El motor se apaga $300\text{ ms}$ para no forzar los engranajes.
  2. Regresa el brazo suavemente a $0^\circ$ usando un **freno electrónico**.
  3. Espera un cuarto de segundo a que la varilla deje de mecerse.
  4. Vuelve a iniciar el columpio automáticamente y se vuelve a parar de cabeza.

---

## 4. Los Secretos del Control (Explicados Fácil)

### A. ¿Por qué el péndulo NO tiene término Integral ($K_i = 0$)?
* **La trampa clásica:** En clase nos enseñan que la integral elimina el error de estado estacionario.
* **La realidad física:** Si tu mesa de trabajo tiene $0.5^\circ$ de inclinación, la vertical de la gravedad no coincide con el cero del sensor. Si el péndulo tuviera integral, intentaría llevar el sensor a $0^\circ$ acumulando voltaje. Pero el brazo también tiene integral intentando quedarse en $0^\circ$. Los dos integradores pelearían entre sí y el brazo saldría disparado girando sin control.
* **La solución:** El péndulo solo usa Proporcional y Derivativo ($PD$). Él encuentra solo la vertical de la gravedad real, y dejamos que el brazo use integral para centrarse.

### B. ¿Por qué los números del brazo son negativos?
* Por lo que explicamos del efecto escoba: para traer el brazo hacia adelante sin tumbar la varilla, primero el motor tiene que acelerar hacia atrás. El signo negativo le dice al motor: *"Primero inclina la varilla hacia donde quieres ir, y luego persíguela"*.

### C. ¿Cómo logramos que el motor no tiemble ni zumbe? (Los filtros)
Cuando conectas un sensor digital que lee 1.000 veces por segundo, el ruido matemático al calcular la velocidad hace que el motor vibre como una licuadora. Además, los piñones de la reductora tienen un pequeño juego entre dientes.
Para solucionarlo pusimos 3 filtros limpios:
1. **Filtro pasa-bajos:** Quita el ruido eléctrico y los saltos de lectura del sensor.
2. **Filtro trampa (Notch) a $11\text{ Hz}$:** Apaga exactamente la frecuencia a la que los piñones hacen ruido y vibran.
3. **Filtro adaptable (El secreto "Ultra-Silk"):**
   * **Cuando está quieto arriba:** El control es súper dócil y suave, el motor no vibra, casi no gasta corriente y no hace ningún ruido.
   * **Si alguien le da un golpe:** El sistema detecta el susto al instante, desconecta el filtro suave y le mete toda la fuerza y velocidad al motor para salvar el péndulo.

### D. La trampa del puente H L298N (Zona muerta)
* Los puentes H viejos como el L298N se tragan casi $2\text{ voltios}$ antes de que el motor empiece a girar.
* Si le metes una compensación de golpe (saltar de $0\%$ a $10\%$ de una), el motor pega tirones bruscos.
* Nosotros pusimos una **rampa suave progresiva**: el motor arranca como una seda sin sacudidas mecánicas.

---

## 5. Resultados del Proyecto (Los datos para convencer)

Todos estos números fueron medidos en el prototipo físico real, no en una simulación:

| ¿Qué se midió? | Resultado Real | ¿Qué significa en la práctica? |
| :--- | :---: | :--- |
| **Tiempo de levantamiento** | **$0.98\text{ s}$** | Sube en menos de 1 parpadeo. |
| **Tiempo de equilibrio continuo** | **$> 14\text{ s}$** | Se queda arriba de forma indefinida sin caerse. |
| **Desviación del péndulo** | **$< 2^\circ$** | Se mantiene prácticamente vertical a simple vista. |
| **Voltaje promedio en equilibrio** | **$0.50 - 0.75\text{ V}$** | Consume casi nada de energía cuando está en equilibrio. |
| **Precisión del modelo matemático** | **$98.51\%$** | La física del código coincide casi al 100% con la realidad. |
| **Retorno al centro** | **$0.02^\circ$** | Al frenar, el brazo queda clavado exactamente en el cero. |

---

## 6. Guion de Sustentación de 5 Minutos (Paso a Paso)

Usa esta estructura cuando te toque hablar. Está calculada para que no te sobre ni te falte tiempo:

### Minuto 1: Introducción y el Reto
> *"Buenos días. Nuestro proyecto es el control digital de un Péndulo Invertido de Furuta. Es uno de los problemas clásicos más exigentes en ingeniería de control porque con un solo motor en la base tenemos que balancear un péndulo arriba, el cual por naturaleza es totalmente inestable: si no se controla cada milisegundo, la gravedad lo tumba en menos de una décima de segundo."*

### Minuto 2: Hardware e Impresión 3D
> *"Para resolverlo construimos un prototipo físico completo. Diseñamos las piezas en FreeCAD y las imprimimos en 3D en PLA optimizadas sin soportes. Como cerebro utilizamos una STM32 Nucleo-F446RE que corre el lazo de control a 1.000 Hz exactos. Usamos dos encoders magnéticos de 12 bits que leen el ángulo sin contacto mecánico, y un motor DC con reductora 80:1 alimentado con un puente H L298N."*

### Minuto 3: Estrategia de Control (El cómo se hizo)
> *"El sistema opera en dos etapas automáticas:  
> Primero, el **Swing-Up**, que bombea energía como un columpio para llevar la varilla arriba en apenas 0.98 segundos modulando la carrera para llegar con velocidad suave.  
> Segundo, el **Equilibrio Dual**, donde un control PD sostiene el péndulo arriba mientras un control PID centra el brazo, teniendo en cuenta que el brazo tiene dinámica de fase no mínima. Además, diseñamos filtros Notch digitales para matar la vibración mecánica de los piñones y una rampa que compensa la caída de voltaje del driver."*

### Minuto 4: Demostración en Vivo (MATLAB)
> *(Abres la interfaz de MATLAB)*  
> *"Aquí en la interfaz vemos la telemetría en tiempo real y nuestro gemelo digital 3D.  
> Le damos al botón **Iniciar**: vemos cómo se mece y se estabiliza en menos de un segundo.  
> Ahora, si yo le doy un empujón suave, el sistema responde corrigiendo de inmediato.  
> Y si le doy un golpe muy fuerte y se cae, observen cómo el sistema no se vuelve loco: frena, regresa el brazo a cero, espera a que se calme y se vuelve a levantar solo."*

### Minuto 5: Conclusiones
> *"En conclusión, logramos una identificación del 98.5% de la planta real y un sistema que se mantiene en pie consumiendo menos de 0.75 V en promedio. Esto demuestra que con buen modelado matemático y filtrado digital determinista se pueden resolver no linealidades complejas de hardware real. Quedamos atentos a sus preguntas."*

---

## 7. Machete de Respuestas para Preguntas del Profesor (Q&A)

Si el profesor o jurado quiere probar tus conocimientos, casi seguro preguntará una de estas:

#### 1. "¿Por qué el péndulo no tiene control Integral?"
* **Respuesta corta:** *"Porque si el péndulo y el brazo tienen integral al mismo tiempo sobre un solo motor, cualquier milímetro de desnivel en la mesa hace que los dos integradores peleen entre sí y el brazo gire al infinito. Al dejar el péndulo solo con PD, él busca la vertical física real y la integral del brazo se encarga de centrarlo."*

#### 2. "¿Por qué las ganancias del brazo son negativas?"
* **Respuesta corta:** *"Por el efecto de fase no mínima. Para traer la base hacia el centro sin que la varilla se caiga, primero el motor tiene que acelerar brevemente en sentido contrario para inclinar la varilla hacia el objetivo, y luego perseguirla."*

#### 3. "¿Por qué muestrean a 1.000 Hz (1 ms) si el péndulo se mueve lento (1.4 Hz)?"
* **Respuesta corta:** *"Porque a 1.000 Hz el retardo de cálculo es imperceptible (0.5 ms), y nos permite aplicar filtros digitales Notch muy nítidos para matar vibraciones de 11 Hz de los piñones sin quitarle velocidad ni estabilidad al péndulo."*

#### 4. "¿Qué pasa si le meto un golpe muy fuerte al brazo?"
* **Respuesta corta:** *"Durante el equilibrio no le pusimos límite al ángulo del brazo para que pueda girar las vueltas que necesite esquivando el golpe. Y si el golpe ya supera los 32 grados de inclinación y es físicamente insalvable, la máquina de estados apaga el motor, regresa a casa con freno electromagnético y se levanta sola."*

#### 5. "¿Cómo compensaron que el motor no arranque con voltajes bajitos?"
* **Respuesta corta:** *"El driver L298N tiene una zona muerta de casi 2 V por sus transistores. En vez de ponerle un salto brusco que haría vibrar el motor, programamos una rampa suave continua que arranca el movimiento progresivamente sin sacudidas."*

#### 6. "¿Por qué los filtros están hechos en doble precisión (double 64-bit)?"
* **Respuesta corta:** *"Porque cuando tienes un filtro a 6.2 Hz o 11 Hz muestreando a 1.000 Hz, los polos del filtro quedan peligrosamente pegados al círculo unitario ($z \approx 0.96$). En precisión simple de 32 bits los redondeos matemáticos desestabilizan el filtro; con precisión de 64 bits es perfectamente exacto y la STM32 con Cortex-M4F lo ejecuta en microsegundos."*

---

## 8. Checklist de 1 Minuto Antes de Salir a Exponer

- [ ] STM32 conectada por cable USB (`COM13` verificado).
- [ ] Fuente de poder de $12\text{ V}$ encendida y conectada al driver L298N.
- [ ] Péndulo colgado verticalmente en reposo hacia abajo.
- [ ] MATLAB abierto con la aplicación iniciada en pantalla completa:
  ```matlab
  run('interfaz_matlab/app_telemetria_sistema_furuta_matlab.m')
  ```
- [ ] Pulsar botón **`CERO (0°)`** con el péndulo quieto.
- [ ] ¡Listos para pulsar **`▶ INICIAR`** y sorprender al jurado!
