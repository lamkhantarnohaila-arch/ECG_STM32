import time
import serial
import matplotlib.pyplot as plt
from collections import deque

# =========================
# CONFIGURATION
# =========================
PORT = "COM4"           # <-- À MODIFIER selon ton PC
BAUDRATE = 115200
FS = 1000               # fréquence d'échantillonnage réelle (Hz)
DURATION = 3            # fenêtre affichée en secondes
REFRESH_S = 0.05        # rafraîchissement ~20 images/s

MAX_POINTS = FS * DURATION

# =========================
# UART
# =========================
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
except serial.SerialException as e:
    raise SystemExit(f"Impossible d'ouvrir {PORT} : {e}")

ser.reset_input_buffer()
print(f"Acquisition ECG démarrée sur {PORT} à {BAUDRATE} bauds, FS={FS} Hz")
print("Ferme la fenêtre pour arrêter.")

# =========================
# DONNÉES
# =========================
time_data = deque(maxlen=MAX_POINTS)
raw_data = deque(maxlen=MAX_POINTS)
filt_data = deque(maxlen=MAX_POINTS)

rx_buffer = b""
sample_offset = 0       # pour gérer le fait que l'index repart à 1
last_index = 0
nb_received = 0

# =========================
# FIGURE
# =========================
plt.ion()
fig, ax = plt.subplots(figsize=(12, 4))
line_raw, = ax.plot([], [], lw=0.8, color='lightgray', label='brut')
line_filt, = ax.plot([], [], lw=1.2, color='green', label='filtré')

ax.set_title("ECG en temps réel")
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Valeur ADC (0-4095)")
ax.set_xlim(0, DURATION)
ax.set_ylim(0, 4095)
ax.grid(True)
ax.legend(loc='upper right')

# =========================
# FONCTIONS
# =========================
def process_line(text):
    """Traite une ligne 'index;raw;filtered'. Retourne True si un point est ajouté."""
    global last_index, sample_offset, nb_received

    text = text.strip()
    if not text:
        return False

    try:
        parts = text.split(';')
        if len(parts) < 3:
            return False
        index = int(parts[0])
        raw = int(parts[1])
        filt = int(parts[2])
    except (ValueError, IndexError):
        return False

    # Vérifier que les valeurs sont plausibles
    if not (0 <= raw <= 4095):
        return False

    # Détection d'un nouveau cycle d'acquisition (index repart à 1)
    if index < last_index:
        sample_offset += last_index
        # Optionnel : vider les buffers pour repartir propre
        # time_data.clear(); raw_data.clear(); filt_data.clear()

    last_index = index
    t = (sample_offset + index) / FS

    time_data.append(t)
    raw_data.append(raw)
    filt_data.append(filt)
    nb_received += 1
    return True


def update_plot():
    if not time_data:
        return

    line_raw.set_data(time_data, raw_data)
    line_filt.set_data(time_data, filt_data)

    t = time_data[-1]
    if t > DURATION:
        ax.set_xlim(t - DURATION, t)
    else:
        ax.set_xlim(0, DURATION)

    ax.set_title(f"ECG temps réel — {nb_received} échantillons reçus")
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# =========================
# ACQUISITION
# =========================
last_refresh = time.time()

try:
    while plt.fignum_exists(fig.number):

        # Lecture non bloquante
        n = ser.in_waiting
        chunk = ser.read(n if n > 0 else 1)
        if chunk:
            rx_buffer += chunk
            *lines, rx_buffer = rx_buffer.split(b"\n")
            for raw in lines:
                process_line(raw.decode("utf-8", errors="ignore"))

        # Rafraîchissement périodique
        now = time.time()
        if now - last_refresh >= REFRESH_S:
            update_plot()
            last_refresh = now
        else:
            plt.pause(0.001)

except KeyboardInterrupt:
    print("\nArrêt demandé par l'utilisateur.")

finally:
    ser.close()
    plt.ioff()
    print(f"Acquisition terminée. {nb_received} échantillons reçus au total.")
    plt.show()