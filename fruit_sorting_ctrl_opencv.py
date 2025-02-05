# ============================================================================
# STRUTTURA PRINCIPALE DEL PROGRAMMA
# ============================================================================
"""
Questo programma implementa un sistema di controllo robotico con la seguente infrastruttura:

1. CONFIGURAZIONE BASE
   - Importazione librerie
   - Inizializzazione robot e nastro trasportatore
   - Configurazione parametri principali

2. GESTIONE FSA (Finite State Automaton)
   - Parsing configurazione stati
   - Gestione transizioni
   - Monitoraggio requisiti

3. GESTIONE DISPOSITIVI
   - Motori (UR5e e gripper)
   - Sensori (distanza e posizione)
   - Camera
   - Display
   - Audio

4. PIPELINE DI ELABORAZIONE
   - Rilevamento frutta
   - Gestione stati
   - Movimento robot
   - Aggiornamento display

5. MACCHINA A STATI
   - Stati principali del sistema
   - Substati per azioni robot
   - Gestione transizioni
"""

# ============================================================================
# 1. IMPORTAZIONE DELLE LIBRERIE NECESSARIE
# ============================================================================
import cv2  # OpenCV per l'elaborazione delle immagini
import numpy as np  # NumPy per operazioni numeriche e array
from controller import Supervisor  # Libreria Webots per controllare il robot
import math # Libreria per operazioni matematiche
import time # Libreria per gestione del tempo
import re  # Libreria per le espressioni regolari
import os  # Per gestire i file

# --- Inizializzazione Robot e Nastro Strasportatore ---

robot = Supervisor() # Creazione dell'istanza del supervisore
conveyor_belt = robot.getFromDef("conveyor_belt") # Ottiene il riferimento al nastro trasportatore dal mondo Webots

# Verifica se il nastro trasportatore è stato trovato correttamente
if conveyor_belt is None:
    print("Errore: Nodo 'conveyor_belt' non trovato nel file .wbt.")
else:
    # Ottiene il campo velocità del nastro
    speed_field = conveyor_belt.getField("speed")
    if speed_field is None:
        print("Errore: Campo 'speed' non trovato nel nodo 'conveyor_belt'.")

# Imposta la velocità iniziale del nastro (deve essere un float)
speed_field.setSFFloat(0.15) # Per fermare il nastro usa: speed_field.setSFFloat(0.0) 

# ============================================================================
# 2. CONFIGURAZIONE DEI PARAMETRI PRINCIPALI
# ============================================================================
# --- Parametri Base ---
timestep = 32 # Timestep di base per la simulazione (32 ms)
current_state = 1 # Stato corrente del sistema
numero_stati = 0 # Numeri stati completi
first_load = True # Stampa FSA

# --- Stati e Contatori ---
fruit = -1 # Variabile per il tipo di frutto (-1 = nessun frutto, 0 = arancia, 1 = mela, 2 = mela marcia)
fruit_names = ['Orange', 'Apple', 'Rottenapple'] # Dizionario dei nomi dei frutti per riferimento
apple, orange, rottenapple = 0, 0, 0# Contatori per i vari tipi di frutta
state_apple_count = 0    # Mele raccolte nello stato corrente
state_orange_count = 0   # Arance raccolte nello stato corrente

# --- Flag di Sistema ---
counter = 0
is_process_complete = False
main_state_changed = False
elapsed_time = 0

# --- Gestione Tempo ---
state_start_time = 0
state_delay_active = False
state_delay_end_time = 0

# --- Contatori Contenitori ---
counter_O1 = 0      # Contatore arance contenitore 1
counter_G1 = 0      # Contatore mele contenitore 1
counter_O2 = 0      # Contatore arance contenitore 2
counter_G2 = 0      # Contatore mele contenitore 2
counter_binblue = 0 # Contatore mele marce

# --- Definizioni di variabile dopo le altre dichiarazioni di variabile globale ---
bin_green1, bin_green2 = "bin_green1", "bin_green2"
bin_orange1, bin_orange2 = "bin_orange1", "bin_orange2"

# --- Dizionario per tenere traccia dei conteggi nei contenitori ---
state_bin_counts = {
    "bin_green1": 0,
    "bin_green2": 0,
    "bin_orange1": 0,
    "bin_orange2": 0
}

# ============================================================================
# 3. CONFIGURAZIONE DELLE POSIZIONI TARGET DEL ROBOT
# ============================================================================
# --- Posizioni Target ---
target_HALT = [0, -1.57, 0, -1.57, 0] # Posizione di riposo (HALT) del robot
target_positions = [                                        # Array delle posizioni target per i vari contenitori
    [-1.570796, -1.87972, -2.139774, -2.363176, -1.50971],  # O1 (posizione arancia 1)
    [0, -1.87972, -2.139774, -2.363176, -1.50971],          # G1 (posizione mela 1)
    [-1, -1.67972, +1.539774, -2.163176, -1.50971],         # B1 (posizione mela marcia)
    [0, -1.57, 0, -1.57, 0],                                # HALT
    [1.570796, -1.87972, -2.139774, -2.363176, -1.50971],   # O2 (posizione arancia 2)
    [1, -1.87972, -2.139774, -2.363176, -1.50971]           # G2 (posizione mela 2)
]

# --- Inizializzazione Dispositivi ---
speed = 2 # Velocità di base del robot UR5e

# Inizializzazione dei motori delle dita del gripper
hand_motors = []
hand_motors.append(robot.getDevice('finger_1_joint_1'))
hand_motors.append(robot.getDevice('finger_2_joint_1'))
hand_motors.append(robot.getDevice('finger_middle_joint_1'))

# Inizializzazione dei motori del braccio UR5e
ur_motors = []
ur_motors.append(robot.getDevice('shoulder_pan_joint'))
ur_motors.append(robot.getDevice('shoulder_lift_joint'))
ur_motors.append(robot.getDevice('elbow_joint'))
ur_motors.append(robot.getDevice('wrist_1_joint'))
ur_motors.append(robot.getDevice('wrist_2_joint'))

# Imposta la velocità di tutti i motori del braccio
for i in range(5):
    ur_motors[i].setVelocity(speed)

# Inizializzazione sensore di distanza del gripper
distance_sensor = robot.getDevice('distance sensor')
distance_sensor.enable(timestep)

# Inizializzazione sensore di posizione del polso
position_sensor = robot.getDevice('wrist_1_joint_sensor')
position_sensor.enable(timestep)

# Inizializzazione della telecamera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Inizializzazione del display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

# Inizializzazione speaker
speaker = robot.getDevice('speaker')

# ============================================================================
# 4. FUNZIONI DI UTILITÀ PER AUDIO E DISPLAY
# ============================================================================
def playSnd(track):
    """Riproduce il suono appropriato in base al tipo di frutto"""
    if track == 0: speaker.playSound(speaker, speaker, 'sounds/women/orange.wav', 1.0, 1.0, 0.0, False)
    elif track == 1: speaker.playSound(speaker, speaker, 'sounds/women/apple.wav', 1.0, 1.0, 0.0, False)
    elif track == 2: speaker.playSound(speaker, speaker, 'sounds/men/apple.wav', 1.0, 1.0, 0.0, False)

def resetDisplay():
    """Resetta il display cancellando tutto il contenuto"""
    display.setAlpha(0.0)
    display.fillRectangle(0, 0, 200, 150)
    display.setAlpha(1.0)

def printDisplay(x, y, w, h, name):
    """
    Disegna un rettangolo e un nome sul display
    
    Args:
        x, y (int): Coordinate del rettangolo
        w, h (int): Larghezza e altezza del rettangolo
        name (str): Nome da visualizzare
    """
    resetDisplay()
    display.drawRectangle(x, y, w, h)
    display.drawText(name, x - 2, y - 20)

def draw_info_panel():
    """
    Crea e aggiorna un pannello informativo sul display del robot
    che mostra vari contatori, stati del sistema e requisiti richiesti
    """
    # Ottiene il riferimento al display    
    info_display = robot.getDevice("info_display")

    if info_display is None:
        print("Errore: Display non trovato!")
        return

    # Configura lo sfondo del display
    info_display.setColor(0xFFFFFF)  # Sfondo bianco
    info_display.setAlpha(0.8)  # Leggera trasparenza
    info_display.fillRectangle(0, 0, 270, 180)
    
    info_display.setFont("Arial", 14, True)
    info_display.setColor(0x000000)  # Testo nero
    info_display.setAlpha(1.0)  # Testo completamente opaco

    # Ottiene i requisiti dello stato corrente
    current_requirements = FSA[current_state]["requirements"] if current_state in FSA else {}
    
    # Calcola i requisiti totali per mele e arance nello stato corrente
    required_apples = sum(count for bin_name, count in current_requirements.items() if bin_name.startswith("bin_green"))
    required_oranges = sum(count for bin_name, count in current_requirements.items() if bin_name.startswith("bin_orange"))

    # Calcola il delay rimanente
    if state_delay_active and not is_process_complete:
        remaining_delay = max(0, state_delay_end_time - robot.getTime())
    else:
        remaining_delay = 0

    # Mostra contatori generali con requisiti
    info_display.drawText(f"Apples: {apple:3d}    {state_apple_count}|{required_apples}", 10, 10)
    info_display.drawText(f"Oranges: {orange:3d}    {state_orange_count}|{required_oranges}", 10, 30)
    info_display.drawText(f"Fruit: {fruit_names[fruit] if fruit != -1 else 'None'}", 10, 50)
    # Mostra stato corrente
    info_display.drawText(f"State: {current_state}|{numero_stati}", 10, 100)
    
    # Mostra contatori dei contenitori con requisiti
    g1_req = current_requirements.get(bin_green1, 0)
    g2_req = current_requirements.get(bin_green2, 0)
    o1_req = current_requirements.get(bin_orange1, 0)
    o2_req = current_requirements.get(bin_orange2, 0)
    
    info_display.drawText(f"G1: {counter_G1} {state_bin_counts[bin_green1]}|{g1_req}", 10, 120)
    info_display.drawText(f"O1: {counter_O1} {state_bin_counts[bin_orange1]}|{o1_req}", 10, 140)
    info_display.drawText(f"G2: {counter_G2} {state_bin_counts[bin_green2]}|{g2_req}", 95, 120)
    info_display.drawText(f"O2: {counter_O2} {state_bin_counts[bin_orange2]}|{o2_req}", 95, 140)
    info_display.drawText(f"B1: {counter_binblue}", 180, 120)
    
    # Mostra delay e substate
    info_display.drawText(f"Delay: {remaining_delay:.1f}", 180, 140)
    info_display.drawText(f"Substate: {'END' if current_substate == 'END' else (current_substate.__name__ if hasattr(current_substate, '__name__') else 'Unknown')}", 10, 160)

# ============================================================================
# 5. GESTIONE FSA (FINITE STATE AUTOMATON)
# ============================================================================
def parse_fsa_message(message):
    """
    Analizza un messaggio FSA e crea un dizionario di configurazioni degli stati
    
    Args:
        message (str): Stringa contenente la configurazione FSA
        
    Returns:
        dict: Dizionario contenente le configurazioni degli stati, o None se c'è un errore
    """
    global numero_stati, first_load

    try:
        # Estrae il numero di stati dal messaggio
        num_states = int(message.split(',')[0].strip())
        numero_stati = num_states + 1 # memorizza quanti stati ci sono
        
        # Usa regex per estrarre le configurazioni degli stati
        state_configs = re.findall(r'\((\d+,(?:G|O)\d+,\d+,(?:G|O)\d+,\d+)\)', message)
        
        # Verifica la corrispondenza tra numero di stati dichiarati e configurazioni trovate
        if len(state_configs) != num_states:
            print(f"Errore: Il numero di stati ({num_states}) non corrisponde al numero di configurazioni ({len(state_configs)})")
            return None
            
        # Inizializza il dizionario FSA
        fsa = {}
        
        # Processa ogni configurazione di stato
        for i, config in enumerate(state_configs, start=1):
            # Divide la configurazione nei suoi componenti
            values = config.split(',')
            g1_count = int(values[0])
            g1_bin = values[1]
            o1_count = int(values[2])
            o1_bin = values[3]
            delay = int(values[4])
            
            # Converte i nomi dei contenitori
            bin_green = f"bin_green{g1_bin[-1]}"
            bin_orange = f"bin_orange{o1_bin[-1]}"
            
            # Calcola lo stato successivo
            # Se è l'ultimo stato, il prossimo stato è HALT (num_states + 1)
            next_state = i + 1 if i < num_states else (num_states + 1)
            
            # Crea la configurazione dello stato
            fsa[i] = {
                "trigger": next_state,
                "requirements": {
                    bin_green: g1_count,
                    bin_orange: o1_count
                },
                "delay": delay
            }
        
        # Aggiungi lo stato HALT
        halt_state_num = num_states + 1
        fsa[halt_state_num] = {
            "trigger": -1,
            "requirements": {},
            "delay": 0
        }
        
        if first_load:    
            print_fsa_debug_info(fsa, num_states)
            first_load = False
        
        return fsa

    except Exception as e:
        print(f"Errore nel parsing del messaggio FSA: {e}")
        return None

def print_fsa_debug_info(fsa, num_states):
    """Stampa informazioni di debug per la configurazione FSA"""
    print("\n=== Configurazione FSA ===")
    for state in range(1, num_states + 1):
        print(f"Stato {state}:")
        print(f"  Trigger: {fsa[state]['trigger']}")
        print("  Requirements:")
        for bin_name, count in fsa[state]['requirements'].items():
            print(f"    {bin_name}: {count} oggetti richiesti")
        print(f"  Delay: {fsa[state]['delay']} secondi")
    
    print(f"\nStato {num_states + 1}:")
    print("  HALT")
    print("\n========================")

# ============================================================================
# INIZIALIZZAZIONE FSA
# ============================================================================
# Funzione per leggere la stringa dal file
def leggi_stringa_da_file(percorso_file):
    try:
        with open(percorso_file, 'r') as file:
            return file.read().strip()  # Rimuove gli spazi vuoti e legge il contenuto
    except Exception as e:
        print(f"Errore durante la lettura del file: {e}")
        return None

# Carica il messaggio FSA dal file
message = leggi_stringa_da_file('fsa_message.json')

# Se non c'è un messaggio nel file, termina il programma
if message is None:
    print("Errore: File FSA non trovato o non leggibile")
    exit(1)  # Termina il programma con codice di errore

# Esegui il parsing del messaggio
FSA = parse_fsa_message(message)

if FSA is None:
    print("Errore: Parsing del messaggio FSA fallito")
    exit(1)  # Termina il programma con codice di errore
    
# Variabili per monitorare l'ultima modifica
last_modified_time = None

# Funzione per controllare se il file è stato modificato
def is_file_modified(percorso_file, last_modified_time):
    global first_load
    
    try:
        current_modified_time = os.path.getmtime(percorso_file)
        if current_modified_time != last_modified_time:
            return current_modified_time  # Il file è stato modificato
            first_load = True
        return last_modified_time  # Nessuna modifica
    except Exception as e:
        print(f"Errore nel controllo della modifica del file: {e}")
        return last_modified_time

# Funzione per leggere e analizzare il messaggio FSA
def load_fsa_message(percorso_file, last_modified_time):
    # Controlla se il file è stato modificato
    last_modified_time = is_file_modified(percorso_file, last_modified_time)
    
    # Se il file è stato modificato, rilegge e analizza il messaggio
    if last_modified_time != -1:
        message = leggi_stringa_da_file(percorso_file)
        
        if message:
            FSA = parse_fsa_message(message)
            if FSA is None:
                print("Errore: Parsing del messaggio FSA fallito")
                exit(1)  # Termina il programma con codice di errore
        else:
            print("Errore: File FSA non trovato o non leggibile")
            exit(1)  # Termina il programma con codice di errore
    else:
        print("Errore: Impossibile controllare le modifiche al file FSA")
        exit(1)  # Termina il programma con codice di errore

    return FSA, last_modified_time

# Imposta il percorso del file e la variabile per monitorare il tempo di modifica
file_path = 'fsa_message.json'
last_modified_time = -1  # Inizializza con un valore di default

# contatore contenitori
bin_counts = {
    "bin_green1": 0,
    "bin_green2": 0,
    "bin_orange1": 0,
    "bin_orange2": 0
}

# ============================================================================
# FUNZIONI DI GESTIONE DELLE POSIZIONI DI PRELIEVO
# ============================================================================
def get_picking_positions(fruit, state):
    """
    Determina la posizione corretta per il rilascio del frutto in base al tipo e allo stato
    
    Args:
        fruit (int): Tipo di frutto (0=arancia, 1=mela, 2=mela marcia)
        state (int): Stato corrente del sistema
        
    Returns:
        list: Lista delle posizioni target per i giunti del robot
    """
    # Gestione delle mele marce - vanno sempre nel cestino blu
    if fruit == 2:
        return target_positions[2]  # Posizione B1 (cestino mele marce)
        
    # Ottiene la configurazione dello stato corrente
    current_state_config = FSA[state]["requirements"]
    
    # Gestione delle mele (fruit == 1)
    if fruit == 1:
        # Cerca nei requisiti quale contenitore verde è richiesto
        for bin_name in current_state_config:
            if bin_name.startswith("bin_green"):
                # Sceglie tra G1 e G2 in base al numero del contenitore
                return target_positions[1] if bin_name.endswith("1") else target_positions[5]
    
    # Gestione delle arance (fruit == 0)
    elif fruit == 0:
        # Cerca nei requisiti quale contenitore arancione è richiesto
        for bin_name in current_state_config:
            if bin_name.startswith("bin_orange"):
                # Sceglie tra O1 e O2 in base al numero del contenitore
                return target_positions[0] if bin_name.endswith("1") else target_positions[4]
    
    # Posizione di fallback in caso di errori
    return target_positions[2]

# ============================================================================
# 6. FUNZIONI DI GESTIONE DEGLI STATI E TRANSIZIONI
# ============================================================================

def handle_state_delay(current_time):
    """
    Gestisce i ritardi tra gli stati e le relative transizioni.
    
    Args:
        current_time (float): Tempo corrente della simulazione
        
    Returns:
        bool: True se il delay è ancora attivo, False altrimenti
    """
    global state_delay_active, current_state, main_state_changed, state_bin_counts
    
    if not state_delay_active:
        return False
        
    speed_field.setSFFloat(0.0)  # Ferma il nastro durante il delay
    
    # Controllo se il delay è terminato
    if current_time >= state_delay_end_time:
        state_delay_active = False
        
        # Gestione transizione di stato
        if FSA[current_state]["trigger"] != -1:
            transition_to_next_state()
        else:
            halt_system()
            
        return False
    return True

def transition_to_next_state():
    """
    Gestisce la transizione al prossimo stato, resettando i contatori
    e riavviando il nastro trasportatore.
    """
    global current_state, main_state_changed, state_bin_counts
    global state_apple_count, state_orange_count
    
    old_state = current_state
    current_state = FSA[current_state]["trigger"]
    main_state_changed = True
    
    # Reset contatori per il nuovo stato
    state_bin_counts = {
        "bin_green1": 0,
        "bin_green2": 0,
        "bin_orange1": 0,
        "bin_orange2": 0
    }
    
    # Reset dei contatori di stato
    state_apple_count = state_orange_count = 0
    
    speed_field.setSFFloat(0.15)  # Riavvia il nastro

def check_state_requirements():
    """
    Verifica se i requisiti dello stato corrente sono stati soddisfatti.
    
    Returns:
        bool: True se i requisiti sono soddisfatti, False altrimenti
    """
    current_state_config = FSA[current_state]
    requirements = current_state_config["requirements"]
    
    for bin_name, required_count in requirements.items():
        current_count = state_bin_counts.get(bin_name, 0)
        if current_count < required_count:
            return False
    return True
    
def halt_system():
    """
    Gestisce l'arresto del sistema, portando il robot in posizione di riposo
    e fermando il nastro trasportatore.
    """
    global current_substate, is_process_complete
    
    is_process_complete = True
    current_substate = "END"  # Mark as END state
    speed_field.setSFFloat(0.0)
    
    # Movimento alla posizione di riposo
    for i in range(5):
        ur_motors[i].setPosition(target_HALT[i])

def start_state_delay(current_time):
    """
    Avvia il delay dello stato corrente.
    
    Args:
        current_time (float): Tempo corrente della simulazione
    """
    global state_delay_active, state_delay_end_time
    
    state_delay_active = True
    state_delay_end_time = current_time + FSA[current_state]["delay"]
    speed_field.setSFFloat(0.0)

def main_state():
    """
    Gestisce lo stato principale del sistema, coordinando le transizioni
    e i delay tra gli stati.
    """
    if is_process_complete:
        halt_system()
        return
    
    current_time = robot.getTime()
    
    # Gestione del delay tra stati
    if handle_state_delay(current_time):
        return
    
    # Verifica requisiti stato corrente
    if check_state_requirements() and not state_delay_active:
        start_state_delay(current_time)


# ============================================================================
# FUNZIONE DI RILEVAMENTO FRUTTA
# ============================================================================    
def find_fruit():
    """
    Analizza l'immagine dalla telecamera per rilevare e classificare i frutti
    
    Returns:
        int: Tipo di frutto rilevato (-1 se nessun frutto trovato)
    """
    # Inizializzazione liste per l'elaborazione dell'immagine
    global fruit_names, fruit_names
    min = []        # Valori minimi HSV per ogni colore
    max = []        # Valori massimi HSV per ogni colore
    cnts = []       # Contorni rilevati
    mask = []       # Maschere per ogni colore
    model = -1      # Tipo di frutto rilevato

    # Acquisizione e pre-processing dell'immagine
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    roi = img[0:150, 35:165]  # Regione di interesse
    imHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # Conversione in HSV

    # Definizione range colori in HSV
    # Arancione
    min.append(np.array([10, 135, 135], np.uint8))
    max.append(np.array([32, 255, 255], np.uint8))
    
    # Verde
    min.append(np.array([30, 50, 50], np.uint8))
    max.append(np.array([90, 255, 255], np.uint8))
    
    # Nero (per mele marce)
    min.append(np.array([0, 0, 0], np.uint8))
    max.append(np.array([179, 50, 30], np.uint8))
       
    
    # Kernel per operazioni morfologiche
    Kernel = np.ones((5, 5), np.uint8)
    
    # Processo di rilevamento per ogni colore
    for i in range(3):
        # Creazione della maschera per il colore corrente
        mask.append(cv2.inRange(imHSV, min[i], max[i]))
        
        # Operazioni morfologiche per migliorare la qualità della maschera
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_CLOSE, Kernel)
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_OPEN, Kernel)

        # Ricerca dei contorni nella maschera
        cnts.append(cv2.findContours(mask[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])

        # Analisi dei contorni trovati
        for c in cnts[i]:
            x, y, w, h = cv2.boundingRect(c)
            if w > 80:  # Se il contorno è abbastanza grande
                model = i  # Identifica il tipo di frutto
                printDisplay(x + 35, y, w, h, fruit_names[i])

    return model

# ============================================================================
# 8. FUNZIONI DI SUBSTATI DEL ROBOT
# ============================================================================
def action_waiting():
    """
    Funzione di attesa: gestisce il rilevamento dei frutti e l'avvio del processo
    di raccolta quando un frutto viene identificato
    
    Returns:
        function: Prossima funzione di stato da eseguire
    """
    global counter, fruit
    fruit = find_fruit()    # Rileva il tipo di frutto presente
    is_fruit_detected = distance_sensor.getValue() < 1000  # Verifica presenza fisica del frutto

    # Se un frutto è rilevato e identificato
    if is_fruit_detected and fruit != -1:
        playSnd(fruit)  # Riproduce il suono corrispondente al frutto
        fruit_counters[fruit]()  # Incrementa il contatore del tipo di frutto
        
        # Chiude le dita del gripper
        for motor in hand_motors:
            motor.setPosition(0.52)  # Chiude le dita
            
    counter = 8  # Imposta un delay per evitare rilevamenti multipli
    return actions_substate_machine.get(("waiting", is_fruit_detected), action_waiting)

def action_picking():
    """
    Funzione per il prelievo: muove il braccio robotico nella posizione corretta
    per prendere il frutto
    
    Returns:
        function: Funzione per la rotazione del braccio
    """
    global current_state, fruit
    
    # Ottiene le posizioni target per il frutto corrente
    selected_positions = get_picking_positions(fruit, current_state)
    
    # Imposta le posizioni dei motori per il prelievo
    for i in range(5):
        ur_motors[i].setPosition(selected_positions[i])

    return action_rotating
    
def action_rotating():
    """
    Funzione per la rotazione: controlla la rotazione del braccio verso
    la posizione di rilascio
    
    Returns:
        function: Prossima funzione di stato basata sulla rotazione completata
    """
    speed_field.setSFFloat(0.0)  # Ferma il nastro durante la rotazione

    # Verifica se la rotazione è completata (varia in base al tipo di frutto)
    if fruit != 2:
        is_rotated = position_sensor.getValue() < -2.3  # Controlla se il braccio ha raggiunto la posizione di rilascio
    else:
        is_rotated = position_sensor.getValue() < -2.16
    return actions_substate_machine.get(("rotating", is_rotated), action_rotating)

def action_dropping():
    """
    Funzione per il rilascio: gestisce il rilascio del frutto nel contenitore appropriato
    e aggiorna i contatori
    
    Returns:
        function: Funzione per il ritorno alla posizione iniziale
    """
    global counter, state_bin_counts, counter_O1, counter_O2, counter_G1, counter_G2, counter_binblue
    global counter_binblue, state_apple_count, state_orange_count
    
    # Apre le dita del gripper per rilasciare il frutto
    for motor in hand_motors:
        motor.setPosition(motor.getMinPosition())
    
    counter = 4  # Delay per il rilascio
    
    # Gestione mele marce
    if fruit == 2:
        counter_binblue += 1
        return action_rotate_back
    
    # Ottiene la configurazione dello stato corrente
    current_state_config = FSA[current_state]["requirements"]
    
    # Gestione mele
    if fruit == 1:
        state_apple_count += 1  # Incrementa il contatore di mele dello stato
        for bin_name in current_state_config:
            if bin_name.startswith("bin_green"):
                state_bin_counts[bin_name] += 1
                if bin_name == "bin_green1":
                    counter_G1 += 1
                elif bin_name == "bin_green2":
                    counter_G2 += 1
                break

    # Gestione arance
    elif fruit == 0:
        state_orange_count += 1  # Incrementa il contatore di arance dello stato
        for bin_name in current_state_config:
            if bin_name.startswith("bin_orange"):
                state_bin_counts[bin_name] += 1
                if bin_name == "bin_orange1":
                    counter_O1 += 1
                elif bin_name == "bin_orange2":
                    counter_O2 += 1
                break
    
    return action_rotate_back
    
    
def action_rotate_back():
    """
    Funzione per il ritorno: riporta il braccio alla posizione iniziale
    
    Returns:
        function: Funzione di attesa o continuazione basata sulla posizione
    """
    speed_field.setSFFloat(0.15)    # Riavvia il nastro
    is_back = position_sensor.getValue() > -0.1  # Verifica posizione di ritorno

    # Riporta tutti i motori alla posizione iniziale
    for motor in ur_motors:
        motor.setPosition(0.0)
    return actions_substate_machine.get(("rotate_back", is_back), action_waiting)

# ============================================================================
# 9. CONFIGURAZIONE MACCHINA A STATI
# ============================================================================
def no_fruit_action():
    """Funzione vuota per quando non ci sono frutti da processare"""
    pass

# Dizionario per aggiornare i contatori dei frutti
fruit_counters = {
    -1: no_fruit_action,  # Nessun frutto
    0: lambda: globals().__setitem__("orange", orange + 1),  # Incremento arance
    1: lambda: globals().__setitem__("apple", apple + 1),        # Incremento mele
    2: lambda: globals().__setitem__("rottenapple", rottenapple + 1)  # Incremento mele marce
}

# Macchina a substati per le azioni, che include anche picking e dropping
actions_substate_machine = {
    ("waiting", True): action_picking,
    ("waiting", False): action_waiting,
    ("picking", True): action_rotating,
    ("rotating", True): action_dropping,
    ("dropping", True): action_rotate_back,
    ("rotate_back", True): action_waiting
}

# ============================================================================
# 10. LOOP PRINCIPALE DEL PROGRAMMA
# ============================================================================
# Inizializzazione stati
current_state = 1  # Stato iniziale del sistema
current_substate = action_waiting  # Sottostato iniziale (attesa)

# Main loop
while robot.step(timestep) != -1:
    # Carica il messaggio FSA se il file è stato modificato
    FSA, last_modified_time = load_fsa_message(file_path, last_modified_time)

    # Gestione dello stato principale
    main_state()
    
    # Se il processo non è completato e il sottostato non è END
    if not is_process_complete and current_substate != "END":
        # Gestione del contatore di delay
        if counter > 0:
            counter -= 1
        else:
            # Gestione del cambio di stato
            if main_state_changed:
                current_substate = action_waiting
                main_state_changed = False
            
            # Aggiornamento del sottostato corrente
            try:
                # Verifica che current_substate sia una funzione prima di chiamarla
                if callable(current_substate):
                    current_substate = current_substate()
            except Exception as e:
                print(f"Error in substate execution: {e}")
                halt_system()
    
    # Aggiornamento del pannello informativo
    draw_info_panel()
    
    # Aggiornamento delle etichette del display
    robot.setLabel(
        1,
        f"Apples: {apple:3d}    Oranges: {orange:3d}    Substate: {'END' if current_substate == 'END' else (current_substate.__name__ if hasattr(current_substate, '__name__') else 'Unknown')}",
        0.07,
        0.96,
        0.06,
        0x00FF00,
        0,
        "Lucida Console",
    )
    robot.setLabel(
        2,
        f"State: {current_state}    G1: {counter_G1}  O1: {counter_O1} " + 
        f"G2: {counter_G2}  O2: {counter_O2}  Blue: {counter_binblue} Delay: {max(0, state_delay_end_time - robot.getTime()) if state_delay_active and not is_process_complete else 0:.1f}",
        0.07,
        0.91,
        0.05,
        0x0000FF,
        0,
        "Lucida Console",
    )
