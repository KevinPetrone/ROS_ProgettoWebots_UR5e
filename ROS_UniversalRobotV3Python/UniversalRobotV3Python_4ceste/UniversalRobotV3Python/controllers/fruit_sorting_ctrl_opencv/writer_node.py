#!/usr/bin/env python3

import rospy  # Libreria ROS
import json  # Libreria per lavorare con JSON
import subprocess
import os

# Percorso del file JSON
FASI = "/mnt/c/Users/utente/Desktop/ROS_UniversalRobotV3Python/UniversalRobotV3Python_4ceste/UniversalRobotV3Python/controllers/fruit_sorting_ctrl_opencv/fsa_message.json" # Aggiorna il percorso con la posizione del file JSON sul tuo sistema
     
def write_to_file(data_str):
    """
    Scrive la stringa nel file JSON.
    """
    try:
        with open(FASI, 'w') as file:
            file.write(data_str)
        rospy.loginfo(f"Dati scritti nel file: {FASI}")
    except Exception as e:
        rospy.logwarn(f"Errore durante la scrittura del file JSON: {e}")

def parse_input(data_str):
    """
    Converte la stringa di input in una stringa compatibile con il formato richiesto da Webots.
    """
    try:
        data_parts = data_str.split(', ', 1)
        num_fasi = int(data_parts[0])
        fasi_list = []

        # Debug: stampa il contenuto della seconda parte
        rospy.loginfo(f"Contenuto fasi: {data_parts[1]}")

        # Rimuove le parentesi esterne e corregge la stringa
        data_fasi = data_parts[1].strip().strip('()')  # Rimuove le parentesi esterne
        rospy.loginfo(f"Fasi dopo la rimozione parentesi: {data_fasi}")

        fasi = data_fasi.split('), (')  # Ora le fasi sono separate correttamente
        rospy.loginfo(f"Fasi separate: {fasi}")

        for fase in fasi:
            fase = fase.strip('()')  # Pulisce le parentesi esterne rimaste
            rospy.loginfo(f"Fase pulita: {fase}")

            # Rimuove spazi extra prima e dopo la virgola
            valori = [val.strip() for val in fase.split(',')]  # Pulisce gli spazi extra
            if len(valori) == 5:  # Verifica che ci siano 5 valori
                target_mele, scatola_mele, target_arance, scatola_arance, ritardo = valori
                fasi_list.append(f"({target_mele},{scatola_mele},{target_arance},{scatola_arance},{ritardo})")
            else:
                rospy.logwarn(f"Errore nei valori della fase: {fase}. Si aspettano 5 valori.")

        # Crea la stringa finale nel formato richiesto da Webots
        fsa_message = f"{num_fasi}, " + ", ".join(fasi_list)
        return fsa_message
    except ValueError as e:
        rospy.logwarn(f"Errore nel parsing dell'input: {e}")
        return None

def writer_node():
    """
    Nodo ROS per raccogliere input e scrivere su un file JSON.
    """
    rospy.init_node('robot_state_writer', anonymous=True)

    while not rospy.is_shutdown():
        rospy.loginfo("Inserisci i dettagli delle fasi nel formato:")
        rospy.loginfo("Esempio: 3, (1,G1,1,O1,5), (1,G2,2,O2,3), (2,O1,1,G1,4)")

        data_str = input("Inserisci il numero e i dettagli di ogni stato: ")

        data_str_formatted = parse_input(data_str)

        if data_str_formatted:
            write_to_file(data_str_formatted)
            rospy.loginfo(f"Dati scritti nel file JSON: {FASI}")
            rospy.loginfo(f"Invio il messaggio: {data_str_formatted}")
        else:
            rospy.logwarn("Input non valido. Verifica il formato e riprova.")
        
        # Chiedere se continuare o meno
        cont = input("Vuoi inserire altri dati? (s/n): ")
        if cont.lower() != 's':
            break

if __name__ == '__main__':
    try:
        writer_node()
    except rospy.ROSInterruptException:
        pass