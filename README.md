# ROS Progetto Webots UR5e

![UR5e in Webots](https://github.com/KevinPetrone/ROS_ProgettoWebots_UR5e/blob/main/images/project.jpg)

Questo repository contiene il codice e la documentazione per un progetto che integra il robot UR5e con il simulatore Webots utilizzando ROS (Robot Operating System). Il progetto è stato sviluppato come parte di un corso o di un progetto personale per esplorare le capacità di simulazione e controllo di un braccio robotico industriale in un ambiente virtuale.

[**Relazione Prezi**](https://prezi.com/view/fzvbbkLgXX44PGaWyzf1) Una relazione tecnica che descrive il funzionamento e la struttura di un sistema di controllo per un robot, probabilmente un braccio robotico come l'UR5, basato su una macchina a stati finiti (FSA). Il sistema è progettato per essere altamente flessibile e configurabile, utilizzando file JSON per definire gli stati, i requisiti e le azioni del robot. Attraverso l'integrazione con ROS (Robot Operating System) e Webots (un simulatore robotico), il sistema permette di modificare dinamicamente il comportamento del robot senza necessità di riavvii, adattandosi a nuove esigenze in tempo reale.

La relazione spiega come il robot gestisce le transizioni tra stati e sottostati, come waiting, picking, rotating, e dropping, e come queste transizioni siano attivate da trigger specifici, come il completamento di un'azione o il soddisfacimento di determinati requisiti. Viene inoltre descritto il ruolo del main loop, che coordina le operazioni del robot, monitora i sensori e gestisce le transizioni di stato, garantendo un funzionamento fluido e tempistiche precise.

Un aspetto importante è la flessibilità del sistema: nuovi stati e azioni possono essere aggiunti semplicemente estendendo il dizionario delle azioni o modificando il file JSON di configurazione. Il sistema include anche una gestione dei delay e uno stato di HALT per interrompere in modo sicuro le operazioni del robot quando necessario.

Infine, la relazione sottolinea l'integrazione tra ROS e Webots, che avviene attraverso un sistema di comunicazione basato su file JSON, permettendo una riconfigurazione dinamica del comportamento del robot. Il tutto è supportato da un pannello informativo che fornisce feedback in tempo reale sullo stato del sistema, facilitando il monitoraggio e la diagnosi di eventuali problemi.

## Descrizione del Progetto

Il progetto mira a simulare il braccio robotico UR5e in un ambiente virtuale creato con Webots, utilizzando ROS per il controllo e la comunicazione tra i vari componenti. Il simulatore Webots fornisce un ambiente realistico per testare e validare algoritmi di controllo, pianificazione del movimento e altre funzionalità robotiche prima di essere implementate su un robot fisico.

### Funzionalità Principali

- **Simulazione del UR5e**: Modello accurato del braccio robotico UR5e in Webots.
- **Integrazione con ROS**: Utilizzo di ROS per il controllo e la comunicazione con il robot simulato.
- **Pianificazione del Movimento**: Implementazione di algoritmi di pianificazione del movimento per il braccio robotico.
- **Interfaccia Utente**: Possibilità di controllare il robot tramite un'interfaccia utente o script ROS.

## Requisiti

Per eseguire questo progetto, sono necessari i seguenti strumenti e librerie:

- **Webots**: Il simulatore robotico. [Scarica Webots](https://cyberbotics.com/)
- **ROS**: Robot Operating System. [Installa ROS](https://github.com/KevinPetrone/ROS_ProgettoWebots_UR5e/blob/main/Tutorial/Installazione_ROS_Noetic.pdf) 
- **UR5e Model**: Il modello del robot UR5e per Webots.
- **Pacchetti ROS**: Vari pacchetti ROS necessari per il controllo e la simulazione, scaricare le guide ROS.

**Clona il repository**:
   ```bash
   git clone https://github.com/KevinPetrone/ROS_ProgettoWebots_UR5e.git
   cd ROS_ProgettoWebots_UR5e
