# Carte Pneumatique

---

## Présentation
carte: **Nucleo F303K8T6**.

À l’aide des commandes de la **Raspberry Pi**, la carte pilote le compresseur, la turbine, les électrovannes et les canons du robot.

La carte agit, en fonction des commandes de la Raspi, sur :
- **compresseur (BLDC)** : D-Shot
- **turbine** : D-Shot
- **électrovannes** : TOR
- **canons (3 BLDC)** : D-Shot
- **capteur de pression** (WSEN PDUSn 2513130810401) : I2C
- **capteur de température** (PT100) : analogique

---

## Documentation / outils
La documentation des capteurs et les détails du protocole de commande Raspi=>Nucleo sont ici :
- `TOOLS-DOCUMENTATION -> DOC`

Le script python pour tester l'envoi de commandes sur la nucleo et les réponses des capteurs (fichiers excel) sont ici :
- `TOOLS-DOCUMENTATION -> TOOLS`

---

## Pinout
![Capture](https://github.com/goldobot/Carte_pneumatique/assets/90452075/6edad983-73cf-4a0e-a7a7-4d94c6e21541)

Infos pinout (complément) :
- **PA2 et PA15 ⇒ UART_TX/RX** car hard wired sur la carte nucleo (voir schematics F303K8)

---

# Carte_Pneuma_dev
########## INFOS CARTE PNEUMA ##########

## Actionneurs et protocoles associés
Actionneurs/capteurs gérés par la nucléo :

### Canons (pour propulsion des cerises)
- ESC Canon 1 : D-Shot  
- ESC Canon 2 : D-Shot  
- ESC Canon 3 : D-Shot  

### Électrovannes (pince / purge)
- Électrovanne 1 (n°1) : TOR  
- Électrovanne 2 (n°2) : TOR  
- Électrovanne 3 (n°3) : TOR  
- Électrovanne 4 (purge) : TOR  

### Aspiration / Réservoir
- ESC Turbine (pour aspiration cerises) : D-Shot  
- ESC Compresseur (pour réservoir) : D-Shot  

### Capteurs
- Capteur de pression (pression du réservoir) : I2C ou analog input  
- Capteur de température (pour Compresseur réservoir) : I2C ou analog input  

---

## Fonctionnalités des actionneurs (cahier des charges)

### Canons
- Démarrer/Arrêter les moteurs canon (pas tous à la même vitesse, à voir en fonction de la balistique) lorsqu’on veut éjecter des cerises.
- Sélectionner au moins 3 niveaux de vitesse, différent pour chaque moteur

### Électrovannes 1 à 3
- Activer en TOR les EV lorsqu’on choppe les gâteaux avec les barillet.

### Électrovanne 4 (purge)
- Purger le réservoir en fin de match / lorsque l'arrêt d'urgence est enclenché.
- Optionnellement la purge servira à souffler les cerises devant le robot.

### ESC Turbine
- Activer lorsqu’on veut aspirer des cerises.

### ESC Compresseur / réservoir
- Activer en fonction de la pression du capteur de pression du réservoir pour garder une pression stable dans le réservoir.
- Retourner à la Rpi la pression courante du réservoir entre 0.0 et 10.0 bar par incrément de 0.1
  - (soit moins de 128 valeurs nécessaires si on veut se limiter à 7 bits)

### Capteur de pression
- Lire en continu les valeurs de pression du réservoir.

### Capteur de température
- Lire en continu et inhiber le compresseur si température critique (l'erreur doit être remontée à la Rpi).

---

## Protocole de com Raspi <--> Nucleo

### Structure binaire minimale (écriture W)
Structure "mots" 8 bits en écriture (W) : **[XXZZZZZZ]**

- **[II]** : instruction  
- **[ZZZZZZ]** : valeurs (selon instruction)

#### [II] : instruction
- **[00]** : reset **[00000000]** ou arrêt compresseur seul **[00ZZZZZZ]**
- **01** : canon
- **10** : compresseur
- **11** : Turbine + EV

---

### W/R Canons (3x2 bits) : [GGDDHH]
- **[00]** : arrêt
- **[01]**, **[10]**, **[11]** : vitesse 1, 2 et 3

---

### W/R Compresseur / réservoir (6 bits) : [PPPPPP]
- **[000000]** : arrêt du maintien en pression et purge
- **[PPPPPP]** : mise en pression du compresseur à la valeur consigne
  - consigne = \([PPPPPP] \times 0{,}1\ \text{bar}\)
  - ne dépassera jamais **6,3 bar** (grâce à la soupape de sécurité)

---

### R Température (optionnel)
(optionnel si les trains de mots ne sont pas possible, sinon elle fait suite au R du compresseur)

Température sur 6 bits : **[TTTTTT]**
- température = \([TTTTTT] \times 2{,}5^\circ\text{C}\)
- plage : **0–160°C**

---

### W/R Électrovannes (4x1 bits) : [ABCE]
- **[0]** : arrêt, pas de pression / pince ouverte
- **[1]** : active, pression / pince fermée

---

### W/R Turbine (1x2 bits) : [FF]
- **[00]** : arrêt, pas de pression / pince ouverte
- **[01]**, **[10]**, **[11]** : vitesse 1, 2 et 3