# Carte_pneumatique
Code de la carte pneumatique.

améliorations + bugs potentiels : 

-   Faire passer la lecture analogique de température PT100 en DMA

-   Faire un timer de 100s pour purger et arrêter le compresseur en fin de match

-   "Bugs" : on ne peut pas envoyer plusieurs commandes à la chaîne sur la nucléo (output : HAL_TIMEOUT), et en mode debug, si on envoit la prochaine commande avant d'exécuter les parties principales de la boucle while du code (régulation de la pression + actions en fonction du mot de commande + retours à la Rpi), cette commande ne sera pas reçue, et il y aura un HAL_TIMEOUT.

Actionneurs/capteurs gérés par la nucléo:

Canons (pour propulsion des cerises):
ESC Canon 1 : D-Shot
ESC Canon 2 : D-Shot
ESC Canon 3 : D-Shot

Électrovannes (pour actionner piston de la pince gâteaux n°x OU pour purger le réservoir):
Électrovanne 1 (n°1) : TOR
Électrovanne 2 (n°2) : TOR
Électrovanne 3 (n°3) : TOR
Électrovanne 4 (purge) : TOR

ESC Turbine (pour aspiration cerises) : D-Shot
ESC Compresseur (pour réservoir) : D-Shot

Capteur de pression (pression du réservoir) : I2C ou analog input
Capteur de température (pour Compresseur réservoir) : I2C ou analog input

Cahier des charges des actions à réaliser :
Canons (pour propulsion des cerises):
Démarrer/Arrêter les moteurs canon (pas tous à la même vitesse, à voir en fonction de la balistique) lorsqu'on veut éjecter des cerises.
Sélectionner au moins 3 niveaux de vitesse, différent pour chaque moteur

Électrovannes 1 à 3: activer en TOR les EV lorsqu'on choppe les gâteaux avec les barillet.
Électrovanne 4: purger le réservoir en fin de match/lorsque l'arrêt d'urgence est enclenché. optionnellement la purge servira à souffler les cerises devant le robot

ESC Turbine : Activer lorsqu'on veut aspirer des cerises.
ESC Compresseur :

Activer en fonction de la pression du capteur de pression du réservoir pour garder une pression stable dans le réservoir.
Retourner à la Rpi la pression courante du réservoir en 10èmes de bar [XZZZZZZZ] avec des valeurs entre 0 et 127, donc le dernier bit est Libre.

Capteur de pression : lire en continu les valeurs de pression du réservoir
Capteur de température : lire en continu et inhiber le compresseur si température critique (l'erreur doit être remontée à la Rpi).

Information binaire minimales requises, structure "mots" 8 bits en écriture (W): [XXZZZZZZ]
- [II] : instruction:
  > [00]: reset [00000000] ou arrêt compresseur seul [00ZZZZZZ]
  > [01]: canon
  > [10]: compresseur
  > [1110]: Turbine
  > [1100]: EV
  > [1101]: LED
  > [1111]: LCD
- [ZZZZZZ] : valeurs, voir ci-dessous:
W/R Canons, 3x2 bits : [GGDDHH]
  > [00] : arrêt
  > [01] ; [10] et [11] : vitesse 1 ; 2 et 3

W/R Compresseur/réservoir, 6 bits : [PPPPPP]
  > [000000] : arrêt du maintien en pression et purge
  > [PPPPPP] : mise en pression du compresseur à la valeur consigne (=[PPPPPP] x 0,1bar) qui ne dépassera jamais 6,3 bar (grâce à la soupape de sécurité)

R Température (optionnel si les trains de mots ne sont pas possible, sinon elle fait suite au R du compresseur), 6 bits :[TTTTTT]
  > [TTTTTT] : température 0-160° (=[TTTTTT] x 2,5°C)

W/R  Electrovanne, 4x1 bits : [1100ABCE]  
  > [0] : arrêt, pas de pression/pince ouverte
  > [1] : active, pression/pince fermée

W/R  Turbine [111000ZZ]
  > [01] ; [10] et [11] : vitesse 1 ; 2 et 3

W  EV-Pulse [1101ZZZZ]
Electrovanne, 4x1 bits : [1101ABCE]
  > [0] : pas d'action
  > [1] : séquence de pulse ON-OFF-ON exécutée une seule fois avec un interval 

t_pulse_OFF configurable en ms dans une variable globale
W/R  LED et LCD [1111000Z]

LED [111100xZ]
  > [0] LED OFF
  > [1] LED ON

LCD [111100Yx]
  > [0] LCD OFF
  > [1] LCD ON

Retours à la Rpi:
1er byte : Pression du réservoir sur 7 bits + 1 bit d'erreur: [EZZZZZZZ]
  > [E] = 0 : communication avec le capteur échouée 
  > [E] = 1 : communication avec le capteur réussie
  > [ZZZZZZZ] : pression 0 - 127 dixièmes de bar

2ème byte : Température critique compresseur + 1 bit de bonne réception de la commande de la Rpi par la nucléo [EXXXXXXC]
  > [E] = 0 : température compresseur OK
  > [E] = 1 : température Critique du compresseur
  > [C] = 0 : Erreur de communication, non réception du mot de commande 8 bits par la nucléo
  > [C] = 1 : Communication OK, la nucléo a reçu la commande et la traiter
