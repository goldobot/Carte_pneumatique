DERNIERE VERSION DU CODE (dernier merge sur main) A TESTER surtout pour temperature critique du compresseur. Si cette derniere version ne marche pas, la derniere version Bonne est l'AVANT DERNIER COMMIT DE LA BRANCHE TEST : "fix erreurs de code: init command_hist a NULL pour pas defiler a linfini dans le insert"

Projet sur carte Nucleo F303K8T6. a l'aide des commandes de la Rpi, sur le compresseur, la turbine, les electrovannes et les canons du Robot.
La carte agit, en fonction des commandes de la Raspi, sur :
  - compresseur (BLDC) : D-Shot
  - turbine : D-Shot
  - electrovannes : TOR
  - canons (3 BLDC) : D-Shot
  - capteur de pression (WSEN PDUSn 2513130810401) : I2C
  - capteur de temperature (PT100) : analogique

La documentation des capteurs et les details du protocole de commande Raspi=>nucleo sont ici:
TOOLS-DOCUMENTATION -> DOC

Le script python pour tester l'envoi de commandes sur la nucleo et les reponses des capteurs (fichiers excel) sont ici:
TOOLS-DOCUMENTATION -> TOOLS

Pinout : 
![Capture](https://github.com/goldobot/Carte_pneumatique/assets/90452075/6edad983-73cf-4a0e-a7a7-4d94c6e21541)
