Projet sur carte Nucléo F303K8T6. à l'aide des commandes de la Rpi, sur le compresseur, la turbine, les électrovannes et les canons du Robot.
La carte agit, en fonction des commandes de la Raspi, sur :
  - compresseur (BLDC) : D-Shot
  - turbine : D-Shot
  - électrovannes : TOR
  - canons (3 BLDC) : D-Shot
  - capteur de pression (WSEN PDUSn 2513130810401) : I2C
  - capteur de température (PT100) : analogique

La documentation des capteurs et les détails du protocole de commande Raspi=>nucléo sont ici:
TOOLS-DOCUMENTATION -> DOC

Le script python pour tester l'envoi de commandes sur la nucléo et les réponses des capteurs (fichiers excel) sont ici:
TOOLS-DOCUMENTATION -> TOOLS

![Pneuma_Pinout](https://github.com/goldobot/Carte_pneumatique/assets/90452075/5299fd09-087e-4b0d-a9cc-e80d472ba9d5)
