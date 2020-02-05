# ddboat
Coco  
Ludosky  
Rhamid  
  
# gpsd_client  
Noeud qui permet de lire les trames NMEA.  
Il publie un message sut le topic /Fix.  
## pour le lancer :  
sudo service gpsd stop  
gpsd -D S -N -n /dev/ttyUSB0  
rosrun gpsd_client gpsd_client

<<<<<<< HEAD
=======

>>>>>>> c1a13662067270b8e212fcde577ed74d7549bbec
