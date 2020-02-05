# ddboat_authors  
Coco : colin.baumgard@ensta-bretagne.org  
Ludosky : ludovic.diguet@ensta-bretagne.org  
Rhamid : hamid.hacene@ensta-bretagne.org  
Liza : antonin.lize@ensta-bretagne.org
 
# Lancer ROS  
catkin_make  
roslaunch gpsd_client setup.launch  

# Pour le GPS  
Modifier le fichier /etc/default/gpsd de la façon suivante : 

'#'Default settings for the gpsd init script and the hotplug wrapper.  
'#' Start the gpsd daemon automatically at boot time  
START_DAEMON="true"  
'#' Use USB hotplugging to add new USB devices automatically to the daemon  
USBAUTO="false"  
'#' Devices gpsd should collect to at boot time.  
'#' They need to be read/writeable, either by user gpsd or the group dialout.    
DEVICES="/dev/ttyS0"  
'#' Other options you want to pass to gpsd  
GPSD_OPTIONS=""  

# Pour installer les librairie    
- Aller dans external_libs/  
- Taper : bash setup_ext_libs.bash  
- Taper : sudo bash sudo_setup.bash  

