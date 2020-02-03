#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"


#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"

#include "tf/tf.h"

# define M_PI           3.14159265358979323846  


double Xbateau[3];  //vecteur d'état du bateau (x,y,theta)
douvle Vbateau[3]; //derivée du vecteur d'étéan
double u;  //vecteur de controle
double Xcible[2]; //coordonnes de la cible
double Vcible[2]; //vitesse de la cible 
double e;

void get_X_bateau(const geometry_msgs::PoseStamped msg){
    Xbateau[0] =  msg.pose.position.x;
    Xbateau[1] =  msg.pose.position.y;
    Xbateau[2] =  tf::getYaw(msg.pose.orientation);

    }

void get_position_cible(const geometry_msgs::Vector3 msg){
    Xcible[0] =  msg.x;
    Xcible[1] =  msg.y;
}

void get_position_cible(const geometry_msgs::Vector3 msg){
    Vcible[0] =  msg.x;
    Vcible[1] =  msg.y;
}

double controller(){
    //double xv = Xc[0]  - Xb[0];
    //double yv = Xc[1]  - Xb[1];
    //double thetav = atan2(yv,xv);
    //e = thetav - Xb[2];

    //if (abs(e) > M_PI/4){
    //    return (-M_PI/10);}
    //else {
    //    return(0.4 *e );
    return(0);
    }




}


int main(int argc, char **argv){

    // INITIALISATION
    ros::init(argc, argv, "control");

    //DECLARATION DE SUBSCRIBERS
    ros::NodeHandle xbateau;
    ros::NodeHandle xcible;
    ros::NodeHandle vcible
    ros::Subscriber sub1 = xbateau.subscribe("X_bateau", 1000, get_X_bateau);
    ros::Subscriber sub2 = xcible.subscribe("X_bateau", 1000, get_X_bateau);
    ros::Subscriber sub3 = vcible.subscribe("X_bateau", 1000, get_X_bateau);

    //DECLARATION DE PUBLISHERS
    ros::NodeHandle thetap;
    ros::Publisher pub = thetap.advertise<std_msgs::float64>("theta_prime", 1000);

    
    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();

        std_msgs::float64 theta_prime;
        geometry_msgs::PointStamped cible;

        theta_prime = controller();

        // PUBLICATION
        pub.publish(theta_prime);
        loop_rate.sleep();

    }

    }



