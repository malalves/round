#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

ros::Publisher twsPub;
double x[] = {4,-3,-3,4}, y[] = {1.5,1.5,-1,-1};
double delta = 0.5;
double ru, theta, th, betha, r, psi, xc, yc;
geometry_msgs::Twist tws;
int i=0,ra;

double normAng (double ang){
	while(ang/M_PI > 1){
		ang = ang - 2*M_PI;
	}
	while(ang/M_PI <= -1){
		ang = ang + 2*M_PI;
	}
	return ang;
}

void callback(const nav_msgs::OdometryConstPtr &od){
	ra = (i+1)%4;	
	ROS_INFO("counter+1 %i",ra);
	double yaw = tf::getYaw(od->pose.pose.orientation);
	if(sqrt(pow((x[ra]-od->pose.pose.position.x),2) + pow((y[ra] - od->pose.pose.position.y),2))<0.3) i = ra;
	
	ru = sqrt(pow((x[i]-od->pose.pose.position.x),2)+pow((y[i]-od->pose.pose.position.y),2));
	th = std::atan2((y[i]-od->pose.pose.position.y),(x[i]-od->pose.pose.position.x));
	theta = std::atan2((y[ra]-y[i]),(x[ra]-x[i]));
	betha = normAng(theta - th);
	r = sqrt((ru*ru)-pow(ru*std::sin(betha),2));

	xc = x[i]+((r+delta)*std::cos(theta));
	yc = y[i]+((r+delta)*std::sin(theta));
	psi = std::atan2((yc-od->pose.pose.position.y),(xc-od->pose.pose.position.x));	
	ROS_INFO("%lf",normAng(psi-yaw));
	tws.linear.x = 1;
	tws.angular.z = 2*normAng(psi-yaw);

	twsPub.publish(tws);
	ROS_INFO("published....");
}

int main(int argc, char **argv){
	ros::init(argc,argv,"round");
	ros::NodeHandle chaser;
	ros::Subscriber odSub = chaser.subscribe("/vrep/vehicle/odometry",1, callback);
	twsPub = chaser.advertise <geometry_msgs::Twist>("/twist",1);
	ros::spin();
}
