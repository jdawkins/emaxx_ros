#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <emaxx_ros/emaxx_status.h>
#include <serial/serial.h>

#define RED 0xFF0000
#define ORANGE 0xFF8000
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define CYAN 0x00FFFF
#define BLUE 0x0000FF
#define PURPLE 0x7F00FF
#define WHITE 0xFFFFFF
#define OFF 0x000000

#define LOITER_STATE 0
#define TELEOP_STATE 1    //Sends commands from Joystick
#define AUTO_STATE 2
#define HOME_STATE 3

#define DRIVE_MODE 0
#define COURSE_MODE 1

#define TELEOPERATION 0
#define WAYPOINT 1
#define PATH 2
#define VELOCITY 3
#define TRAJECTORY 4

#define PI (3.14159)
#define TS 0.01
#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)

#define TRACK_WIDTH 0.3 //m
#define MAX_SPEED (128*4.5/1000) //max linear speed in m/s
#define MAX_YAW_RATE MAX_SPEED/(TRACK_WIDTH)
#define THROTTLE_FACTOR 0.5

#define CRUISE_SPEED 2


ros::Subscriber joy_sub,path_sub,wp_sub,traj_sub,vel_sub;

ros::Publisher cmd_pub, imu_pub, vel_pub,pose_pub;
ros::Publisher status_pub;
geometry_msgs::Twist ugv_cmd;

float LX,LY,LT,RX,RY,RT,DPX,DPY;
bool BA,BB,BX,BY,BL1,BR1;
int path_index, path_size;
std::string ugv_name, nav_frame,port_name;
struct timespec current_time,start_time;

nav_msgs::Path curr_path;

geometry_msgs::PoseStamped  pose, old_pose, des_pose;
//geometry_msgs::PoseStamped des_pose;
geometry_msgs::Twist des_vel, vel;
sensor_msgs::Imu imu; 
trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory;
emaxx_ros::emaxx_status status,old_status;
serial::Serial my_serial;

double max_yaw_rate, max_speed, cruise_speed,ctrl_offset;
double err_rate_x, err_rate_y,err_x,err_y,err_x_old,err_y_old;
std::vector<double> home;
struct timespec path_init_time, path_time, path_stop_time;

double wheel_radius, track_width;
double Kp_psi,Kp_spd,Ki_spd;
double arrived_radius;

double ax,ay,az,gx,gy,gz;
double roll,pitch,yaw;
double des_roll,des_pitch,des_yaw;

double pos_n,pos_e,vel_n,vel_e,ref_lat,ref_lon,ref_alt;
double brightness;

bool armed, path_received;
int sim_mode;

void sendLED(int color,float brightness);

void waypointCallbBack(const geometry_msgs::PoseStamped pose_msg){
    status.ctrl_mode = COURSE_MODE;
    status.nav_goal = WAYPOINT;
    
    des_pose = pose_msg;
  
}
void targetVelCallBack(const geometry_msgs::Twist vel_msg){
    status.ctrl_mode = COURSE_MODE;
    status.nav_goal = VELOCITY;
    
    des_vel = vel_msg;
  
}
void pathCallBack(const nav_msgs::Path path_msg){
    status.ctrl_mode = COURSE_MODE;
    status.nav_goal = PATH;

    curr_path = path_msg;
    path_index = 0;
    path_size = path_msg.poses.size();
    path_received = true;
}

void trajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg){
    trajectory = trajectory_msg;
    path_stop_time.tv_sec = trajectory.time_from_start.sec;
    path_stop_time.tv_nsec = trajectory.time_from_start.nsec;
    ROS_ERROR("sec: %d nsec: %d\n",path_stop_time.tv_sec,path_stop_time.tv_nsec);
    path_received = true;
}

void joyCallBack(const sensor_msgs::Joy &joy_msg){


    LX = joy_msg.axes[0];
    LY = joy_msg.axes[1];
    RX = joy_msg.axes[3];
    RY = joy_msg.axes[4];

    BA = joy_msg.buttons[0];
    BB = joy_msg.buttons[1];
    BX = joy_msg.buttons[2];
    BY = joy_msg.buttons[3];
    BL1 = joy_msg.buttons[4];
    BR1 = joy_msg.buttons[5];

    if(BA){
        status.nav_state = LOITER_STATE;
        status.armed = true;
    }
    if(BX){	  
      status.nav_state = LOITER_STATE;
	status.armed = false;
    }    

    if(status.armed){

        if(BR1){
            status.nav_state = AUTO_STATE;
            ROS_INFO("Navigation State Set to Auto");	    
	    sendLED(RED,brightness);
        }        

        if(BB){
	  status.nav_state = LOITER_STATE;
	  status.nav_goal = TELEOPERATION;
          ROS_INFO("Navigation State Set to Loiter");
	  sendLED(BLUE,brightness);
	  
        }
        if(BY){
	  status.nav_state = HOME_STATE;
	  status.nav_goal = WAYPOINT;
	  sendLED(GREEN,brightness);
	  
          ROS_INFO("Navigation State Set to Return Home");
	  
        }
        //If either of the sticks are moved
        if(abs(LX)>0.1 || abs(LY)>0.1 || abs(RX)>0.1 || abs(RY)>0.1){
	 status.nav_state = TELEOP_STATE;
	    if(old_status.nav_state != status.nav_state){
		ROS_INFO("Teleop Control Active");
		status.ctrl_mode = DRIVE_MODE;
		status.nav_goal = TELEOPERATION;
	      
	    }
	}
    } 

}

double computeNorm(double x, double y){

  double dist = sqrt(pow(x,2)+ pow(y,2));
  return dist;
}
double calculateBaseline(geometry_msgs::TransformStamped point_1,geometry_msgs::TransformStamped point_2){

    double range = sqrt(pow(point_2.transform.translation.x-point_1.transform.translation.x,2)+
                        pow(point_2.transform.translation.y-point_1.transform.translation.y,2));
    return range;
}
double calculateBaseline(geometry_msgs::PoseStamped point_1,geometry_msgs::TransformStamped point_2){

    double range = sqrt(pow(point_2.transform.translation.x-point_1.pose.position.x,2)+
                        pow(point_2.transform.translation.y-point_1.pose.position.y,2));
    return range;
}

double wrapToPi(double angle){

    while(angle > PI){
        angle -= 2*PI;
    }
    while(angle < -PI){
        angle += 2*PI;
    }

    return angle;
}

void parseMessage(std::string data){

  std::string head = data.substr(0,4);
  
  if(!strcmp(head.c_str(),"$EKF")){
    
      sscanf(data.c_str(),"$EKF,%lf,%lf,%lf,%lf,%lf",&pos_n,&pos_e,&vel_n,&vel_e,&yaw);
    
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion q;
      geometry_msgs::TransformStamped pose_tf;

      transform.setOrigin( tf::Vector3(pos_n, pos_e, 0.0) );
      q.setW(imu.orientation.w);
      q.setX(imu.orientation.x);
      q.setY(imu.orientation.y);
      q.setZ(imu.orientation.z);         

      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), nav_frame, ugv_name));
            
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = pos_n;
      pose.pose.position.y = pos_e;
      
      pose.pose.orientation.w = q.getW();
      pose.pose.orientation.x = q.getX();
      pose.pose.orientation.y = q.getY();
      pose.pose.orientation.z = q.getZ();
      
      vel.linear.x = vel_n;
      vel.linear.y = vel_e;
      
      pose_pub.publish(pose);
      vel_pub.publish(vel);    
    
  }
  
  if(!strcmp(head.c_str(),"$IMU")){    

      imu.header.frame_id = ugv_name;
      sscanf (data.c_str(),"$IMU,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&ax,&ay,&az,&gx,&gy,&gz,&roll,&pitch,&yaw);
      
      imu.linear_acceleration.x = ax;
      imu.linear_acceleration.y = ay;
      imu.linear_acceleration.z = az;
      imu.angular_velocity.x = gx;
      imu.angular_velocity.y = gy;
      imu.angular_velocity.z = gz;
            
      tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
      imu.orientation.w = q.getW();
      imu.orientation.x = q.getX();
      imu.orientation.y = q.getY();
      imu.orientation.z = q.getZ();
      
      imu_pub.publish(imu);
                 
  }
 // std::cout << head.c_str() << std::endl;
  //sscanf (sentence,"%s %*s %d",str,&i);
  
}

void sendSim(int mode,double x0,double y0, double psi0){
    char msg[50];
  
    sprintf(msg,"$SIM,%d,%f,%f,%f\n\r",mode,x0, y0, psi0);
    my_serial.write(msg);
     std::cout << msg << std::endl;    

  
}
void sendLED(int color,float brightness){
    char msg[50];
      sprintf(msg,"$LED,%x,%.1f\n\r",color,brightness);
      my_serial.write(msg);
      
      std::cout << msg << std::endl;    
}
void sendCtrlParams(double Kp_psi,double Kp_spd, double Ki_spd){
    char msg[50];
  
    sprintf(msg,"$PRM,1,%f,%f,%f\n\r",Kp_psi, Kp_spd, Ki_spd);
    my_serial.write(msg);
     std::cout << msg << std::endl;    

  
}
void sendGPSref(double lat,double lon, double alt){
    char msg[50];
  
    sprintf(msg,"$PRM,2,%f,%f,%f\n\r",lat,lon,alt);
    my_serial.write(msg);
    std::cout << msg << std::endl;    

  
}
void sendCommand(double cmd_yaw,double cmd_spd){

  char msg[50];
  
  sprintf(msg,"$CMD,%d,%.3f,%.3f\n\r",status.ctrl_mode,cmd_yaw,cmd_spd);
  my_serial.write(msg);
  std::cout << msg << std::endl;
  
}

void teleopControl(){

   sendCommand(LX,THROTTLE_FACTOR*RY);
  
}


void velocityControl(){

  float des_psi = atan2(des_vel.linear.y,des_vel.linear.x);
  double des_speed = computeNorm(des_vel.linear.x,des_vel.linear.y);

  if(des_speed > max_speed){
        des_speed = max_speed;
  }
    sendCommand(des_psi,des_speed);    
}

void waypointController(){

    double err_x = des_pose.pose.position.x - pose.pose.position.x;
    double err_y = des_pose.pose.position.y - pose.pose.position.y; 
    
    double range = computeNorm(err_x,err_y);
    des_yaw = atan2(err_y,err_x);
        
    double psi_err = wrapToPi(des_yaw - yaw);

 
    if(range >= arrived_radius){
       sendCommand(des_yaw,cruise_speed);
    }
    else{
       status.nav_goal = TELEOPERATION;
       sendCommand(des_yaw,0);
    }

}

void runControl(const ros::TimerEvent& e){

	if(status.armed){
	
	  if(status.nav_state == AUTO_STATE || status.nav_state == HOME_STATE){
	  switch (status.nav_goal){
	      
	      case VELOCITY: {
	      
		velocityControl();
		break;
	      }
	      case WAYPOINT: {
	      
		waypointController();
		break;
	      }	      
	      default:{
		teleopControl();

		break;
	      }
	    }	// end switch 
	    
	  } else{
	  
	      teleopControl();
	    
	  } // end else AUTO_STATE
	 } // end if armed
	    
 
  
}
/*void trajectoryController(){
    double err_x = desose.transform.translation.x - pose.transform.translation.x;
    double err_y = des_pose.transform.translation.y - pose.transform.translation.y; 
    
    double range = calculateBaseline(des_pose,pose);
    double des_speed = sqrt(pow(des_vel.linear.x,2) + pow(des_vel.linear.y,2));
    
    tf::Quaternion q;
    q.setW(des_pose.transform.rotation.w);
    q.setX(des_pose.transform.rotation.x);
    q.setY(des_pose.transform.rotation.y);
    q.setZ(des_pose.transform.rotation.z);
    
    tf::Matrix3x3 m(q);
    m.getEulerYPR(des_yaw,des_pitch,des_roll);      
    
    float des_psi = RAD2DEG*atan2(des_vel.linear.y,des_vel.linear.x);

 
    if(range >= arrived_radius){
       sendCommand(des_psi,cruise_speed);
    }
    else{
       sendCommand(des_psi,0);
    }  
 }*/


int main(int argc, char **argv){

  printf("Program Start!!!");

    ros::init(argc, argv, "emaxx_control_interface");

    ros::NodeHandle nh;

    status.nav_state = TELEOP_STATE;
    status.ctrl_mode = DRIVE_MODE;
    status.nav_goal = TELEOPERATION;
    status.armed = false;

    nh.param("port_name",port_name,std::string("/dev/ttyUSB0"));
    nh.param("ugv_name",ugv_name,std::string("emaxx"));
    nh.param("parent_frame",nav_frame,std::string("ned_frame"));
    nh.param("sim_mode",sim_mode,0);
    nh.param("gains/Kp_psi",Kp_psi,1.1);
    nh.param("gains/Kp_spd",Kp_spd,0.4);
    nh.param("gains/Ki_spd",Ki_spd,0.07);
    nh.param("wp_radius",arrived_radius,0.5);

    nh.param("limits/speed",max_speed,1.0);
    nh.param("limits/cruise_speed",cruise_speed,1.0);
    
    nh.param("gps/ref_lat",ref_lat,38.984057);
    nh.param("gps/ref_lon",ref_lon,-76.484199);
    nh.param("gps/ref_alt",ref_alt,1.8);
    
    nh.param("led/brightness",brightness,0.5);
    
    imu_pub = nh.advertise<sensor_msgs::Imu>("/"+ugv_name+"/imu",10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/"+ugv_name+"/vel_ned",10);
    pose_pub =nh.advertise<geometry_msgs::PoseStamped>("/"+ugv_name+"/pose_ned",10);
    status_pub = nh.advertise<emaxx_ros::emaxx_status>("/"+ugv_name+"/status",10);


    joy_sub = nh.subscribe("/joy",100,joyCallBack);
    path_sub = nh.subscribe("/"+ugv_name+"/path",100,pathCallBack);
    wp_sub = nh.subscribe("/"+ugv_name+"/waypoint",10,waypointCallbBack);
    traj_sub = nh.subscribe("/"+ugv_name+"/trajectory",1,trajectoryCallBack);
    vel_sub = nh.subscribe("/"+ugv_name+"/des_vel",10,targetVelCallBack);

    std::cout << "attemping to open port " << port_name << " at: " << 57600 << std::endl;
    my_serial.setPort(port_name);
    my_serial.setBaudrate(57600);
    my_serial.setTimeout(0,10,1,10,1);
    
    my_serial.open();
    
    if(my_serial.isOpen()){
    
      ROS_INFO("Serial Port is Open!");
    }else{
     
      ROS_ERROR("Serial Port Error check settings");
    }

    sleep(1);
    sendGPSref(ref_lat,ref_lon,ref_alt);
    sleep(1);
    sendSim(sim_mode,0,0,0);
    sleep(1);
   // sendGPSref(ref_lat,ref_lon,ref_alt);                
    //sleep(3);
    sendCtrlParams(Kp_psi,Kp_spd,Ki_spd);
    sleep(1);
    sendLED(OFF,0.0);
    sleep(0.2);
    sendLED(OFF,0.0);
    sleep(0.2);
    sendLED(OFF,0.0);
    sleep(0.2);    

    ros::Timer timer = nh.createTimer(ros::Duration(0.05),runControl);
    
    path_size = 0;
    err_x = err_y = err_rate_x = err_rate_y = err_x_old = err_y_old = 0;

    ros::Rate loop_rate(100);
    while(ros::ok()){


	//std::cout << my_serial.available() << std::endl;
	if(my_serial.available() > 20){
	 std::vector<std::string> lines;
	 lines =  my_serial.readlines();
	
	    for(int i=0;i<lines.size();i++){
		parseMessage(lines[i]);
	   // std::cout << lines[i].c_str() << std::endl;
		
	    }
	}	  
		
	if(status.nav_state!=old_status.nav_state ||
	   status.nav_goal != old_status.nav_goal ||
           status.armed != old_status.armed ||
           status.ctrl_mode != old_status.ctrl_mode||
           status.nav_solution != old_status.nav_solution){ //publish status only if it has changed
           status_pub.publish(status);
        }
        old_status = status; // age status
        ros::spinOnce();// Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate
    }

    return 0;
}
