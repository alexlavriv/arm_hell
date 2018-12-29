#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>  
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <iterator>
#include <stdlib.h>  
#include <boost/thread.hpp>

using namespace std;
const int distance_arr_size = 360;
# define TAU           6.283185307  /* 2*pi */
# define e         2.71828182846


float distanceFromObsticle = 500;
float distanceToMove = 0.5;
float speed = 0.1;
float degree = 0.0174532925;
float fov = 1.3962634;
int fovDeg = 80;
int picture_width = 800;
int userColor;
int theXCord = 0;
float distanceFromObject = 0;
float distancesArray[360];
ros::Subscriber scan_sub;
ros::Subscriber cam_sub;
ros::Publisher movement_pub;

int call_back_counter = 0;


const int BLUE = 0;
const int GREEN = 1;
const int RED = 2;

void identifyObject(ros::NodeHandle n);
void cameraCallBack(const sensor_msgs::ImageConstPtr& msg);
void findDistanceFromObject(ros::NodeHandle n);
void moveForward(ros::NodeHandle n);
void turn(ros::NodeHandle n, float degrees);
void stopRobot(ros::NodeHandle n, geometry_msgs::Twist t);
void searchForObject(ros::NodeHandle n);
void turnRobot(ros::NodeHandle n, float radians);
string getColorFromUser();
float getDegreeFromX();
void moveToObject(ros::NodeHandle n);
float calculteSpeedByTime(double x, float distance);
float getVelocity(double t);
void turnDistance (ros::NodeHandle n, bool left, double distance);
class stack {
    const static int  size = 4;
  float values[size];
    public:
  stack(){
    for (int i = 0; i < size; ++i)
    {
      values[i] = 0;
    }
  }
  void push (float value){
    moveStackLeft();
    values[size-1] = value;
  }
  float sumofLast (int indexes){
    float sum = 0;
    for (int i = size - 1; i >= indexes; --i)
    {
      sum += values[i];
    }
    return sum;
  }
    private: 
  void moveStackLeft(){
    for (int i = 1; i < size; ++i)
    {
      values[i-1] = values[i];
    }
  }
};


void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){

  distanceFromObsticle = msg->ranges[0];
  for (int i = 0; i < distance_arr_size; ++i){
    if (((i <= 5) || (i >= 355)) && (msg->ranges[i] < distanceFromObsticle))
    {
      distanceFromObsticle = msg->ranges[i];
    }
    distancesArray[i] = msg->ranges[i];
  }
  
}
int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_menu");

    ros::NodeHandle n;
    int userInput;
  scan_sub = n.subscribe("scan", 100, scanCallBack);
  cam_sub = n.subscribe("/camera/image_raw", 100, cameraCallBack);
  movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  int c = 10;
    std::cout << "The message was published\n";
  while (userInput != 6){

    std::cout << "mashu\n";
    std::cout << "1. Move forward\n";
    std::cout << "2. Turn around\n";
    std::cout << "3. Distance to object with color X\n";
    std::cout << "4. Find object with color X\n";
    std::cout << "5. Send stop\n";
    std::cout << "6. Exit \n";
    std::cin >> userInput;

  switch(userInput) {
      case 1  :
        moveForward(n);
        break; //optional
      case 2  :
      { 
      float degrees = 0;
      printf("Enter the angle (deg)\n");
      std::cin >>degrees;
        turn(n,degrees);
        break; //optional
      }
    case 3 : 
      findDistanceFromObject(n);
      break;

    case 4 : 
    {
      getColorFromUser();
      searchForObject(n);
      break;
    }

    }
  }
    return 0;
 }

void findDistanceFromObject(ros::NodeHandle n){

  string colorName = getColorFromUser();
  printf("after if the userColor %d\n",userColor);
  ros::Rate r(40); // 10 hz
  while (theXCord == 0){
  ros::spinOnce();
  r.sleep();
  }
  printf("The X cord is %d\n", theXCord);
  int nIndex = 0;
  float fIndex = getDegreeFromX();
  if (fIndex > 0) {
    nIndex = static_cast<int>(fIndex);
  }else{
    nIndex = distance_arr_size + static_cast<int>(fIndex);
  }
  //printf("the index is %d \n", nIndex );
  //printf("The distance fron %s object is %f \n", colorName.c_str(), distancesArray[nIndex]);
  theXCord = 0;


    // get the picture
    // get the x from the picture
    // calculte the angle
    // get the distance from the array
 }
float getDegreeFromX(){
  float m = -(float) fovDeg / (float)picture_width ;
  float c =  (float)fovDeg / 2;

  float fIndex = m*theXCord + c;
  printf("the m is %f, the x is %d, the c is %f\n", m, theXCord, c);
  printf("the fIndex %f\n", fIndex);
  return fIndex;
}

bool isRed(int * pixel){
    return ((pixel[2] > 0) && (pixel[2] > pixel[1])
      && ( pixel[0]< pixel[2])
      
      );
}

bool isGreen(int * pixel){
     return ((pixel[1] > 0) && (pixel[0]< pixel[1])
      && ( pixel[2] < pixel[1])
      
      );
}

bool isBlue(int * pixel){
     return ((pixel[0] > 0) && (pixel[1]< pixel[0])
      && ( pixel[2] < pixel[0])
      
      );
}

int getColor(int * pixel){
  if (isRed(pixel)){
    return RED;
  }
  else if (isGreen(pixel)){
    return GREEN;
  }
  else if (isBlue(pixel)){
    return BLUE;
  }
  return -1;
}

void cameraCallBack(const sensor_msgs::ImageConstPtr& msg){

  //boost::thread::id id = boost::this_thread::get_id();
  //cout<<"Thread "<<id<<" in camera call back."<<endl;

  cv::Mat display = cv_bridge::toCvShare(msg, "bgr8")->image;

  int rows = display.rows;
  int cols = display.cols;

  cv::Size s = display.size();
  rows = s.height;
  cols = s.width;

  int output_array[rows*cols] = {0};

  int user_choice = userColor;
  int color;
  int k = 0;
  for (int i = 0; i < rows; ++i){
    for (int j = 0; j < cols; ++j){
      int pixel[3] = {
        display.at<cv::Vec3b>(i,j)[0],
        display.at<cv::Vec3b>(i,j)[1],
        display.at<cv::Vec3b>(i,j)[2]
      };
         
      color = getColor(pixel);
     
      if (color == user_choice){
        output_array[k++] = j;
          //printf("B %u G %u R %u\n", pixel[0], pixel[1], pixel[2] );
          //printf("adding cord color %d %d \n", i,j);
          // int answer[2] = {i, j};
          // return &answer;
      }
    }
  } 
  // if(k>0){
  //    printf("found X %d, the k is %d\n", output_array[k/2], k);
  // }

    theXCord = output_array[k/2];

  if (theXCord == 0){
    theXCord = -1; 
  }
  call_back_counter++;
  if (call_back_counter % 20 == 0){
    cout<<"x cord is: "<<theXCord<<endl;
  }
}

void stopRobot(ros::NodeHandle n, geometry_msgs::Twist t){
   
  int c=10;
  ros::Rate r(10);

     /* generate secret number between 1 and 10: */

  t.linear.x = speed;
  //movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  // while (c){
  //  if (t.linear.x >= -0.04){
  //    t.linear.x -=0.02;
  //  }
  //  t.angular.z =0;
  //  movement_pub.publish(t);
  //  ros::spinOnce();
  //  c--;
  //  r.sleep();
  // }
  int k = 10;
 
  while (k){
   
    r.sleep();
    t.linear.x = 0.0;
    t.angular.z = 0;
    movement_pub.publish(t);
    ros::spinOnce();
    k--;
  }
 
  k=10;
  c=10;
 
}
void moveForward(ros::NodeHandle n){

  ros::spinOnce();

  if (distanceFromObsticle > distanceToMove){
     
    geometry_msgs::Twist t;
    t.linear.x = speed;
    t.angular.z = 0;
    //ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate r(40);
    ros::Time start = ros::Time::now();
      while(ros::Time::now() - start < ros::Duration(7.0))
      {
        double time = (ros::Time::now() - start).toSec();
        t.linear.x = calculteSpeedByTime(time,0.5);
        printf("Sending move message, the speed is %f \n", t.linear.x);
        if ( t.linear.x < 0.01) {
          t.linear.x = 0;
        }
          movement_pub.publish(t);

          ros::spinOnce();
          r.sleep();
      }
   
  }else{
    printf("Cant move, the distance from obsticle is %f \n", distanceFromObsticle );
  }
 }

void turnDistance (ros::NodeHandle n, bool left, double distance){
  float turnSpeed = 0.2;
  int degrees = 1;
  float radians = degrees * degree;

   
    
  float dur = radians / turnSpeed;
  geometry_msgs::Twist t;
 
    if (dur < 0){
      dur = dur * -1;
    }
    if (left){
      t.angular.z = turnSpeed;
    }else{
      t.angular.z = -turnSpeed;
    } 
  
 
  ros::Rate r(40);
  ros::Time start = ros::Time::now();
  while(distanceFromObsticle > distance){
     
    movement_pub.publish(t);

    ros::spinOnce();
    r.sleep();
    }
   
   

  stopRobot(n, t);
}

void turn(ros::NodeHandle n, float degrees){

 
  float turnSpeed = 0.2;
  float radians = degrees * degree;

   
    
  float dur = radians / turnSpeed;
  geometry_msgs::Twist t;
 
    if (dur < 0){
      dur = dur * -1;
    } 
  t.angular.z = -turnSpeed;
 
  ros::Rate r(40);
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start < ros::Duration(dur)){
     
    movement_pub.publish(t);

    ros::spinOnce();
    r.sleep();
    }
   
   

  stopRobot(n, t);
 
 }
void turnRobot(ros::NodeHandle n, float radians){
  theXCord = 0;
  float turnSpeed = 0.5;
  float dur = (radians + TAU) / turnSpeed;
  if (radians > TAU/2){
    radians = radians - TAU;
  }
  geometry_msgs::Twist t;
  
  t.angular.z = -turnSpeed;
  //ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate r(40);
  ros::Time start = ros::Time::now();
  while((ros::Time::now() - start < ros::Duration(dur)) 
    && ((theXCord <=0 ) || ((theXCord > 50) && (theXCord < 750)))){
    
    movement_pub.publish(t);

    ros::spinOnce();
    r.sleep();
  } 
  cout<<"stopping the turn, the X is" << theXCord<<endl;
  stopRobot(n,t); 
}

std::string getColorFromUser (){

  std::string colorName;
  printf("Enter color name\n");
  std::cin >> colorName;
  printf("before if\n");
  if (colorName.compare("green") == 0){
    userColor = GREEN;
  } else
  if (colorName.compare("blue") ==0){
    userColor = BLUE;
  }else
  if (colorName.compare("red") ==0){
    userColor = RED;
  }
  return colorName;
}

void searchForObject(ros::NodeHandle n){

  cout<<"in searchForObject"<<endl;
  theXCord = 0;
  ros::spinOnce();
  int randomNumber = rand() % 3;
  stack angleStack;
  float minDistance = 1.0;
  float angleCandidat;
  bool shouldGenerateAgain = true;
  float mu = 3.0;


  geometry_msgs::Twist t;
  //t.linear.x = speed;
  t.angular.z = 0;
  //ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  //geometry_msgs::Twist zero;
  //ero.linear.x = 0;
  ros::Rate r(40);
  ros::spinOnce();
  turnRobot(n,0);
  double currentTime = 0;
  double savedTime = 0;
     ros::Time start;
     bool isSavedTime  = false;
     bool theBotTurned = true;
     
  while (theXCord <= 0){
    if (theBotTurned){
       isSavedTime = false;
      start = ros::Time::now();
      theBotTurned = false;
    }
    printf("The distance from obsticle is %f\n", distanceFromObsticle);
    if (minDistance  < distanceFromObsticle){
      
      currentTime = (ros::Time::now() - start).toSec();
    
      // constanst speed
      if ( currentTime > mu){
        cout<<"In constanst speed"<<endl;
        if (!isSavedTime){
          savedTime = currentTime;
          isSavedTime = true;
        }
        t.linear.x = getVelocity(savedTime);
      }
      else{ // acceleration
        cout<<"In acceleration speed"<<endl;
        savedTime = currentTime;
        t.linear.x = getVelocity(currentTime);
        
      }

      movement_pub.publish(t);//pub 
    }
    r.sleep();  
    ros::spinOnce();
    printf("The distance is %f \n", distanceFromObsticle);
      if (distanceFromObsticle < minDistance){

      if (t.linear.x > 0){
      
          cout<<"In stoping, the t.linear.x "<< t.linear.x <<endl;
          cout<<"In stoping, the savedTime "<< savedTime <<endl;
        

          savedTime = mu - savedTime;
          if (savedTime < 0)
          {
            savedTime =0 ;
          }
          cout<<" the updated savedTime "<< savedTime <<endl;
          start = ros::Time::now();
          currentTime = (ros::Time::now() - start).toSec() ;
          while (currentTime < mu - savedTime +1){
                  cout<<" mu "<< mu <<endl;
                  cout<<" currentTime "<< currentTime <<endl;
                  cout<<" savedTime "<< savedTime <<endl;
            t.linear.x = getVelocity(mu + currentTime + savedTime);
            if ( t.linear.x < 0.01) {
              t.linear.x = 0;
              }
            movement_pub.publish(t);
            currentTime = (ros::Time::now() - start).toSec() ;
            r.sleep();
          }
    }


      printf("generating angle");
      shouldGenerateAgain = true;

      while (shouldGenerateAgain){
        randomNumber = rand() % 3 + 1;
        angleCandidat = (TAU / 4) * randomNumber;
          if (angleStack.sumofLast(1) + angleCandidat == TAU){
          shouldGenerateAgain = true;
          }else if (angleStack.sumofLast(2) + angleCandidat == TAU){
          shouldGenerateAgain = true;
          }else if (angleStack.sumofLast(3) + angleCandidat == TAU){
            shouldGenerateAgain = true;
          }else {
          shouldGenerateAgain = false;
          }
        }
        angleStack.push(angleCandidat);
        turnRobot(n,angleCandidat);
        theBotTurned = true;
      }
  }
  printf("exited find while, moving to object, the x cord is %f \n", theXCord);
  moveToObject(n);

  // //stop

  //move
}

void moveToObject(ros::NodeHandle n){
  cout<<"in move to object"<<endl;
  float degrees = getDegreeFromX();
  int xBeforeTurn = theXCord;
  cout<< "turning to the object" << degrees << endl;
  turn(n,degrees);
  cout<<"moving towards object..."<<endl;
  // turn back
  if (distanceFromObsticle < 1.0){
    if (degrees > 0 ){
      turnDistance(n,true, 1.0);
    }
    else
    {
      turnDistance(n,false, 1.0); 
    }
  }
  ros::spinOnce();
  geometry_msgs::Twist t;
  //t.linear.x = speed;
  t.angular.z = 0;
     double currentTime = 0;
     double savedTime = 0;
     ros::Time start ;
     bool isSavedTime  = false;
     bool theBotTurned = true;
      float mu = 3.0;
  ros::Rate r(40);
  cout<<"the x is: "<<theXCord<<endl;
  cout<<"the distance is: "<<distanceFromObsticle<<endl;
  start = ros::Time::now();
  while((distanceFromObsticle > 1.0) && (theXCord > 0))
  {
    //printf("Sending move message\n");
    cout<<"moving... the x is: "<<theXCord<<endl;
    if(theXCord == -1){break;}
      currentTime = (ros::Time::now() - start).toSec();
      // constanst speed
      if ( currentTime > mu){
        cout<<" moveToObject : In constanst speed"<<endl;
        if (!isSavedTime){
          savedTime = currentTime;
          isSavedTime = true;
        }
        t.linear.x = getVelocity(savedTime);
      }
      else{ // acceleration
        cout<<"moveToObject : In acceleration speed"<<endl;
        savedTime = currentTime;
        t.linear.x = getVelocity(currentTime);
        
      }

      movement_pub.publish(t);//pub 



    ros::spinOnce();
    r.sleep();
  }
  cout<<"### exited while, distance from obsticle is: "<<distanceFromObsticle<<" x is: "<<theXCord<<endl;
          if (t.linear.x > 0){
      
          cout<<"moveToObject : In stoping, the t.linear.x "<< t.linear.x <<endl;
          cout<<"moveToObject :In stoping, the savedTime "<< savedTime <<endl;
        

          savedTime = mu - savedTime;
          if (savedTime < 0)
          {
            savedTime =0 ;
          }
          cout<<"moveToObject: the updated savedTime "<< savedTime <<endl;
          start = ros::Time::now();
          currentTime = (ros::Time::now() - start).toSec() ;
          while (currentTime < mu - savedTime +1){
                  cout<<"moveToObject: mu "<< mu <<endl;
                  cout<<"moveToObject: currentTime "<< currentTime <<endl;
                  cout<<"moveToObject: savedTime "<< savedTime <<endl;
            t.linear.x = getVelocity(mu + currentTime + savedTime);
            if ( t.linear.x < 0.01) {
              t.linear.x = 0;
              }
            movement_pub.publish(t);
            currentTime = (ros::Time::now() - start).toSec() ;
            r.sleep();
          }
    }
  //stopRobot(n, t);
    cout<<"*** before recursive calls "<<endl;
    cout<<"*** theXCord is " << theXCord <<endl;
    cout<<"*** distanceFromObsticle is " << distanceFromObsticle <<endl;
  if (theXCord == -1){
    cout<<"calling recursive searchForObject" << endl;
    searchForObject(n);
  }
  if (distanceFromObsticle > 1.0){
      cout<<"calling recursive moveToObject" << endl;
    moveToObject(n);

  }
  cout<<"Success!!"<<endl;
}

float calculteSpeedByTime(double t, float distance){
  float k = 1 / (2*sqrt(TAU));
 
  float mu = 3.0;
  float p = -pow((t - mu),2)/4;
  float res = k* pow(e, p);
  printf("The time is %f ", t);
  cout<<" the speed is " << res << endl;
  return distance * res;

}
// 1/sqrt(2pi)*e^(-(x-2))
float getVelocity(double t){
  
  float k = 1 / (2*sqrt(TAU));
  float mu = 3.0;
  float p = -pow((t - mu),2)/4;
  float res = k* pow(e, p);
  printf("The time is %f ", t);
  cout<<" the speed is " << res << endl;
  return res;

}
