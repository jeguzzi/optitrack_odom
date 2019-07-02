/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "ros/ros.h"
#include "optitrack2odom/RigidBodies.h"
#include "optitrack_msgs/RigidBodies.h"
#include <map>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"


//#define HISTORY_LENGTH 10
#define MAX_SPEED 10 //m/s
#define RELIANCE_DURATION 1 //seconds
#define MAX_NOT_TRACKED_TIME 0.1
//TODO: make it a real filter to be later fused with other odom sources using ros extended kalman filter package(s)

static double deltaTime;
static int history_length=10;
static std::map<std::string,std::string> markerFrames;
static bool publish_rigid_bodies= false;
static std::string frame_id;

static std::string sanitize(std::string name)
{
        std::string s = name;
        std::replace(s.begin(), s.end(), ' ', '_');
        std::remove(s.begin(), s.end(), '~');
        std::remove(s.begin(), s.end(), '/');
        return s;
}


bool odomMsgIsValid(const nav_msgs::Odometry &msg) {
  // should have a frame id
  if ( msg.header.frame_id == "" ) return false;

  // are there any nan

  if ( isnan(msg.pose.pose.position.x) ) return false;
  if ( isnan(msg.pose.pose.position.y) ) return false;
  if ( isnan(msg.pose.pose.position.z) ) return false;

  if ( isnan(msg.pose.pose.orientation.x) ) return false;
  if ( isnan(msg.pose.pose.orientation.y) ) return false;
  if ( isnan(msg.pose.pose.orientation.z) ) return false;
  if ( isnan(msg.pose.pose.orientation.w) ) return false;

  // is the quaternion unitary?
  float n = pow(msg.pose.pose.orientation.w, 2) + pow(msg.pose.pose.orientation.y, 2)
      + pow(msg.pose.pose.orientation.x, 2) + pow(msg.pose.pose.orientation.z, 2);
  if ( n < 0.99 || n > 1.01 ) return false;

  return true;
}


class Body
{

private:

uint32_t seq;
geometry_msgs::Pose pose;
geometry_msgs::Twist velocity;
ros::Time timeStamp;
ros::Time lastTimeWhenUnreliable;
std::deque<tf::Stamped<tf::Vector3> > position_h;
std::deque<tf::Stamped<tf::Quaternion> > orientation_h;
int id;
ros::Subscriber odomSubscriber;
std::string odomFrame;
tf::Transform inverseOdomTransform;
tf::TransformBroadcaster br;
std::string markerFrame;

public:

ros::Publisher odometryPublisher;
bool tracked;
std::string name;

Body()
{
}

Body(int _id,std::string _name)
{
        seq=0;
        odomFrame="";
        ros::NodeHandle nh;
        id=_id;
        name=sanitize(_name);

        std::cout<<name<<" "<<markerFrames.count(_name)<<std::endl;
        //std::cout<<"thymio"<<" "<<markerFrames.count("thymio")<<std::endl;
        if(markerFrames.count(_name)>0)
        {
                markerFrame=markerFrames[_name];
        }
        else
        {
                markerFrame=name;
        }
        ros::AdvertiseOptions ad_options = ros::AdvertiseOptions::create<nav_msgs::Odometry>("/"+markerFrame+"/mocap_odom", 1, NULL,NULL, ros::VoidPtr(), NULL);
        ad_options.has_header = false;
        odometryPublisher=nh.advertise(ad_options);
        std::string topic=markerFrame+"/odom";
        std::cout<<"subscribe to odom at "<<topic<<std::endl;
        odomSubscriber=nh.subscribe(topic,1,&Body::updateOdometryFromMessage,this);
};


void publishOdom()
{
        nav_msgs::Odometry msg=odometryMsg();
        msg.header.seq=seq;
        msg.header.stamp=ros::Time::now();
        //msg.header.stamp=ros::Time(rigidBodiesMsg.fTimestamp);
        seq++;
        odometryPublisher.publish(msg);
};

void publishTF()
{
        tf::Pose tf_pose;
        tf::poseMsgToTF(pose,tf_pose);
        tf::StampedTransform st;
        std::string child_frame_id;
        //std::cout<<"odom frame: "<<odomFrame<<std::endl;
        if(!odomFrame.empty())
        {
                //std::cout<<"pub optitrack to odom"<<std::endl;
                tf_pose=tf_pose*inverseOdomTransform;

                child_frame_id=odomFrame;
        }
        else{
                child_frame_id=markerFrame; // "/base_link"; //
        }
        st=tf::StampedTransform(tf_pose,ros::Time::now(),frame_id,child_frame_id);
        br.sendTransform(st);
};


nav_msgs::Odometry odometryMsg()
{
        nav_msgs::Odometry msg;
        msg.header.frame_id=frame_id;

        //msg.child_frame_id=tf::resolve(tf_prefix,"base_link");
        //CHANGED: now Twist in World frame
        msg.child_frame_id=frame_id;//"base_link";//markerFrame;
        msg.pose.pose=pose;
        msg.twist.twist=velocity;
        return msg;
};


optitrack2odom::RigidBodyData msg()
{
        optitrack2odom::RigidBodyData data;
        data.id = id;
        data.name = name;
        // TODO:  data.mean_error =0.0;
        data.pose = pose;
        data.velocity=velocity;
        return data;
};


void updateTrackingStatus(ros::Time new_timeStamp)
{
        if((new_timeStamp-timeStamp).toSec()>MAX_NOT_TRACKED_TIME ||
           (new_timeStamp-lastTimeWhenUnreliable).toSec()<RELIANCE_DURATION)
        {
                //printf("%.1f %.1f %.1f\n",timeStamp,o.timeStamp,o.lastTimeWhenUnreliable);
                tracked=false;
        }
        else
        {
                tracked=true;
        }
};

void updatePosition(geometry_msgs::Pose new_pose,double mean_error,ros::Time new_timeStamp)
{
        //ROS_INFO("update %s at %.3f s",name.c_str(),new_timeStamp.toSec());
        geometry_msgs::PointStamped p;
        p.header.stamp=new_timeStamp;
        p.point=new_pose.position;
        geometry_msgs::QuaternionStamped q;
        q.header.stamp=new_timeStamp;
        q.quaternion=new_pose.orientation;

        tf::Stamped<tf::Quaternion> orientation,oldOrientation;
        tf::quaternionStampedMsgToTF(q,orientation);

        tf::Stamped<tf::Point> position,oldPosition;
        tf::pointStampedMsgToTF(p,position);

        position_h.push_back(position);
        orientation_h.push_back(orientation);

        int current_history_length=position_h.size();

        if(position_h.size()>1)
        {
                oldOrientation=orientation_h.front();
                oldPosition=position_h.front();
                double dt=(position.stamp_-oldPosition.stamp_).toSec();
                //ROS_INFO("dt: %.5f",dt);
                while(position_h.size()+1>history_length)
                {
                        position_h.pop_front();
                        orientation_h.pop_front();
                }
                //ROS_INFO("oy %.5f (%.5f ) ny %.5f (%.5f)",oldPosition.y(),oldPosition.stamp_.toSec(),position.y(),position.stamp_.toSec());
                tf::Vector3 velocityVector=(position-oldPosition)/dt;
                double speed=velocityVector.length();
                //ROS_INFO("speed %.5f",speed);
                if(speed>MAX_SPEED)
                {
                        //The optitrack information about this rigid body is not reliable
                        lastTimeWhenUnreliable=new_timeStamp;
                }


                //TODO
                tf::vector3TFToMsg(velocityVector,velocity.linear);
                tf::Quaternion diff=(orientation-oldOrientation)/dt;
                //tf::Quaternion angularVelocityQuaternion=orientation.inverse()*diff*2.0;
                // CHANGED: world frame
                tf::Quaternion angularVelocityQuaternion=(diff*2.0)*orientation.inverse();

                tf::Vector3 angularVelocityVector=tf::Vector3(
                        angularVelocityQuaternion.getX(),
                        angularVelocityQuaternion.getY(),
                        angularVelocityQuaternion.getZ());
                //tf::Vector3 angularVelocityVector=tf::Vector3(angularVelocityQuaternion);
                //geometry_msgs::Quaternion quaternionMsg;
                tf::vector3TFToMsg(angularVelocityVector, velocity.angular);
                //
                // b.velocity.angular.x=angularVelocityQuaternion.dot(tf::Quaternion(1,0,0,0));//quaternionMsg.x;
                // b.velocity.angular.y=angularVelocityQuaternion.dot(tf::Quaternion(0,1,0,0));//quaternionMsg.y;
                // b.velocity.angular.z=angularVelocityQuaternion.dot(tf::Quaternion(0,0,1,0));//quaternionMsg.z;
        }
        else{
                velocity=geometry_msgs::Twist();
        }
        pose=new_pose;
        timeStamp=new_timeStamp;
};



void updateOdometryFromMessage(const nav_msgs::Odometry &msg) {
    if (!odomMsgIsValid(msg)) {
      ROS_WARN("Odometry msg is not valid: ignore it.");
      return;
    }

    odomFrame=msg.header.frame_id;
    //std::cout<<"got odom from "<<std::endl;
    //std::cout<<name<<std::endl;
    //std::cout<<"frame "<<odomFrame<<std::endl;
    tf::Transform t;
    tf::poseMsgToTF(msg.pose.pose,t);
    inverseOdomTransform=t.inverse();
}


};

static std::map<int,Body*> bodies;
static ros::Publisher rigidBodiesPublisher;

static void updateRigidBodiesFromMessage(const optitrack_msgs::RigidBodies &msg)
{

        static std::deque<ros::Time> stamps;

        std::vector<optitrack_msgs::RigidBodyData>::const_iterator it=msg.rigid_bodies.begin();
        ros::Time timeStamp=msg.header.stamp;

        //ROS_INFO("%d %.4f",msg.header.seq,timeStamp.toSec());

        stamps.push_back(timeStamp);

        if(stamps.size()==history_length)
        {
                double dt=(timeStamp-stamps.front()).toSec();
                int target_history_length=(int)(round((history_length-1.0)*deltaTime/dt+1.0));


                if(target_history_length!=history_length && target_history_length>1)
                {
                        if(3*target_history_length>4*history_length || 4*target_history_length < 3*history_length)
                        {
                                //ROS_INFO("hl %d, dt %.3f -> %d (%.3f)",history_length,dt,target_history_length,deltaTime);
                                history_length=target_history_length;
                        }
                }
                while(stamps.size()+1>history_length) stamps.pop_front();
        }


        //if(timeStamp.toSec()==0) timeStamp=ros::Time::now();
        //printf("%.1f\n",msg.fTimestamp);



        for(; it!=msg.rigid_bodies.end(); it++)
        {
                if(!bodies.count(it->id))
                {
                        Body *b=new Body(it->id,it->name);
                        //printf("Created body %d %p\n",it->id,b);
                        bodies[it->id]=b;
                        //.insert(std::make_pair(it->id,Body(it->id,it->name)));
                }

                if(it->number_of_visible_markers>2)
                {
                        //Tracked
                        Body *b=bodies[it->id];
                        b->updatePosition(it->pose,it->mean_error,timeStamp);
                }
        }
        std::map<int,Body*>::const_iterator o_it=bodies.begin();
        for(; o_it!=bodies.end(); o_it++)
        {
                Body *b=o_it->second;
                b->updateTrackingStatus(timeStamp);
        }
}


static void timerCallback(const ros::TimerEvent& event)
{
        std::map<int,Body*>::const_iterator it=bodies.begin();
        static uint32_t seq=0;
        optitrack2odom::RigidBodies rigidBodiesMsg;
        rigidBodiesMsg.header.stamp=ros::Time::now();
        rigidBodiesMsg.header.seq=seq;
        //rigidBodiesMsg.fTimestamp = ros::Time::now().toSec();
        for(; it!=bodies.end(); it++)
        {
                Body *b=it->second;
                if(b->tracked)
                {
                        rigidBodiesMsg.rigid_bodies.push_back(b->msg());
                        b->publishOdom();
                        b->publishTF();
                }
                else
                {
                        //std::cout<<b->name<<std::endl;
                }
        }
        seq++;
        rigidBodiesPublisher.publish(rigidBodiesMsg);
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "optitrack2odom_node");
        ros::NodeHandle n("~");

        ros::Subscriber sub=n.subscribe("/optitrack/rigid_bodies",10,updateRigidBodiesFromMessage);
        n.getParam("marker_frame",markerFrames);

        for(std::map<std::string, std::string>::const_iterator it = markerFrames.begin();
            it != markerFrames.end(); ++it)
        {
                std::cout << it->first << " " << it->second <<  "\n";
        }


        n.param("delta_time",deltaTime,0.1);
        double rate;
        n.param("rate",rate,10.0);
        n.param("frame_id", frame_id, std::string("World"));
        //n.param("publish_rigid_bodies", publish_rigid_bodies, false);
        //
        ROS_INFO("filtered optitrack initialized with rate %.1f and deltaTime %.4f",rate,deltaTime);

        // if(publish_rigid_bodies)
        // {
        rigidBodiesPublisher=n.advertise<optitrack2odom::RigidBodies>("/filtered_optitrack/rigid_bodies",1);
        // }
        //  ROS_INFO("Here");

        ros::Timer timer = n.createTimer(ros::Duration(1.0/rate), timerCallback);
        ros::spin();
}
