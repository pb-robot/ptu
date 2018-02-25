/*
   PTU-46 control node
 */

/*
Author: Gert Kanter
 */

#include <string.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>

#define RESOLUTION 0.00089762283 // in radians

class PanTiltUnit
{
   private:
      serial::Serial ptu_port_;
      double pan_;
      double tilt_;
      ros::Publisher pub_pan_;
      ros::Publisher pub_tilt_;
      bool pan_moving_;
      bool tilt_moving_;
      // TODO: mutex so that you don't update while sending data

   public:
      PanTiltUnit(ros::NodeHandle node, std::string port, uint baud, uint timeout, std::string pan, std::string tilt)
         : pan_(0.0), tilt_(0.0), tilt_moving_(true), pan_moving_(true)
      {
         ptu_port_.setPort(port);
         ptu_port_.setBaudrate(baud);
         serial::Timeout to(serial::Timeout::simpleTimeout(1000));
         ptu_port_.setTimeout(to);
         try
         {
            ptu_port_.open();
         }
         catch (std::exception &e)
         {
            ROS_ERROR("ERROR: %s", e.what());
            node.shutdown();
         }
         if (!ptu_port_.isOpen())
         {
            ROS_ERROR("Could not open serial port!");
         }
         else
         {
            pub_pan_ = node.advertise<std_msgs::Float64>(pan, 1, true);
            pub_tilt_ = node.advertise<std_msgs::Float64>(tilt, 1, true);
         }
      }
      bool portOpen()
      {
         return ptu_port_.isOpen();
      }
      void PanReceived(const std_msgs::Float64ConstPtr& msg)
      {
         if (pan_ != msg->data)
         {
            pan_ = msg->data;
            std::string send("PP" + boost::lexical_cast<std::string>((int)((double)pan_ / RESOLUTION)) + "\n");
            ptu_port_.write(send);
            ptu_port_.read(send.length());
            std::string result = ptu_port_.readline(1000, "\n");
            if (result[0] != '*')
            {
               ROS_ERROR("PTU UNEXPECTED RESPONSE: %s", result.c_str()); 
            }
            else
            {
               pan_moving_ = true;
            }
         }
      }
      void TiltReceived(const std_msgs::Float64ConstPtr& msg)
      {
         if (tilt_ != msg->data)
         {
            tilt_ = msg->data;
            std::string send("TP" + boost::lexical_cast<std::string>((int)((double)tilt_ / RESOLUTION)) + "\n");
            ptu_port_.write(send);
            ptu_port_.read(send.length());
            std::string result = ptu_port_.readline(1000, "\n");
            if (result[0] != '*')
            {
               ROS_ERROR("PTU UNEXPECTED RESPONSE: %s", result.c_str()); 
            }
            else
            {
               tilt_moving_ = true;
            }
         }
      }
      void Update()
      {
         std::string send;
         std::string result;

         if (pan_moving_)
         {
            send = "PP\n";
            ptu_port_.write(send);
            ptu_port_.readline(100, "\n");
            result = ptu_port_.readline(1000, "\n");
            if (result[0] != '*')
            {
               ROS_ERROR("PTU UNEXPECTED RESPONSE: %s", result.c_str());
            }
            else
            {
               int val;
               sscanf(result.c_str(), "%*s %*s %*s %*s %*s %d\n", &val);
               std_msgs::Float64 p;
               p.data = (double)val * RESOLUTION;
               pub_pan_.publish(p);
               if (pan_ == val) pan_moving_ = false;
            }
         }
         if (!tilt_moving_) return;
         send = "TP\n";
         ptu_port_.write(send);
         ptu_port_.readline(100, "\n");
         result = ptu_port_.readline(1000, "\n");
         if (result[0] != '*')
         {
            ROS_ERROR("PTU UNEXPECTED RESPONSE: %s", result.c_str());
         }
         else
         {
            int val;
            sscanf(result.c_str(), "%*s %*s %*s %*s %*s %d\n", &val);
            std_msgs::Float64 p;
            p.data = (double)val * RESOLUTION;
            pub_tilt_.publish(p);
            if (tilt_ == val) tilt_moving_ = false;
         }
      }
};

//main function of the node
void ptu(){
   //creating node handles for subscription/publishing and parameter server queries
   ros::NodeHandle node;
   ros::NodeHandle pnode("~"); //used to get private parameters ~x

   // setting topic names and port configuration
   std::string cmd_pan("/ptu/cmd_pan");
   std::string cmd_tilt("/ptu/cmd_tilt");
   std::string pan("/ptu/pan");
   std::string tilt("/ptu/tilt");
   std::string port("/dev/ttyUSB0");
   std::string baud("9600");
   std::string timeout("1000");
   node.getParam("cmd_pan", cmd_pan);
   node.getParam("cmd_tilt", cmd_tilt);
   node.getParam("pan", pan);
   node.getParam("tilt", tilt);
   pnode.getParam("port", port); // ~ private parameters
   pnode.getParam("baud", baud);
   pnode.getParam("timeout", timeout);

   PanTiltUnit PTU(node, port, boost::lexical_cast<uint>(baud), boost::lexical_cast<uint>(timeout), pan, tilt);

   //subscribing to pan and tilt
   if (!PTU.portOpen()) return;
   ros::Subscriber sub_pan = node.subscribe<std_msgs::Float64>(cmd_pan, 1, &PanTiltUnit::PanReceived, &PTU);  
   ros::Subscriber sub_tilt = node.subscribe<std_msgs::Float64>(cmd_tilt, 1, &PanTiltUnit::TiltReceived, &PTU);
   ros::Rate r(10); //an object to maintain specific frequency of a control loop

   ROS_INFO("SERIAL CONNECTION TO PTU INITIALIZED!");

   while(ros::ok())
   {
      ros::spinOnce();

      PTU.Update();
      //sleeping so, that the loop won't run faster than r's frequency
      r.sleep();
   }
}

//entry point of the executable
int main(int argc, char** argv)
{ 
   ros::init(argc, argv, "ptu");
   ptu();
   return 0;
}
