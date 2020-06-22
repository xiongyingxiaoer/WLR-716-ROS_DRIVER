#include <ros/ros.h>
#include "async_client.h"
#include "wj_716_lidar_protocol.h"
using namespace wj_lidar;

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */
wj_716_lidar_protocol *protocol;
Async_Client *client;
void CallBackRead(const char* addr,int port,unsigned char* data,const int len)
{
  protocol->dataProcess(data,len);
}

void callback(wj_716_lidar::wj_716_lidarConfig &config,uint32_t level)
{
  protocol->setConfig(config,level);
}

void timerCallback(const ros::TimerEvent&)
{
unsigned char scanmodeset1[34]= {0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,
                           0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01,
                           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,
                           0xEE,0xEE};

//  //cout << "20ms" <<endl;
 client->sendMsg(scanmodeset1,34);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "wj_716_lidar_01");
  ros::NodeHandle nh("~");
  ros::Timer timer;
  std::string hostname;
  nh.getParam("hostname",hostname);
  std::string port;
  nh.getParam("port",port);
  cout << "laser ip: " << hostname << ", port:" << port <<endl;

  protocol = new wj_716_lidar_protocol();
  dynamic_reconfigure::Server<wj_716_lidar::wj_716_lidarConfig> server;
  dynamic_reconfigure::Server<wj_716_lidar::wj_716_lidarConfig>::CallbackType f;
  f = boost::bind(&callback,_1,_2);
  server.setCallback(f);

  client = new Async_Client(&CallBackRead);
  timer= nh.createTimer(ros::Duration(2), timerCallback);

  protocol->heartstate = false;
  while(!client->m_bConnected)
  {
    ROS_INFO("Start connecting laser!");
    if(client->connect(hostname.c_str(),atoi(port.c_str())))
    {
      ROS_INFO("Succesfully connected. Hello wj_716_lidar!");
    }
    else
    {
      ROS_INFO("Failed to connect to laser. Waiting 5s to reconnect!");
    }
    ros::Duration(5).sleep();
  }
  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(2).sleep();
    if(client->m_bConnected)
    {
      if(protocol->heartstate)
      {
        protocol->heartstate = false;
      }
      else
      {
        client->m_bConnected = false;
      }
    }
    else
    {
      //reconnect
      if(!client->m_bReconnecting)
      {
        boost::thread t(boost::bind(&Async_Client::reconnect, client));
      }
    }
  }
}
