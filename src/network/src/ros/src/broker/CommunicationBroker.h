#ifndef HIVE_CONNECT_COMMUNICATIONBROKER_H
#define HIVE_CONNECT_COMMUNICATIONBROKER_H

#include "hive_connect/Broadcast.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <map>
//Todo: add way to dynamically register and unregister robots after starting
class CommunicationBroker {
  public:
    CommunicationBroker();

  private:
    static std::vector<uint16_t> getRobotList();

    std::map<uint16_t, ros::Publisher> m_publishersMap;
    std::vector<ros::Subscriber> m_subscribersList;

    void communicationCallback(const hive_connect::Broadcast& msg);

    ros::NodeHandle m_nodeHandle;
};

#endif // HIVE_CONNECT_COMMUNICATIONBROKER_H
