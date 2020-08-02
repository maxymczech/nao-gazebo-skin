#include "ContactPlugin.hh"
#include <memory>
#include <string>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

ContactPlugin::ContactPlugin() : SensorPlugin(){
}

ContactPlugin::~ContactPlugin(){
  char m[] = "Shutting down...";
  SendUDP(m);
  close(this->sockd);
}

void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor){
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated( boost::bind(&ContactPlugin::OnUpdate, this) );

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  this->InitUDP();
  
  std::string parent_sensor_name = (this->parentSensor)->ParentName().substr(10, std::string::npos);
  std::cout << "ContactPlugin runnning." << "\n" << "Parent sensor name: " << parent_sensor_name << "\n";
  
  // ROS interface
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_contact_plugin_publisher");
  ros::NodeHandle n;
  this->chatter_pub = n.advertise<std_msgs::String>("/gazebo_contact_info/" + parent_sensor_name, 1);
  //ros::spin(); //Seems like we don't need this here
}

void ContactPlugin::OnUpdate(){
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  std::set<std::pair<std::string, std::string>> measuredSet;
  std::string str_out = "none";
  //std::cout << "Contact size: " << contacts.contact_size() << "\n";
  for (unsigned int i = 0; i < contacts.contact_size(); ++i){
    std::string col1 = contacts.contact(i).collision1();
    std::string col2 = contacts.contact(i).collision2();

    if(measuredSet.count(std::make_pair(col1, col2)) == 0){
      common::Time msgTime = msgs::Convert(contacts.contact(i).time());
      std::string col;

      //Get real time
      struct timespec realTime;
      clock_gettime(CLOCK_REALTIME, &realTime);
      char c[2048];

      //Name of parent link
      std::stringstream ss;
      std::string delimiter = "::";
      std::string s = (this->parentSensor)->ParentName();

      //Find collision name of this taxel
      col = "none";
      if(col1.find(s)){
        col = col1;
      }
      else if(col2.find(s)){
        col = col2;
      }
      else{
        std::cout << "Colliding segment not found!" << "\n";
      }
      str_out = col;
	
      ss << col << "::" << msgTime.FormattedString() << "::" << realTime.tv_sec << "::" << realTime.tv_nsec;

      //Print collision
      //std::cout << "[col " << i << "/" << contacts.contact_size() << "]\n" << col << "\n" << col1 << "\n" << col2 << "\n\n";
      //std::cout << col1 << "::" << col2 << "::" << msgTime.FormattedString() << "::" << realTime.tv_sec << "::" << realTime.tv_nsec << "\n";
      std::string ts = ss.str();
      strcpy(c, ts.c_str());

      this->SendUDP(c);
      measuredSet.insert(std::make_pair(col1, col2));
    }
  }
  
  //Publish str_out to ROS topic
  std_msgs::String msg;
  msg.data = str_out;
  this->chatter_pub.publish(msg);
  //std::cout << "ROS message: " << str_out << "\n";
}

int ContactPlugin::InitUDP(){
  //Create UDP socket
  this->sockd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockd == -1){
    perror("Socket creation error");
    exit(1);
  }

  //Configure client address
  my_addr.sin_family = AF_INET;
  my_addr.sin_addr.s_addr = INADDR_ANY;
  my_addr.sin_port = 0;

  bind(this->sockd, (struct sockaddr*)&(this->my_addr), sizeof(this->my_addr));

  strcpy(this->buf, "UDP status OK...");

  //Set server address
  this->srv_addr.sin_family = AF_INET;
  inet_aton(IP, &(this->srv_addr).sin_addr);
  this->srv_addr.sin_port = htons(atoi(PORT));

  sendto(
    this->sockd,
    this->buf,
    strlen(this->buf) + 1,
    0,
    (struct sockaddr*)&this->srv_addr,
    sizeof(this->srv_addr)
  );
}

int ContactPlugin::SendUDP(char message[]){
  strcpy(this->buf, message);
  sendto(
    this->sockd,
    this->buf,
    strlen(this->buf) + 1,
    0,
    (struct sockaddr*)&this->srv_addr,
    sizeof(this->srv_addr)
  );
}

