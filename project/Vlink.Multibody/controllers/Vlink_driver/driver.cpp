// File:          driver.cpp
// Date:
// Description:
// Author:
// Modifications:
constexpr double pi=3.1415926;
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <webots/utils/AnsiCodes.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>

using namespace webots;
using namespace std;
class Driver : public Supervisor {
  public:
  Driver();
  void run();
  private:
  int timeStep;
  Emitter *emitter;
  Keyboard *keyboard;
  Node *robot0;
  Node *robot1;
};
Driver::Driver() {
  timeStep=8;
  emitter=getEmitter("emitter");
  robot0=getFromDef("R0");
  robot1=getFromDef("R1");
  keyboard=getKeyboard();
  keyboard->enable(timeStep);
}
void Driver::run() {
  string message("");
	while (step(timeStep) != -1) 
  {
    int k =keyboard->getKey();
    switch(k){
      case '0':
        emitter->setChannel(0);
        message.assign("1");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        emitter->setChannel(1);
        message.assign("0");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        cout<<AnsiCodes::GREEN_FOREGROUND<<"controller set channel 0"<<AnsiCodes::RESET<<endl;
        break;
      case '1':
        emitter->setChannel(0);
        message.assign("0");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        emitter->setChannel(1);
        message.assign("1");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        cout<<AnsiCodes::GREEN_FOREGROUND<<"controller set channel 1"<<AnsiCodes::RESET<<endl;
        break;
      case '2':
        emitter->setChannel(0);
        message.assign("1");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        emitter->setChannel(1);
        message.assign("1");
        if(!message.empty()) emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
        cout<<AnsiCodes::GREEN_FOREGROUND<<"controller set broadcast"<<AnsiCodes::RESET<<endl;
        break;
      default:
      message.clear();
    }
    if(!message.empty()&&message.compare("1")!=0&&message.compare("0")!=0){
      emitter->send(message.c_str(),(int)strlen(message.c_str())+1);
      cout<<AnsiCodes::GREEN_FOREGROUND<<"sent \""<<message.c_str()<<"\" to channel "<<emitter->getChannel()<<AnsiCodes::RESET<<endl;
    }
  }
}
int main(int argc, char **argv) {
  Driver *controller = new Driver();
  controller->run();
  delete controller;
  return 0;
}
