// Subscriber only: no ChannelPublisher, robot control client or actuator API.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"usage: read_full_state INTERFACE OUTPUT_YAML\n";return 2;}
  try {
    std::mutex mutex;std::vector<double> q;int samples=0;
    auto last=std::chrono::steady_clock::now();
    unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);
    auto sub=std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    sub->InitChannel([&](const void* data) {
      if(!data)return;const auto& message=*static_cast<const unitree_hg::msg::dds_::LowState_*>(data);
      std::vector<double> values;
      for(size_t i=0;i<29;++i){double value=message.motor_state().at(i).q();if(!std::isfinite(value))return;values.push_back(value);}
      std::lock_guard<std::mutex> lock(mutex);q=values;++samples;last=std::chrono::steady_clock::now();
    },1);
    auto begin=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()-begin<std::chrono::seconds(10)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      std::lock_guard<std::mutex> lock(mutex);
      if(samples>=10 && std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(500) && std::chrono::steady_clock::now()-last<std::chrono::milliseconds(100)) {
        YAML::Node report;report["sdk_connected"]=true;report["full_body_state"]=true;
        report["authority_acquired"]=false;report["hardware_motion_commands_sent"]=false;
        report["source"]=std::string("rt/lowstate_subscriber_only");report["samples"]=samples;
        report["received_at_unix_s"]=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        for(double value:q)report["motor_q"].push_back(value);
        std::ofstream file(argv[2]);if(!file)throw std::runtime_error("Cannot write snapshot");file<<report<<"\n";
        std::cout<<"Captured all 29 motor positions without command transport"<<std::endl;return 0;
      }
    }
    std::cerr<<"No fresh rt/lowstate feedback"<<std::endl;return 1;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
