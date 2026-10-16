// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <iostream>
int main(int argc,char **argv)
{
  if(argc!=2)return 1;
  using namespace eprosima::fastdds::dds;
  auto factory=DomainParticipantFactory::get_instance();
  if(factory->load_XML_profiles_file(argv[1])!=ReturnCode_t::RETCODE_OK)return 1;
  auto participant=factory->create_participant(181,PARTICIPANT_QOS_DEFAULT);
  if(!participant)return 2;
  auto publisher=participant->create_publisher(PUBLISHER_QOS_DEFAULT);
  if(!publisher)return 3;
  const auto original=publisher->get_default_datawriter_qos();
  if(original.reliability().max_blocking_time.seconds!=0 ||
     original.reliability().max_blocking_time.nanosec!=100000000)return 4;
  for(const auto *name:{"calib_container","coord_container","detection_container",
      "tracking_container","pixel_to_3d_container","fastlio_integration_container"}) {
    auto qos=original;
    if(publisher->get_datawriter_qos_from_profile(std::string("rr/")+name+"/_container/load_nodeReply",qos)!=ReturnCode_t::RETCODE_OK)return 5;
    if(qos.reliability().max_blocking_time.seconds!=2 || qos.reliability().max_blocking_time.nanosec!=0)return 6;
    // The full QoS may differ only in the explicitly named reliability policy.
    auto expected=original;expected.reliability()=qos.reliability();
    if(!(qos==expected))return 7;
  }
  participant->delete_publisher(publisher);factory->delete_participant(participant);
  std::cout<<"six component reply profiles=2s; default writer=100ms; other QoS unchanged\n";
}
