// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
using namespace eprosima::fastdds::dds;
using namespace std::chrono_literals;
struct Bytes : TopicDataType {
 Bytes(){setName("easyfly_native_lifetime_bytes");m_typeSize=8;m_isGetKeyDefined=false;}
 bool serialize(void* data,eprosima::fastrtps::rtps::SerializedPayload_t* p)override {std::memcpy(p->data,data,4);p->length=4;return true;}
 bool deserialize(eprosima::fastrtps::rtps::SerializedPayload_t* p,void* data)override {if(p->length!=4)return false;std::memcpy(data,p->data,4);return true;}
 std::function<uint32_t()> getSerializedSizeProvider(void*)override {return []{return 4;};}
 void* createData()override {return new uint32_t{};}
 void deleteData(void* p)override {delete static_cast<uint32_t*>(p);}
 bool getKey(void*,eprosima::fastrtps::rtps::InstanceHandle_t*,bool)override{return false;}
};
struct Listener : DataWriterListener {
 std::atomic<bool> matched{false},removing{false},delete_started{false},deleted{false},early{false};
 void on_publication_matched(DataWriter*,const PublicationMatchedStatus& s)override {
  if(s.current_count_change>0)matched=true;
  if(s.current_count_change<0){
   removing=true;auto end=std::chrono::steady_clock::now()+2s;
   while(!delete_started&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
   std::this_thread::sleep_for(30ms);early=deleted.load();
   std::cerr<<"DDS owner deleted during callback="<<early<<std::endl;
  }
 }
};
int main(){
 auto* f=DomainParticipantFactory::get_instance();
 auto* a=f->create_participant(158,PARTICIPANT_QOS_DEFAULT);auto* b=f->create_participant(158,PARTICIPANT_QOS_DEFAULT);
 if(!a||!b)return 2;
 TypeSupport type(new Bytes);type.register_type(a);type.register_type(b);
 auto* at=a->create_topic("easyfly_native_owner_lifetime",type.get_type_name(),TOPIC_QOS_DEFAULT);
 auto* bt=b->create_topic("easyfly_native_owner_lifetime",type.get_type_name(),TOPIC_QOS_DEFAULT);
 auto* sub=a->create_subscriber(SUBSCRIBER_QOS_DEFAULT);auto* pub=b->create_publisher(PUBLISHER_QOS_DEFAULT);
 if(!at||!bt||!sub||!pub)return 3;
 Listener listener;
 auto* reader=sub->create_datareader(at,DATAREADER_QOS_DEFAULT);
 auto* writer=pub->create_datawriter(bt,DATAWRITER_QOS_DEFAULT,&listener);
 if(!reader||!writer)return 4;
 auto end=std::chrono::steady_clock::now()+5s;
 while(!listener.matched&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
 if(!listener.matched)return 5;
 bool delete_ok=false;
 std::thread deleter([&]{
  auto deadline=std::chrono::steady_clock::now()+5s;
  while(!listener.removing&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(1ms);
  if(!listener.removing)return;
  listener.delete_started=true;delete_ok=pub->delete_datawriter(writer)==eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK;
  listener.deleted=true;
 });
 sub->delete_datareader(reader);deleter.join();
 bool ok=delete_ok&&listener.deleted&&!listener.early;
 std::cerr<<"DDS owner delete complete="<<listener.deleted<<" ok="<<ok<<std::endl;
 a->delete_contained_entities();b->delete_contained_entities();f->delete_participant(a);f->delete_participant(b);
 return ok?0:6;
}
