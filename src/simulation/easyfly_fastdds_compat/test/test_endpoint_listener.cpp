// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <fastdds/rtps/RTPSDomain.h>
#include <fastdds/rtps/participant/RTPSParticipant.h>
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastdds/rtps/reader/RTPSReader.h>
#include <fastdds/rtps/reader/ReaderListener.h>
#include <fastdds/rtps/writer/RTPSWriter.h>
#include <fastdds/rtps/history/ReaderHistory.h>
#include <fastdds/rtps/history/WriterHistory.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/qos/WriterQos.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace std::chrono_literals;
struct Listener : ReaderListener {
 std::atomic<bool> matched{false},removing{false},setter_started{false},cleared{false};
 std::atomic<int> removed_status{0};
 std::atomic<bool> cleared_during_callback{false};
 void onReaderMatched(RTPSReader*,MatchingInfo& info) override {
  if(info.status==MATCHED_MATCHING)matched=true;
  if(info.status==REMOVED_MATCHING) {
   removing=true;
   auto end=std::chrono::steady_clock::now()+2s;
   while(!setter_started && std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
   std::this_thread::sleep_for(30ms);
   cleared_during_callback=cleared.load();
   std::cerr<<"inside removal callback, listener cleared="<<cleared<<std::endl;
  }
 }
 void onReaderMatched(RTPSReader*,const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override {
  if(info.current_count_change<0)++removed_status;
 }
};
int main() {
 RTPSParticipantAttributes pa;pa.setName("easyfly_native_listener_repro");
 auto* a=RTPSDomain::createParticipant(155,pa);
 auto* b=RTPSDomain::createParticipant(155,pa);
 if(!a||!b)return 2;
 HistoryAttributes ha;ha.payloadMaxSize=128;
 ReaderHistory rh(ha);WriterHistory wh(ha);Listener listener;
 ReaderAttributes ra;WriterAttributes wa;
 ra.endpoint.reliabilityKind=RELIABLE;wa.endpoint.reliabilityKind=RELIABLE;
 auto* r=RTPSDomain::createRTPSReader(a,ra,&rh,&listener);
 auto* w=RTPSDomain::createRTPSWriter(b,wa,&wh);
 if(!r||!w)return 3;
 TopicAttributes topic;topic.topicName="easyfly_native_listener_race";topic.topicDataType="native_bytes";
 ReaderQos rq;WriterQos wq;rq.m_reliability.kind=RELIABLE_RELIABILITY_QOS;wq.m_reliability.kind=RELIABLE_RELIABILITY_QOS;
 if(!a->registerReader(r,topic,rq)||!b->registerWriter(w,topic,wq))return 4;
 auto end=std::chrono::steady_clock::now()+5s;
 while(!listener.matched && std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
 if(!listener.matched){std::cerr<<"not matched"<<std::endl;return 5;}
 std::thread setter([&]{
  auto deadline=std::chrono::steady_clock::now()+5s;
  while(!listener.removing && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(1ms);
  if(!listener.removing)return;
  listener.setter_started=true;
  r->setListener(nullptr);
  listener.cleared=true;
 });
 RTPSDomain::removeRTPSWriter(w);
 setter.join();
 std::cerr<<"removed_status="<<listener.removed_status<<" cleared="<<listener.cleared<<std::endl;
 // Clearing must make progress inside the callback: holding the reader lock
 // across callbacks/PDP status updates can deadlock real RMW shutdown.
 bool ok=listener.removed_status==1 && listener.cleared && listener.cleared_during_callback;
 RTPSDomain::removeRTPSReader(r);RTPSDomain::removeRTPSParticipant(a);RTPSDomain::removeRTPSParticipant(b);
 return ok?0:6;
}
