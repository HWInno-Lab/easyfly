// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Inspect real transport delivery in the native asynchronous flow controller. A send
// must retain its selector lock from delivery until flush, otherwise teardown
// can acquire selector -> flow while this thread holds flow -> selector.
#include <fastdds/rtps/RTPSDomain.h>
#include <fastdds/rtps/participant/RTPSParticipant.h>
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastdds/rtps/reader/RTPSReader.h>
#include <fastdds/rtps/reader/ReaderListener.h>
#include <fastdds/rtps/writer/RTPSWriter.h>
#include <fastdds/rtps/writer/LocatorSelectorSender.hpp>
#include <fastdds/rtps/history/ReaderHistory.h>
#include <fastdds/rtps/history/WriterHistory.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/attributes/LibrarySettingsAttributes.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/qos/WriterQos.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <dlfcn.h>
#include <execinfo.h>
#include <iostream>
#include <pthread.h>
#include <thread>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace std::chrono_literals;

extern "C" int native_mutex_unlock(pthread_mutex_t*);
extern "C" int native_mutex_lock(pthread_mutex_t*);
asm(".symver native_mutex_unlock,__pthread_mutex_unlock@GLIBC_2.2.5");
asm(".symver native_mutex_lock,__pthread_mutex_lock@GLIBC_2.2.5");
namespace {
std::atomic<pthread_mutex_t*> watched{nullptr};
thread_local bool capture = false, inspecting = false;
thread_local int depth = 0;
std::atomic<int> sends{0}, unlocked_sends{0};
}

extern "C" int pthread_mutex_lock(pthread_mutex_t* mutex) noexcept
{
    if (capture) watched = mutex;
    if (mutex == watched.load() && !inspecting)
    {
        inspecting = true;
        void* frames[12];
        int count = backtrace(frames, 12);
        for (int i = 1; i < count; ++i)
        {
            Dl_info info{};
            if (dladdr(frames[i], &info) && info.dli_sname &&
                    std::strstr(info.dli_sname, "RTPSMessageGroup4sendEv"))
            {
                ++sends;
                if (depth == 0) ++unlocked_sends;
                break;
            }
        }
        inspecting = false;
    }
    const int result = native_mutex_lock(mutex);
    if (result == 0 && mutex == watched.load()) ++depth;
    return result;
}

extern "C" int pthread_mutex_unlock(pthread_mutex_t* mutex) noexcept
{
    if (mutex == watched.load()) --depth;
    return native_mutex_unlock(mutex);
}

struct Listener : ReaderListener
{
    std::atomic<bool> matched{false};
    std::atomic<int> received{0};
    void onReaderMatched(RTPSReader*, MatchingInfo& info) override
    {
        if (info.status == MATCHED_MATCHING) matched = true;
    }
    void onNewCacheChangeAdded(RTPSReader* reader, const CacheChange_t* const change) override
    {
        if (change->serializedPayload.length == 4 && change->serializedPayload.data[0] == 42)
            ++received;
        reader->getHistory()->remove_change(const_cast<CacheChange_t*>(change));
    }
};

int main()
{
    LibrarySettingsAttributes settings;
    settings.intraprocess_delivery = INTRAPROCESS_OFF;
    xmlparser::XMLProfileManager::library_settings(settings);
    RTPSParticipantAttributes pa;
    pa.setName("easyfly_flow_flush");
    auto* a = RTPSDomain::createParticipant(211, pa);
    auto* b = RTPSDomain::createParticipant(211, pa);
    if (!a || !b) return 1;
    HistoryAttributes ha;
    ha.payloadMaxSize = 16;
    ReaderHistory rh(ha);
    WriterHistory wh(ha);
    Listener listener;
    ReaderAttributes ra;
    WriterAttributes wa;
    ra.endpoint.reliabilityKind = wa.endpoint.reliabilityKind = RELIABLE;
    wa.mode = ASYNCHRONOUS_WRITER;
    auto* reader = RTPSDomain::createRTPSReader(a, ra, &rh, &listener);
    auto* writer = RTPSDomain::createRTPSWriter(b, wa, &wh);
    if (!reader || !writer) return 2;
    capture = true;
    writer->get_async_locator_selector().lock();
    capture = false;
    writer->get_async_locator_selector().unlock();
    TopicAttributes topic;
    topic.topicName = "easyfly_flow_flush";
    topic.topicDataType = "native_bytes";
    ReaderQos rq;
    WriterQos wq;
    rq.m_reliability.kind = wq.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    if (!a->registerReader(reader, topic, rq) || !b->registerWriter(writer, topic, wq)) return 3;
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!listener.matched && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(1ms);
    if (!listener.matched) return 4;
    for (int i = 0; i < 4; ++i)
    {
        auto* change = writer->new_change([] {return 4u;}, ALIVE);
        if (!change) return 5;
        change->serializedPayload.length = 4;
        std::memset(change->serializedPayload.data, 42, 4);
        if (!wh.add_change(change)) return 6;
        deadline = std::chrono::steady_clock::now() + 3s;
        while (listener.received <= i && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
    }
    RTPSDomain::removeRTPSParticipant(b);
    RTPSDomain::removeRTPSParticipant(a);
    std::cout << "received=" << listener.received << " sends=" << sends
              << " unlocked_sends=" << unlocked_sends << std::endl;
    return listener.received == 4 && sends > 0 && unlocked_sends == 0 ? 0 : 7;
}
