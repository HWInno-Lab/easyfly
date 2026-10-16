// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Linux/glibc scheduling fixture: real endpoints, real mutex and real setter.
// The test-only unlock barrier lets setListener(nullptr) run in the actual
// discovery callback's unlocked window; no production instruction is replaced.
#include <fastdds/rtps/RTPSDomain.h>
#include <fastdds/rtps/participant/RTPSParticipant.h>
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastdds/rtps/reader/RTPSReader.h>
#include <fastdds/rtps/reader/ReaderListener.h>
#include <fastdds/rtps/writer/RTPSWriter.h>
#include <fastdds/rtps/history/ReaderHistory.h>
#include <fastdds/rtps/history/WriterHistory.h>
#include <fastdds/rtps/builtin/data/WriterProxyData.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/qos/WriterQos.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <memory>
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
pthread_mutex_t* watched_mutex = nullptr;
thread_local bool armed = false;
thread_local int lock_depth = 0;
const char* watched_function = nullptr;
std::atomic<bool> unlocked{false}, cleared{false}, timed_out{false};
}

extern "C" int pthread_mutex_lock(pthread_mutex_t* mutex) noexcept
{
    const int result = native_mutex_lock(mutex);
    if (result == 0 && armed && mutex == watched_mutex) ++lock_depth;
    return result;
}

extern "C" int pthread_mutex_unlock(pthread_mutex_t* mutex) noexcept
{
    const bool outermost = armed && mutex == watched_mutex && --lock_depth == 0;
    const int result = native_mutex_unlock(mutex);
    if (outermost)
    {
        // Avoid intercepting nested history/WriterProxy unlocks. Resolve the
        // actual caller from the loaded library rather than a hardcoded PC.
        Dl_info caller{};
        if (dladdr(__builtin_return_address(0), &caller) && caller.dli_sname &&
                std::strstr(caller.dli_sname, watched_function))
        {
            armed = false;
            unlocked = true;
            const auto end = std::chrono::steady_clock::now() + 2s;
            while (!cleared && std::chrono::steady_clock::now() < end)
            {
                std::this_thread::yield();
            }
            timed_out = !cleared;
        }
    }
    return result;
}

struct Listener : ReaderListener
{
    std::atomic<bool> matched{false}, cleared_before_callback{false};
    std::atomic<int> callbacks{0};
    WriterDiscoveryInfo::DISCOVERY_STATUS expected = WriterDiscoveryInfo::REMOVED_WRITER;
    std::unique_ptr<WriterProxyData> proxy;
    void onReaderMatched(RTPSReader*, MatchingInfo& info) override
    {
        if (info.status == MATCHED_MATCHING) matched = true;
    }
    void on_writer_discovery(RTPSReader*, WriterDiscoveryInfo::DISCOVERY_STATUS status,
            const GUID_t&, const WriterProxyData* data) override
    {
        if (status == WriterDiscoveryInfo::DISCOVERED_WRITER && !proxy && data)
        {
            proxy.reset(new WriterProxyData(*data));
        }
        if (status == expected)
        {
            ++callbacks;
            cleared_before_callback = cleared.load();
        }
    }
};

int main(int argc, char** argv)
{
    const bool reliable = argc == 3 && std::strcmp(argv[1], "reliable") == 0;
    if (argc != 3 || (!reliable && std::strcmp(argv[1], "best_effort"))) return 1;
    const bool remove = std::strcmp(argv[2], "remove") == 0;
    const bool add = std::strcmp(argv[2], "add") == 0;
    const bool qos = std::strcmp(argv[2], "qos") == 0;
    if (!remove && !add && !qos) return 1;
    watched_function = remove ? "matched_writer_remove" : "matched_writer_add";
    RTPSParticipantAttributes pa;
    pa.setName("easyfly_reader_discovery_race");
    auto* a = RTPSDomain::createParticipant(reliable ? 209 : 210, pa);
    auto* b = RTPSDomain::createParticipant(reliable ? 209 : 210, pa);
    if (!a || !b) return 2;
    HistoryAttributes ha;
    ha.payloadMaxSize = 128;
    ReaderHistory rh(ha);
    WriterHistory wh(ha);
    Listener listener;  // Kept alive through both endpoint removals.
    listener.expected = remove ? WriterDiscoveryInfo::REMOVED_WRITER :
            (add ? WriterDiscoveryInfo::DISCOVERED_WRITER : WriterDiscoveryInfo::CHANGED_QOS_WRITER);
    ReaderAttributes ra;
    WriterAttributes wa;
    ra.endpoint.reliabilityKind = wa.endpoint.reliabilityKind = reliable ? RELIABLE : BEST_EFFORT;
    auto* reader = RTPSDomain::createRTPSReader(a, ra, &rh, &listener);
    auto* writer = RTPSDomain::createRTPSWriter(b, wa, &wh);
    if (!reader || !writer) return 3;
    TopicAttributes topic;
    topic.topicName = "easyfly_reader_discovery_race";
    topic.topicDataType = "native_bytes";
    ReaderQos rq;
    WriterQos wq;
    rq.m_reliability.kind = wq.m_reliability.kind = reliable ?
            RELIABLE_RELIABILITY_QOS : BEST_EFFORT_RELIABILITY_QOS;
    if (!a->registerReader(reader, topic, rq) || !b->registerWriter(writer, topic, wq)) return 4;
    const auto end = std::chrono::steady_clock::now() + 5s;
    while (!listener.matched && std::chrono::steady_clock::now() < end)
    {
        std::this_thread::sleep_for(1ms);
    }
    if (!listener.matched || !listener.proxy) return 5;
    if (add && !reader->matched_writer_remove(writer->getGuid(), false)) return 5;
    listener.callbacks = 0;
    listener.cleared_before_callback = false;
    watched_mutex = reader->getMutex().native_handle();
    std::thread setter([&] {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!unlocked && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::yield();
        }
        if (unlocked)
        {
            reader->setListener(nullptr);
            cleared = true;
        }
    });
    armed = true;
    const bool changed = remove ? reader->matched_writer_remove(writer->getGuid(), false) :
            reader->matched_writer_add(*listener.proxy);
    armed = false;
    setter.join();
    const bool ok = changed == !qos && unlocked && cleared && !timed_out &&
            listener.callbacks == 1 && listener.cleared_before_callback;
    std::cout << argv[1] << ' ' << argv[2] << " changed=" << changed << " unlocked=" << unlocked
              << " cleared=" << cleared << " timeout=" << timed_out
              << " callbacks=" << listener.callbacks
              << " cleared_before_callback=" << listener.cleared_before_callback << std::endl;
    RTPSDomain::removeRTPSReader(reader);
    RTPSDomain::removeRTPSWriter(writer);
    RTPSDomain::removeRTPSParticipant(a);
    RTPSDomain::removeRTPSParticipant(b);
    return ok ? 0 : 6;
}
