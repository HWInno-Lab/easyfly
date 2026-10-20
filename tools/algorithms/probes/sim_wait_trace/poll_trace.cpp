// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Wrap libc poll exactly once, retaining only calls from MAVLink libraries.
#include <poll.h>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr size_t capacity = 150000;
struct Row {long long begin, end, tid; int timeout, result, saved_errno;};
Row rows[capacity];
std::atomic<size_t> count{0};
long long now() {timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec*1000000000LL+t.tv_nsec;}
__attribute__((destructor)) void save() {
  const char * path=std::getenv("EASYFLY_POLL_TRACE");
  if (!path) return;
  FILE * f=std::fopen(path,"w"); if (!f) return;
  size_t total=count.load(), size=total<capacity?total:capacity;
  std::fprintf(f,"# CLOCK_MONOTONIC capacity=%zu discarded=%zu\n",capacity,total-size);
  std::fprintf(f,"begin_ns,end_ns,tid,timeout_ms,result,errno\n");
  for (size_t i=0;i<size;++i) {const auto & r=rows[i];
    std::fprintf(f,"%lld,%lld,%lld,%d,%d,%d\n",r.begin,r.end,r.tid,r.timeout,r.result,r.saved_errno);
  }
  std::fclose(f);
}
}
extern "C" int poll(pollfd * fds, nfds_t nfds, int timeout) {
  const int incoming_errno=errno;
  static auto original=reinterpret_cast<int(*)(pollfd*,nfds_t,int)>(dlsym(RTLD_NEXT,"poll"));
  Dl_info info{};
  const bool target=dladdr(__builtin_return_address(0),&info) && info.dli_fname &&
    (std::strstr(info.dli_fname,"libmavlink_interface.so") ||
     std::strstr(info.dli_fname,"libgazebo_mavlink_interface.so"));
  if (!target) {errno=incoming_errno; return original(fds,nfds,timeout);}
  const auto begin=now(); errno=incoming_errno;
  const int result=original(fds,nfds,timeout); const int saved_errno=errno;
  const auto end=now(); const size_t index=count.fetch_add(1);
  if (index<capacity) rows[index]={begin,end,syscall(SYS_gettid),timeout,result,saved_errno};
  errno=saved_errno; return result;
}
