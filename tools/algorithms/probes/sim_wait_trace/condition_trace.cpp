// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Additional focused probe of the MAVLink library's existing clockwait calls.
#include <pthread.h>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
namespace {
constexpr size_t capacity=150000;
struct Row {int64_t begin,end,deadline,tid; uintptr_t offset; int clock,result,saved_errno;};
Row rows[capacity];
std::atomic<size_t> count{0};
int64_t now() {timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000000LL+t.tv_nsec;}
__attribute__((destructor)) void save_conditions() {
  const char * base=std::getenv("EASYFLY_POLL_TRACE"); if(!base) return;
  const std::string path=std::string(base)+".conditions.csv";
  FILE * f=std::fopen(path.c_str(),"w");if(!f)return;
  size_t total=count.load(),size=total<capacity?total:capacity;
  std::fprintf(f,"# CLOCK_MONOTONIC capacity=%zu discarded=%zu\n",capacity,total-size);
  std::fprintf(f,"begin_ns,end_ns,deadline_ns,tid,caller_offset,clock_id,result,errno\n");
  for(size_t i=0;i<size;++i) {const auto &r=rows[i];
    std::fprintf(f,"%lld,%lld,%lld,%lld,%zu,%d,%d,%d\n",(long long)r.begin,(long long)r.end,
      (long long)r.deadline,(long long)r.tid,(size_t)r.offset,r.clock,r.result,r.saved_errno);
  }
  std::fclose(f);
}
}
extern "C" int pthread_cond_clockwait(pthread_cond_t * cond,pthread_mutex_t * mutex,
                                      clockid_t clock,const timespec * deadline) {
  const int incoming_errno=errno;
  static auto original=reinterpret_cast<int(*)(pthread_cond_t*,pthread_mutex_t*,clockid_t,
    const timespec*)>(dlsym(RTLD_NEXT,"pthread_cond_clockwait"));
  void * caller=__builtin_return_address(0); Dl_info info{};
  const bool target=dladdr(caller,&info) && info.dli_fname &&
    std::strstr(info.dli_fname,"libgazebo_mavlink_interface.so");
  if(!target) {errno=incoming_errno;return original(cond,mutex,clock,deadline);}
  const auto begin=now(); const auto deadline_ns=deadline->tv_sec*1000000000LL+deadline->tv_nsec;
  errno=incoming_errno;const int result=original(cond,mutex,clock,deadline);
  const int saved_errno=errno;const auto end=now();const size_t index=count.fetch_add(1);
  if(index<capacity) rows[index]={begin,end,deadline_ns,syscall(SYS_gettid),
    reinterpret_cast<uintptr_t>(caller)-reinterpret_cast<uintptr_t>(info.dli_fbase),
    clock,result,saved_errno};
  errno=saved_errno;return result;
}
