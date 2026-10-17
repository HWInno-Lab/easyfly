// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QFile>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include "native_window.hpp"

namespace easyfly_connection_manager
{
namespace
{
int ignore_closed_window(Display *, XErrorEvent *) {return 0;}
bool owned(quint64 pid, qint64 group, const QString & directory)
{
  QFile file(QString("/proc/%1/stat").arg(pid));
  if (!file.open(QIODevice::ReadOnly)) {return false;}
  const auto data = file.readAll();
  const auto fields = data.mid(data.lastIndexOf(')') + 2).simplified().split(' ');
  if (fields.size() < 4 || fields[2].toLongLong() != group || fields[3].toLongLong() != group) {
    return false;
  }
  QFile environment(QString("/proc/%1/environ").arg(pid));
  return environment.open(QIODevice::ReadOnly) && environment.readAll().split('\0').contains(
    "XDG_CONFIG_HOME=" + directory.toUtf8());
}
}  // namespace
int close_owned_windows(qint64 group, const QString & config_directory)
{
  auto * display = XOpenDisplay(nullptr);
  if (!display) {return 0;}
  const auto previous_handler = XSetErrorHandler(ignore_closed_window);
  Atom type; int format; XID size, remaining; unsigned char * bytes = nullptr;
  const auto root = DefaultRootWindow(display);
  const auto clients = XInternAtom(display, "_NET_CLIENT_LIST", False);
  const auto pid_property = XInternAtom(display, "_NET_WM_PID", False);
  const auto protocols = XInternAtom(display, "WM_PROTOCOLS", False);
  const auto close = XInternAtom(display, "WM_DELETE_WINDOW", False);
  int count = 0;
  if (XGetWindowProperty(
      display, root, clients, 0, 4096, False, XA_WINDOW,
      &type, &format, &size, &remaining, &bytes) == Success && type == XA_WINDOW && format == 32)
  {
    const auto * windows = reinterpret_cast<Window *>(bytes);
    for (XID index = 0; index < size; ++index) {
      Window transient;
      if (XGetTransientForHint(display, windows[index], &transient)) {continue;}
      unsigned char * value = nullptr; XID length, extra; Atom actual; int bits;
      if (XGetWindowProperty(
          display, windows[index], pid_property, 0, 1, False, XA_CARDINAL,
          &actual, &bits, &length, &extra, &value) == Success &&
        actual == XA_CARDINAL && bits == 32 && length == 1)
      {
        const auto pid = *reinterpret_cast<XID *>(value);
        if (owned(pid, group, config_directory)) {
          XEvent focus{};
          focus.xclient.type = ClientMessage; focus.xclient.window = windows[index];
          focus.xclient.message_type = XInternAtom(display, "_NET_ACTIVE_WINDOW", False);
          focus.xclient.format = 32; focus.xclient.data.l[0] = 2;
          focus.xclient.data.l[1] = CurrentTime;
          XSendEvent(
            display, root, False, SubstructureRedirectMask | SubstructureNotifyMask,
            &focus);
          XEvent event{};
          event.xclient.type = ClientMessage; event.xclient.window = windows[index];
          event.xclient.message_type = protocols; event.xclient.format = 32;
          event.xclient.data.l[0] = close;
          event.xclient.data.l[1] = CurrentTime;
          if (XSendEvent(display, windows[index], False, NoEventMask, &event)) {++count;}
        }
      }
      if (value) {XFree(value);}
    }
  }
  if (bytes) {XFree(bytes);}
  XSync(display, False); XSetErrorHandler(previous_handler); XCloseDisplay(display);
  return count;
}
}  // namespace easyfly_connection_manager
