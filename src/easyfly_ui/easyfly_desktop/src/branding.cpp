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
#include <QImage>
#include <QPixmap>
#include <QResource>
#include <algorithm>
#include "widgets.hpp"
// A direct reference retains the qrc object when desktop_widgets is a static library.
static void initialize_easyfly_resources()
{
  Q_INIT_RESOURCE(easyfly_assets);
}
namespace easyfly_desktop
{
QPixmap brand_pixmap()
{
  initialize_easyfly_resources();
  const QImage original(":/easyfly/logo.png");
  const auto image = original.convertToFormat(QImage::Format_ARGB32);
  int left = image.width(), top = image.height(), right = -1, bottom = -1;
  for (int y = 0; y < image.height(); ++y) {
    const auto * pixels = reinterpret_cast<const QRgb *>(image.constScanLine(y));
    for (int x = 0; x < image.width(); ++x) {
      if (qAlpha(pixels[x]) > 0) {
        left = std::min(left, x); top = std::min(top, y);
        right = std::max(right, x); bottom = std::max(bottom, y);
      }
    }
  }
  // Trim transparent padding only at display time; the supplied asset is unchanged.
  return right >= left ? QPixmap::fromImage(
    original.copy(
      QRect(left, top, right - left + 1, bottom - top + 1))) : QPixmap{};
}
}  // namespace easyfly_desktop
