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
#include "theme.hpp"
namespace easyfly_desktop
{
namespace theme
{
QPalette palette()
{
  QPalette result;
  result.setColor(QPalette::Window, QColor("#f3f6fb"));
  result.setColor(QPalette::WindowText, QColor("#1f2937"));
  result.setColor(QPalette::Base, QColor("#ffffff"));
  result.setColor(QPalette::AlternateBase, QColor("#f7f9fd"));
  result.setColor(QPalette::Text, QColor("#1f2937"));
  result.setColor(QPalette::Button, QColor("#ffffff"));
  result.setColor(QPalette::ButtonText, QColor("#24589f"));
  result.setColor(QPalette::Highlight, QColor("#1769df"));
  result.setColor(QPalette::HighlightedText, QColor("#ffffff"));
  result.setColor(QPalette::Link, QColor("#1769df"));
  result.setColor(QPalette::ToolTipBase, QColor("#ffffff"));
  result.setColor(QPalette::ToolTipText, QColor("#1f2937"));
  result.setColor(QPalette::Disabled, QPalette::Text, QColor("#64748b"));
  result.setColor(QPalette::Disabled, QPalette::WindowText, QColor("#64748b"));
  result.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#64748b"));
  return result;
}
QString stylesheet()
{
  return QStringLiteral(
    "QWidget { color:#1f2937; font-size:13px; }"
    "QMainWindow, QWidget#workspaceRoot, QStackedWidget, QScrollArea { background:#f3f6fb; }"
    "QWidget#pageContent, QWidget#recordingPage { background:#f3f6fb; }"
    "QWidget:disabled { color:#64748b; }"
    "QLabel { background:transparent; }"
    "QLabel[heading=true] { font-size:19px; font-weight:600; color:#111827; }"
    "QLabel#pageSubtitle, QLabel#requestNote { color:#526176; }"
    "QWidget#topBar { background:#ffffff; border-bottom:1px solid #e5eaf0; }"
    "QWidget#topBar QLabel { color:#344256; }"
    "QLabel#brand { padding:0; }"
    "QFrame[dashboardCard=true] { background:#ffffff; border:1px solid #dce3eb;"
    " border-radius:10px; border-top:3px solid #d8e7fc; }"
    "QLabel[metricValue=true] { color:#111827; font-size:21px; font-weight:600; }"
    "QLabel[metricTitle=true] { color:#526176; font-size:12px; }"
    "QTabWidget::pane { background:#ffffff; border:1px solid #dce3eb; border-radius:5px; }"
    "QTabBar::tab { background:#ffffff; color:#526176; padding:7px 12px; }"
    "QTabBar::tab:selected { color:#1769df; border-bottom:2px solid #1769df; }"
    "QGroupBox { background:#ffffff; border:1px solid #dce3eb; border-radius:7px;"
    " margin-top:12px; padding:12px 10px 8px; }"
    "QGroupBox::title { subcontrol-origin:margin; left:14px; color:#344256;"
    " background:#ffffff; padding:0 5px; }"
    "QPushButton { background:#ffffff; color:#24589f; border:1px solid #cbd5e1;"
    " border-radius:5px; padding:6px 11px; }"
    "QPushButton:hover { background:#eff6ff; border-color:#8cb7f1; }"
    "QPushButton:pressed { background:#dbeafe; }"
    "QPushButton:focus { border:2px solid #1769df; }"
    "QPushButton[primary=true], QPushButton#startConnection { background:#1d4ed8;"
    " color:#ffffff; border-color:#1769df; }"
    "QPushButton[primary=true]:hover, QPushButton#startConnection:hover { background:#1e40af; }"
    "QPushButton#observerToggle { background:#ffffff; color:#24589f; border-color:#cbd5e1; }"
    "QPushButton#observerToggle:hover { border-color:#1769df; }"
    "QPushButton[danger=true] { color:#a52b2b; border-color:#e5bcbc; background:#fff8f8; }"
    "QToolButton { color:#526176; border:0; padding:5px; background:transparent; }"
    "QToolButton:checked { color:#164b9b; background:#dbeafe; border-radius:4px; }"
    "QToolButton:hover { color:#1d4ed8; background:#eaf0f8; }"
    "QPushButton:disabled, QPushButton#queryMission:disabled,"
    " QPushButton#observerToggle:disabled { background:#ffffff;"
    " color:#64748b; border:1px solid #dce3eb; }"
    "QLineEdit, QComboBox, QPlainTextEdit { background:#ffffff; color:#1f2937;"
    " border:1px solid #cbd5e1; border-radius:4px; padding:5px;"
    " selection-background-color:#dbeafe; selection-color:#174a91; }"
    "QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus { border-color:#1769df; }"
    "QLineEdit:disabled, QComboBox:disabled { background:#ffffff; color:#64748b; }"
    "QComboBox QAbstractItemView { background:#ffffff; color:#1f2937;"
    " selection-background-color:#1769df; selection-color:#ffffff; }"
    "QListWidget, QTableView { background:#ffffff; color:#1f2937;"
    " alternate-background-color:#f7f9fd; border:1px solid #dce3eb;"
    " gridline-color:#e5eaf0; selection-background-color:#dbeafe;"
    " selection-color:#174a91; }"
    "QListWidget::item { padding:6px 8px; }"
    "QListWidget::item:selected { background:#dbeafe; color:#174a91; }"
    "QListWidget#navigation { background:#eaf0f8; color:#526176; border:none; border-radius:10px; }"
    "QListWidget#navigation::item { padding:11px 8px; margin:3px"
    " 5px; border-radius:7px; border-left:3px solid transparent; }"
    "QListWidget#navigation::item:hover { background:#dfeafa; color:#1769df; }"
    "QListWidget#navigation::item:selected { background:#d9e8ff; color:#164b9b;"
    " border-left-color:#1769df; font-weight:600; }"
    "QHeaderView::section { background:#ffffff; color:#526176; padding:7px;"
    " border:0; border-bottom:1px solid #dce3eb; }"
    "QTableView::item:selected { background:#dbeafe; color:#174a91; }"
    "QSplitter::handle { background:#ffffff; border:1px solid #e5eaf0; }"
    "QMenuBar { background:#202d3d; color:#e6edf5; }"
    "QMenuBar::item { background:transparent; color:#e6edf5; padding:4px 8px; }"
    "QMenuBar::item:selected { background:#30465d; }"
    "QMenu { background:#ffffff; color:#1f2937; border:1px solid #dce3eb; }"
    "QMenu::item:selected { background:#1769df; color:#ffffff; }"
    "QStatusBar { background:#edf2f9; color:#526176; padding:3px; }"
    "QDockWidget::title { background:#ffffff; color:#344256; padding:7px;"
    " border-bottom:1px solid #dce3eb; }"
    "QLabel#warningBanner { background:#ffffff; color:#8a4b08;"
    " border:1px solid #edc36a; border-radius:4px; padding:8px; }"
    "QScrollBar:vertical { background:#ffffff; width:12px; margin:0; }"
    "QScrollBar:horizontal { background:#ffffff; height:12px; margin:0; }"
    "QScrollBar::handle { background:#b5c1d0; border-radius:4px; }"
    "QScrollBar::handle:vertical { min-height:24px; }"
    "QScrollBar::handle:horizontal { min-width:24px; }"
    "QScrollBar::handle:hover { background:#8499b1; }"
    "QScrollBar::add-line, QScrollBar::sub-line { width:0; height:0; }"
    "QScrollBar::add-page, QScrollBar::sub-page { background:transparent; }");
}
}  // namespace theme
}  // namespace easyfly_desktop
