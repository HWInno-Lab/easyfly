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
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QCheckBox>
#include <QRegularExpression>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QLocale>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTabWidget>
#include <QToolButton>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include "easyfly_desktop/mission_draft.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "easyfly_desktop/mission_geometry.hpp"
#include "layout_state.hpp"
namespace easyfly_desktop
{
namespace mission = easyfly_mission_fsm;
struct MissionEditor::Impl
{
  MissionEditor * owner;
  DraftWorker worker;
  std::function<void()> changed;
  QPlainTextEdit * body;
  QLabel * status;
  QListWidget * list;
  QComboBox * type;
  QComboBox * add_type;
  QWidget * fields;
  QPlainTextEdit * forbidden{nullptr};
  QCheckBox * return_home{nullptr};
  QFormLayout * form;
  QLineEdit * directory;
  QLineEdit * name;
  MissionGeometryCanvas * geometry;
  QLabel * geometry_summary;
  uint64_t geometry_revision{0};
  std::map<std::string, QLineEdit *> inputs;
  std::vector<QPushButton *> edit_buttons;
  QPushButton * save;
  QTimer * debounce;
  QMessageBox * prompt{nullptr};
  std::vector<mission::MissionStep> steps;
  std::string content, clean_content;
  uint64_t revision{0}, valid_revision{0}, request{0}, consumed{0};
  bool valid{false}, form_dirty{false}, io{false}, closing{false}, loading{false};
  QString root;
  QString exported_root;
  std::optional<ExportedDraft> exported;
  Impl(MissionEditor * widget, std::vector<std::string> paths, std::function<void()> callback)
  : owner(widget), worker(std::move(paths)), changed(std::move(callback))
  {
    owner->setObjectName("pageContent"); owner->setAttribute(Qt::WA_StyledBackground, true);
    auto * layout = new QVBoxLayout(owner);
    auto * heading = new QLabel(QStringLiteral("任务编辑 · 本地草稿"), owner);
    heading->setProperty("heading", true); layout->addWidget(heading);
    auto * note = new QLabel(
      QStringLiteral(
        "仅编辑、校验和导出，不执行任务。坐标 odom · m / rad / s · 顺序执行，失败终止。\n"
        "导出后不会自动进入服务目录；仿真参数示例不代表实机安全配置。"), owner);
    note->setWordWrap(true); note->hide();
    auto * help = new QToolButton(owner); help->setText(QStringLiteral("任务编辑说明"));
    help->setCheckable(true);
    QObject::connect(help, &QToolButton::toggled, note, &QWidget::setVisible);
    auto * toolbar = new QHBoxLayout;
    button(
      toolbar, "draftNew", QStringLiteral("新建示例"), [this] {
        replace_question(
          [this] {
            DraftJob j; j.operation = DraftOperation::Normalize; j.revision = revision;
            for (const auto * t : {"takeoff", "hover", "land"}) {
              j.steps.push_back(draft_default_step(t));
            }
            submit(std::move(j), true);
          });
      });
    button(
      toolbar, "draftOpen", QStringLiteral("打开本地文件"), [this] {
        auto * dialog = new QFileDialog(owner, QStringLiteral("读取本地任务（不改原文件）"));
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setOption(QFileDialog::DontUseNativeDialog);
        dialog->setFileMode(QFileDialog::ExistingFile);
        dialog->setNameFilter(QStringLiteral("任务 (*.yaml *.yml *.mission);;所有文件 (*)"));
        QObject::connect(
          dialog, &QFileDialog::fileSelected, owner, [this](const QString & p) {
            open(p);
          });
        dialog->open();
      });
    button(toolbar, "draftValidate", QStringLiteral("校验任务"), [this] {validate();});
    toolbar->addStretch(); toolbar->addWidget(help); layout->addLayout(toolbar);
    layout->addWidget(note);
    auto * tabs = new QTabWidget(owner); tabs->setObjectName("draftTabs");
    layout->addWidget(tabs, 1);
    auto * split = new QSplitter(Qt::Horizontal, tabs);
    split->setObjectName("draftStepParameterSplitter"); split->setChildrenCollapsible(false);
    tabs->addTab(split, QStringLiteral("步骤与参数"));
    auto * raw = new QGroupBox(QStringLiteral("YAML 正文 · 保留原文直至编辑"), tabs);
    auto * raw_layout = new QVBoxLayout(raw); body = new QPlainTextEdit(raw);
    body->setObjectName("draftBody");
    body->setPlaceholderText(QStringLiteral("新建示例或打开本地文件；也可直接粘贴 v1 YAML。"));
    body->setLineWrapMode(QPlainTextEdit::NoWrap); raw_layout->addWidget(body);
    auto * sequence_page = new QWidget(split); sequence_page->setMinimumWidth(230);
    auto * sequence_layout = new QVBoxLayout(sequence_page);
    sequence_layout->addWidget(new QLabel(QStringLiteral("任务步骤"), sequence_page));
    auto * parameters_page = new QWidget(split); parameters_page->setMinimumWidth(320);
    auto * parameters_layout = new QVBoxLayout(parameters_page);
    parameters_layout->addWidget(new QLabel(QStringLiteral("当前步骤参数"), parameters_page));
    auto * geometry_page = new QWidget(tabs);
    auto * geometry_layout = new QVBoxLayout(geometry_page);
    tabs->addTab(geometry_page, QStringLiteral("二维几何示意"));
    auto * geometry_note = new QLabel(
      QStringLiteral(
        "蓝点：明确目标；青框：搜索边界；橙色：选中。\n"
        "虚线仅目标顺序；滚轮缩放/中键平移/单击选择步骤。\n"
        "非规划轨迹/覆盖证明，无起点补造；搜索/动态步骤断开连线。"), geometry_page);
    geometry_note->setWordWrap(true); geometry_layout->addWidget(geometry_note);
    geometry = new MissionGeometryCanvas(geometry_page); geometry_layout->addWidget(geometry, 1);
    auto * view_actions = new QHBoxLayout;
    button(view_actions, "draftGeometryFit", QStringLiteral("适配全部"), [this] {geometry->fit();});
    button(
      view_actions, "draftGeometryZoomIn", QStringLiteral("放大"),
      [this] {geometry->zoom(1.25);});
    button(
      view_actions, "draftGeometryZoomOut", QStringLiteral("缩小"),
      [this] {geometry->zoom(.8);});
    geometry_layout->addLayout(view_actions);
    geometry_summary = new QLabel(geometry_page); geometry_summary->setObjectName(
      "draftGeometrySummary");
    geometry_summary->setTextFormat(Qt::PlainText); geometry_summary->setWordWrap(true);
    geometry_summary->setMinimumWidth(0);
    geometry_summary->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    geometry_layout->addWidget(geometry_summary);
    geometry->selection_handler(
      [this](size_t row) {
        if (valid && valid_revision == revision && !form_dirty && !busy() &&
        row < static_cast<size_t>(list->count())) {list->setCurrentRow(static_cast<int>(row));}
      });
    list = new QListWidget(sequence_page); list->setObjectName("draftSteps");
    sequence_layout->addWidget(list, 1);
    type = new QComboBox(parameters_page); type->setObjectName("draftType");
    type->addItems(
      {"takeoff", "hover", "waypoint", "navigate", "track", "search", "land",
        "precision_land", "explore"});
    parameters_layout->addWidget(type);
    add_type = new QComboBox(sequence_page); add_type->setObjectName("draftAddType");
    for (int i = 0; i < type->count(); ++i) {add_type->addItem(type->itemText(i));}
    sequence_layout->addWidget(new QLabel(QStringLiteral("追加步骤类型"), sequence_page));
    sequence_layout->addWidget(add_type);
    auto * actions = new QHBoxLayout;
    button(
      actions, "draftAdd", QStringLiteral("追加"), [this] {
        auto candidate = steps;
        candidate.push_back(draft_default_step(add_type->currentText().toStdString()));
        normalize(std::move(candidate));
      }, true);
    button(
      actions, "draftDelete", QStringLiteral("删除"), [this] {
        auto candidate = steps; const int row = list->currentRow();
        if (row >= 0) {candidate.erase(candidate.begin() + row); normalize(std::move(candidate));}
      }, true);
    for (int direction : {-1, 1}) {
      button(
        actions, direction < 0 ? "draftUp" : "draftDown",
        direction < 0 ? QStringLiteral("上移") : QStringLiteral("下移"), [this, direction] {
          const int row = list->currentRow(), to = row + direction;
          if (row < 0 || to < 0 || to >= static_cast<int>(steps.size())) {return;}
          auto candidate = steps; std::swap(candidate[row], candidate[to]);
          normalize(std::move(candidate));
        }, true);
    }
    sequence_layout->addLayout(actions);
    auto * scroll = new QScrollArea(parameters_page); scroll->setWidgetResizable(true);
    fields = new QWidget(scroll); form = new QFormLayout(fields); scroll->setWidget(fields);
    fields->setObjectName("draftFields"); fields->setStyleSheet(
      "QWidget#draftFields {background:#ffffff;}");
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    parameters_layout->addWidget(scroll, 1);
    auto * apply = new QHBoxLayout;
    button(apply, "draftApply", QStringLiteral("应用表单"), [this] {apply_form();}, true);
    button(apply, "draftResetForm", QStringLiteral("重置表单"), [this] {show_fields();}, true);
    parameters_layout->addLayout(apply);
    split->setStretchFactor(0, 0); split->setStretchFactor(1, 1);
    split->setSizes({250, 700}); remember_splitter(split, "mission-editor");
    tabs->addTab(raw, QStringLiteral("高级 YAML"));
    auto * export_box = new QGroupBox(QStringLiteral("独立草稿目录 · 非任务服务目录"), owner);
    auto * export_layout = new QVBoxLayout(export_box);
    auto * folder = new QHBoxLayout;
    directory = new QLineEdit(export_box); directory->setObjectName("draftDirectory");
    directory->setPlaceholderText(QStringLiteral("绝对路径：新目录或之前创建的专用草稿目录"));
    folder->addWidget(directory, 1);
    button(
      folder, "draftBrowse", QStringLiteral("选择父目录"), [this] {
        auto * dialog = new QFileDialog(
          owner, QStringLiteral("选择父目录（子目录 easyfly-drafts）"));
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setOption(QFileDialog::DontUseNativeDialog);
        dialog->setFileMode(QFileDialog::Directory); dialog->setOption(QFileDialog::ShowDirsOnly);
        QObject::connect(
          dialog, &QFileDialog::fileSelected, owner, [this](const QString & p) {
            directory->setText(p + "/easyfly-drafts");
          }); dialog->open();
      });
    button(
      folder, "draftPrepare", QStringLiteral("确认专用目录"), [this] {
        prepare(directory->text());
      });
    export_layout->addLayout(folder);
    auto * export_row = new QHBoxLayout; name = new QLineEdit("mission.yaml", export_box);
    name->setObjectName("draftFilename"); export_row->addWidget(name, 1);
    save = button(
      export_row, "draftExport", QStringLiteral("导出已校验草稿"), [this] {
        export_to(name->text());
      });
    export_layout->addLayout(export_row);
    auto * export_toggle = new QToolButton(owner);
    export_toggle->setObjectName("toggleDraftExport");
    export_toggle->setText(QStringLiteral("导出与目录设置")); export_toggle->setCheckable(true);
    export_box->hide(); layout->addWidget(export_toggle); layout->addWidget(export_box);
    QObject::connect(export_toggle, &QToolButton::toggled, export_box, &QWidget::setVisible);
    status = new QLabel(QStringLiteral("未校验 · 本地草稿不发送任何 ROS 请求"), owner);
    status->setMinimumWidth(0); status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    status->setObjectName("draftStatus"); status->setTextFormat(Qt::PlainText); status->setWordWrap(
      true);
    status->setTextInteractionFlags(Qt::TextSelectableByMouse); layout->addWidget(status);
    debounce = new QTimer(owner); debounce->setSingleShot(true); debounce->setInterval(180);
    QObject::connect(debounce, &QTimer::timeout, owner, [this] {validate();});
    QObject::connect(
      body, &QPlainTextEdit::textChanged, owner, [this] {
        if (loading) {return;}
        content = body->toPlainText().toUtf8().toStdString(); ++revision; valid = false;
        steps.clear(); list->clear(); show_fields(); status->setText(
          QStringLiteral(
            "正文已更改，等待后台校验"));
        debounce->start(); notify(); enable();
      });
    QObject::connect(
      list, &QListWidget::currentRowChanged, owner, [this] {
        if (!loading) {
          show_fields();
        }
      });
    QObject::connect(
      type, &QComboBox::currentTextChanged, owner, [this] {
        if (loading || !valid || list->currentRow() < 0) {return;}
        form_dirty = true; build_fields(
          draft_default_step(
            type->currentText().toStdString())); notify(); enable();
      });
    QObject::connect(directory, &QLineEdit::textChanged, owner, [this] {root.clear(); enable();});
    auto * timer = new QTimer(owner); timer->setInterval(50);
    QObject::connect(timer, &QTimer::timeout, owner, [this] {poll();}); timer->start(); enable();
  }
  QPushButton * button(
    QHBoxLayout * layout, const char * id, const QString & title,
    std::function<void()> action, bool edit = false)
  {
    auto * b = new QPushButton(title, owner); b->setObjectName(id); layout->addWidget(b);
    QObject::connect(b, &QPushButton::clicked, owner, std::move(action)); if (edit) {
      edit_buttons.push_back(b);
    }
    return b;
  }
  void notify() {if (changed) {changed();}}
  bool dirty() const {return form_dirty || content != clean_content;}
  bool busy() const {return io || prompt;}
  void enable()
  {
    const bool ready = !closing && !busy();
    body->setReadOnly(!ready || form_dirty); directory->setEnabled(ready); name->setEnabled(ready);
    const bool checked = ready && valid && valid_revision == revision;
    list->setEnabled(checked && !form_dirty); type->setEnabled(checked);
    add_type->setEnabled(checked && !form_dirty);
    fields->setEnabled(checked && list->currentRow() >= 0);
    for (auto * b : owner->findChildren<QPushButton *>()) {
      if (b->objectName().startsWith("draft")) {b->setEnabled(ready);}
    }
    for (auto * b : edit_buttons) {
      const bool form_button = b->objectName() == "draftApply" ||
        b->objectName() == "draftResetForm";
      b->setEnabled(checked && (form_button ? list->currentRow() >= 0 : !form_dirty));
    }
    save->setEnabled(checked && !form_dirty && !root.isEmpty());
    refresh_geometry();
  }
  void refresh_geometry()
  {
    if (closing || !valid || valid_revision != revision || form_dirty) {
      const auto reason = form_dirty ? QStringLiteral("表单改动尚未应用；几何已清除") :
        QStringLiteral("当前正文未通过 v1 校验；几何已清除");
      geometry_revision = 0; geometry->invalidate(reason); geometry_summary->setText(reason);
      return;
    }
    if (geometry_revision != revision) {
      geometry->plan({steps, {}}); geometry_revision = revision;
    }
    geometry->select_step(list->currentRow()); const auto & g = geometry->geometry();
    if (!g.accepted) {geometry_summary->setText(QString::fromStdString(g.error)); return;}
    QString selected;
    const int row = list->currentRow();
    if (row >= 0 && row < static_cast<int>(steps.size())) {
      const auto & s = steps[row]; selected = QString("#%1 %2").arg(row + 1).arg(
        QString::fromStdString(
          s.type));
      if (s.type == "waypoint" || s.type == "navigate") {
        selected += QString(" · (%1, %2, %3)m").arg(s.x, 0, 'g', 8).arg(s.y, 0, 'g', 8).arg(
          s.z, 0,
          'g', 8);
      } else if (s.type == "search") {
        selected += QString(" · center=(%1,%2)m · %3×%4m · z=%5m")
          .arg(s.center_x).arg(s.center_y).arg(s.area_width).arg(s.area_height).arg(s.altitude);
      } else {selected += QStringLiteral(" · 无独立 XY 几何，不推断起点/动态位置");}
    }
    geometry_summary->setText(
      QStringLiteral("%1 个目标 · %2 个区域 · %3 个无独立 XY 步骤\n%4")
      .arg(g.points.size()).arg(g.areas.size()).arg(g.unlocated.size()).arg(selected));
  }
  void question(const QString & title, const QString & text, std::function<void()> accepted)
  {
    if (prompt || closing) {return;}
    prompt = new QMessageBox(
      QMessageBox::Warning, title, text,
      QMessageBox::Yes | QMessageBox::Cancel, owner);
    auto * p = prompt; p->setObjectName("draftPrompt"); p->setTextFormat(Qt::PlainText);
    p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
    p->setButtonText(QMessageBox::Yes, QStringLiteral("确认")); p->setButtonText(
      QMessageBox::Cancel, QStringLiteral(
        "保留原状"));
    QObject::connect(
      p, &QMessageBox::finished, owner, [this, p, accepted = std::move(accepted)](int result) {
        if (prompt != p) {return;}
        prompt = nullptr; p->setObjectName(""); p->deleteLater(); enable();
        if (result == QMessageBox::Yes && !closing) {accepted();}
      }); p->open(); enable();
  }
  void replace_question(std::function<void()> action)
  {
    if (busy() || closing) {return;}
    if (dirty()) {
      question(
        QStringLiteral("替换未导出草稿？"), QStringLiteral(
          "当前本地改动将丢弃；不会改动原文件或远端任务。"),
        std::move(action));
    } else {action();}
  }
  void submit(DraftJob j, bool explicit_io)
  {
    debounce->stop(); request = worker.submit(std::move(j));
    if (!request) {
      status->setText(
        QStringLiteral(
          "操作未入队：正文须≤64KiB，步骤≤256，或后台操作尚未完成")); io = false;
    } else {io = explicit_io; status->setText(QStringLiteral("后台处理本地草稿…"));}
    enable();
  }
  void validate()
  {
    if (busy() || closing || form_dirty) {return;}
    DraftJob j; j.revision = revision; j.content = content; submit(std::move(j), false);
  }
  void open(const QString & path)
  {
    replace_question(
      [this, path] {
        DraftJob j; j.operation = DraftOperation::Read;
        j.path = path.toUtf8().toStdString(); j.revision = revision;
        submit(std::move(j), true);
      });
  }
  void prepare(const QString & path)
  {
    if (busy() || closing) {return;}
    root.clear(); DraftJob j; j.operation = DraftOperation::PrepareDirectory;
    j.path = path.toUtf8().toStdString();
    submit(std::move(j), true);
  }
  void export_to(const QString & filename)
  {
    if (busy() || closing || !valid || form_dirty || valid_revision != revision || root.isEmpty()) {
      return;
    }
    DraftJob j; j.operation = DraftOperation::InspectExport; j.path = root.toUtf8().toStdString();
    j.filename = filename.toUtf8().toStdString(); j.content = content; j.revision = revision;
    submit(std::move(j), true);
  }
  void normalize(std::vector<mission::MissionStep> candidate)
  {
    if (busy() || closing || !valid) {return;}
    question(
      QStringLiteral("规范化 YAML？"), QStringLiteral(
        "表单操作将重写整份正文：移除注释、展开默认值、统一格式并改变内容哈希。\n"
        "原文件不会改动。候选步骤须通过同一个后台校验器才会替换正文。"),
      [this, candidate = std::move(
        candidate)] {
        DraftJob j; j.operation = DraftOperation::Normalize;
        j.revision = revision; j.steps = candidate;
        submit(std::move(j), true);
      });
  }
  void build_fields(const mission::MissionStep & step)
  {
    while (form->rowCount()) {form->removeRow(0);} inputs.clear();
    forbidden = nullptr; return_home = nullptr;
    for (const auto & f : draft_fields(step)) {
      auto * line = new QLineEdit(QString::number(f.value, 'g', 17), fields);
      line->setObjectName("draftField_" + QString::fromStdString(f.key)); inputs[f.key] = line;
      form->addRow(QString::fromStdString(f.key + " [" + f.unit + "]"), line);
      QObject::connect(
        line, &QLineEdit::textEdited, owner, [this] {
          form_dirty = true; notify(); enable();
        });
    }
    if (step.type == "explore") {
      return_home = new QCheckBox(QStringLiteral("完成后返回探索起点"), fields);
      return_home->setObjectName("draftExploreReturnHome");
      return_home->setChecked(step.return_home); form->addRow(return_home);
      QObject::connect(
        return_home, &QCheckBox::toggled, owner, [this] {
          form_dirty = true; notify(); enable();
        });
      forbidden = new QPlainTextEdit(fields); forbidden->setObjectName("draftExploreForbidden");
      forbidden->setMinimumHeight(100);
      QStringList rows;
      for (const auto & box : step.forbidden_boxes) {
        QStringList values;
        for (double v : box.min) {values << QString::number(v, 'g', 17);}
        for (double v : box.max) {values << QString::number(v, 'g', 17);}
        rows << values.join(' ');
      }
      forbidden->setPlainText(rows.join('\n'));
      forbidden->setAccessibleName(QStringLiteral("禁飞盒坐标，每行六个数，单位米"));
      auto * hint = new QLabel(
        QStringLiteral(
          "禁飞盒（可空，最多16行）：每行 xmin ymin zmin xmax ymax zmax，单位m。"
          "范围使用 odom，Z 限0.5～2.5m；飞行许可仍由实时地图与飞控检查。"), fields);
      hint->setWordWrap(true); form->addRow(hint); form->addRow(forbidden);
      QObject::connect(
        forbidden, &QPlainTextEdit::textChanged, owner, [this] {
          form_dirty = true; notify(); enable();
        });
    }
  }
  void show_fields()
  {
    form_dirty = false; const int row = list->currentRow(); loading = true;
    if (row >= 0 && row < static_cast<int>(steps.size())) {
      type->setCurrentText(QString::fromStdString(steps[row].type)); build_fields(steps[row]);
    } else {while (form->rowCount()) {form->removeRow(0);} inputs.clear();}
    loading = false; enable();
  }
  void apply_form()
  {
    const int row = list->currentRow(); if (row < 0 || row >= static_cast<int>(steps.size())) {
      return;
    }
    auto candidate = steps;
    auto s = type->currentText().toStdString() == candidate[row].type ? candidate[row] :
      draft_default_step(type->currentText().toStdString());
    for (const auto & input : inputs) {
      bool ok = false; const double value = QLocale::c().toDouble(input.second->text(), &ok);
      if (!ok || !std::isfinite(value) || !draft_set_field(s, input.first, value)) {
        status->setText(
          QStringLiteral("表单数值无效：") + QString::fromStdString(input.first)); return;
      }
    }
    if (s.type == "explore" && forbidden && return_home) {
      s.return_home = return_home->isChecked(); s.forbidden_boxes.clear();
      const auto rows = forbidden->toPlainText().split('\n', Qt::SkipEmptyParts);
      if (rows.size() > 16 || forbidden->toPlainText().size() > 8192) {
        status->setText(QStringLiteral("禁飞盒最多16行，内容不能超过8192字符")); return;
      }
      for (const auto & line : rows) {
        const auto values = line.trimmed().split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        mission::ExplorationBox box{}; bool valid_box = values.size() == 6;
        for (int i = 0; valid_box && i < 6; ++i) {
          bool ok = false; const double v = QLocale::c().toDouble(values[i], &ok);
          valid_box = ok && std::isfinite(v);
          (i < 3 ? box.min[i] : box.max[i - 3]) = v;
        }
        if (!valid_box) {
          status->setText(QStringLiteral("禁飞盒每行需要六个有限数值；输入已保留")); return;
        }
        s.forbidden_boxes.push_back(box);
      }
    }
    candidate[row] = s; normalize(std::move(candidate));
  }
  void replace_body(const std::string & bytes, bool clean)
  {
    loading = true;
    body->setPlainText(QString::fromUtf8(bytes.data(), static_cast<int>(bytes.size())));
    loading = false;
    content = bytes; if (clean) {clean_content = bytes;}
    ++revision; valid = false; form_dirty = false; notify(); validate();
  }
  void poll()
  {
    const auto r = worker.latest(); if (!r || r->id != request || r->id == consumed) {return;}
    consumed = r->id; io = false;
    if (!r->success) {
      if (r->job.operation == DraftOperation::Validate && r->job.revision != revision) {return;}
      if (r->job.operation == DraftOperation::Validate) {
        valid = false; steps.clear(); list->clear(); show_fields();
      }
      status->setText(
        QStringLiteral("未通过 / 未确认成功：") + QString::fromStdString(r->error) +
        (r->backup.empty() ? QString{} : QStringLiteral("\n原内容备份保留于：") +
        QString::fromStdString(r->backup)));
      enable(); return;
    }
    if (r->job.operation == DraftOperation::Read || r->job.operation == DraftOperation::Normalize) {
      replace_body(r->content, r->job.operation == DraftOperation::Read); return;
    }
    if (r->job.operation == DraftOperation::Validate) {
      if (r->job.revision != revision) {return;}
      valid_revision = revision; valid = r->format == "yaml_v1";
      steps = valid ? r->plan.steps : std::vector<mission::MissionStep>{};
      loading = true; list->clear();
      for (size_t i = 0; i < steps.size(); ++i) {
        list->addItem(QString::number(i + 1) + " · " + QString::fromStdString(steps[i].type));
      }
      if (!steps.empty()) {list->setCurrentRow(0);} loading = false; show_fields();
      double timeout = 0; for (const auto & s : steps) {timeout += s.timeout;}
      status->setText(
        valid ? QStringLiteral("v1 校验通过 · %1 步 · 超时上限合计 %2s\nSHA-256: %3")
        .arg(steps.size()).arg(timeout).arg(QString::fromStdString(r->hash)) :
        QStringLiteral(
          "旧式 legacy_flat 格式：仅显示正文，不自动转换、不允许 v1 表单编辑或导出。"));
    } else if (r->job.operation == DraftOperation::PrepareDirectory) {
      directory->setText(QString::fromStdString(r->path)); root = QString::fromStdString(r->path);
      status->setText(QStringLiteral("专用草稿目录已确认：") + root);
      validate();
    } else if (r->job.operation == DraftOperation::InspectExport) {
      auto j = r->job; j.operation = DraftOperation::Export; j.expected = r->fingerprint;
      auto write = [this, j] {
          if (revision == j.revision && content == j.content) {
            submit(j, true);
          }
        };
      if (r->fingerprint.exists) {
        question(
          QStringLiteral("覆盖专用目录内的草稿？"), QStringLiteral(
            "仅替换：%1/%2\n当前 SHA-256: %3\n原内容保留为隐藏备份；确认后目标变化则拒绝导出。")
          .arg(
            QString::fromStdString(j.path), QString::fromStdString(j.filename),
            QString::fromStdString(r->fingerprint.hash)), write);
      } else {write();}
    } else if (r->job.operation == DraftOperation::Export) {
      if (revision == r->job.revision && content == r->job.content) {
        clean_content = content;
        exported_root = QString::fromStdString(r->job.path);
        exported = ExportedDraft{revision, r->job.filename, content,
          mission::missionContentSha256(content)};
        notify();
      }
      status->setText(
        QStringLiteral("已导出本地草稿（此操作不导入、不执行）：") +
        QString::fromStdString(r->path) +
        "\nSHA-256: " + QString::fromStdString(mission::missionContentSha256(r->job.content)) +
        (r->backup.empty() ? QString{} : QStringLiteral("\n原内容备份：") +
        QString::fromStdString(r->backup)));
    }
    enable();
  }
};
MissionEditor::MissionEditor(
  QWidget * parent, std::vector<std::string> paths,
  std::function<void()> changed)
: QWidget(parent), impl_(std::make_unique<Impl>(this, std::move(paths), std::move(changed))) {}
MissionEditor::~MissionEditor() = default;
bool MissionEditor::dirty() const {return impl_->dirty();}
bool MissionEditor::busy() const {return impl_->busy();}
bool MissionEditor::stopped() const {return impl_->worker.state().stopped;}
std::optional<ExportedDraft> MissionEditor::exported_draft() const
{
  const auto & i = *impl_;
  if (!i.exported || i.closing || i.busy() || i.form_dirty || !i.valid ||
    i.valid_revision != i.revision || i.exported->revision != i.revision ||
    i.exported->content != i.content || i.exported_root != i.root ||
    i.name->text().toUtf8().toStdString() != i.exported->name) {return {};}
  return i.exported;
}
void MissionEditor::stop()
{
  impl_->closing = true; impl_->debounce->stop(); impl_->worker.stop(); impl_->enable();
}
void MissionEditor::open_file(const QString & p) {impl_->open(p);}
void MissionEditor::prepare_directory(const QString & p) {impl_->prepare(p);}
void MissionEditor::export_file(const QString & n) {impl_->export_to(n);}
}  // namespace easyfly_desktop
