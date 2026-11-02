// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include "simulation_algorithm_panel.hpp"
#include "easyfly_connection_manager/profile.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSet>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <functional>
#include <vector>

namespace easyfly_connection_manager
{
namespace
{
QLabel * wrappedLabel(const QString & text, QWidget * parent = nullptr)
{
  auto * label = new QLabel(text, parent);
  label->setTextFormat(Qt::PlainText);
  label->setWordWrap(true);
  label->setMinimumWidth(0);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

QScrollArea * scrollArea(QWidget * body, QWidget * parent = nullptr)
{
  auto * area = new QScrollArea(parent);
  area->setFrameShape(QFrame::NoFrame);
  area->setWidgetResizable(true);
  area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  area->setWidget(body);
  return area;
}

QFormLayout * formLayout(QWidget * parent)
{
  auto * form = new QFormLayout(parent);
  form->setContentsMargins(0, 0, 0, 0);
  form->setVerticalSpacing(10);
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  form->setRowWrapPolicy(QFormLayout::WrapLongRows);
  return form;
}

QString moduleFor(const QJsonObject & field)
{
  const auto key = field["key"].toString();
  const auto group = field["group"].toString();
  if (key == "slam" || key.startsWith("slam_") || group.startsWith("slam")) {return "slam";}
  if (key == "strategy" || group == "navigation") {return "navigation";}
  if (key == "exploration") {return "exploration";}
  if (key == "ellipse_enabled" || group == "ellipse") {return "ellipse";}
  if (key == "color_line_enabled" || group == "color_line") {return "color_line";}
  if (key == "single_tracking_backend" || group == "single_tracking") {return "single_tracking";}
  if (key == "multi_tracking_backend" || group == "tracking") {return "tracking";}
  if (key.startsWith("combined_grid_")) {return "combined_grid";}
  if (key.startsWith("global_grid_") || group == "grid") {return "grid";}
  if (key == "provider" || key == "detection_backend" || group == "object_detection") {
    return "object_detection";
  }
  if (group == "input") {return "input";}
  return "general";
}

bool primaryField(const QString & key)
{
  return QStringList{
    "slam", "strategy", "exploration", "ellipse_enabled", "color_line_enabled", "provider",
    "detection_backend", "multi_tracking_backend", "single_tracking_backend",
    "global_grid_backend", "combined_grid_backend", "camera"}.contains(key);
}
}  // namespace

struct SimulationAlgorithmPanel::Impl
{
  struct Module
  {
    QString title;
    QTreeWidgetItem * item{nullptr};
    QWidget * overview{nullptr};
    QFormLayout * choices{nullptr};
    QScrollArea * parameters{nullptr};
    QFormLayout * parameter_form{nullptr};
    QWidget * parameter_row{nullptr};
    QLabel * parameter_hint{nullptr};
    QLabel * summary{nullptr};
    QPushButton * edit{nullptr};
    QStringList keys;
    int parameter_count{0};
  };

  SimulationAlgorithmPanel * owner;
  QMap<QString, QWidget *> controls;
  QMap<QString, QJsonObject> definitions;
  QMap<QString, Module> modules;
  QMap<QString, QJsonObject> recipes;
  QComboBox * profile_selector{nullptr};
  QLabel * selection_notice;
  QLabel * joint_plan;
  bool applying{false};
  QSet<QString> automatic_added;
  QMap<QWidget *, QString> control_modules;
  QTreeWidget * tree;
  QStackedWidget * middle;
  QStackedWidget * side;
  QStackedWidget * parameter_pages;
  QScrollArea * details;
  QLabel * title;
  QLabel * description;
  QLabel * inputs;
  QLabel * dependencies;
  QLabel * outputs;
  QLabel * notes;
  QLabel * slam_summary;
  QLabel * slam_caption;
  QString current;

  Impl(SimulationAlgorithmPanel * panel, const QJsonArray & fields) : owner(panel)
  {
    owner->setObjectName("simulationAlgorithmPanel");
    auto * layout = new QVBoxLayout(owner);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    auto * hint = wrappedLabel(
      QStringLiteral("勾选算法会自动加入配合项；保存后重启仿真生效。"));
    hint->setWordWrap(false); hint->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    layout->addWidget(hint);
    selection_notice = wrappedLabel({});
    selection_notice->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Maximum);
    selection_notice->setObjectName("simulationModuleSelectionNotice");
    selection_notice->hide(); layout->addWidget(selection_notice);
    for (const auto & value : simulation_module_definitions()["modules"].toArray()) {
      const auto row = value.toObject(); recipes.insert(row["id"].toString(), row);
    }
    auto * columns = new QSplitter(Qt::Horizontal, owner);
    columns->setObjectName("simulationAlgorithmColumns");
    columns->setChildrenCollapsible(false);
    tree = new QTreeWidget(columns);
    tree->setObjectName("simulationAlgorithmTree");
    tree->setAccessibleName(QStringLiteral("仿真算法分类"));
    tree->setHeaderHidden(true);
    tree->setMinimumWidth(145);
    tree->setIndentation(14);
    tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    middle = new QStackedWidget(columns);
    middle->setObjectName("simulationAlgorithmOverview");
    middle->setMinimumWidth(220);
    side = new QStackedWidget(columns);
    side->setObjectName("simulationAlgorithmSidePanel");
    side->setMinimumWidth(240);
    columns->addWidget(tree);
    columns->addWidget(middle);
    columns->addWidget(side);
    columns->setStretchFactor(0, 0);
    columns->setStretchFactor(1, 2);
    columns->setStretchFactor(2, 3);
    columns->setSizes({180, 300, 400});
    layout->addWidget(columns, 1);

    auto * detail_body = new QWidget;
    auto * detail_layout = new QVBoxLayout(detail_body);
    detail_layout->setContentsMargins(16, 12, 16, 12);
    detail_layout->setSpacing(10);
    title = wrappedLabel({});
    title->setObjectName("simulationAlgorithmTitle");
    auto font = title->font(); font.setBold(true); title->setFont(font);
    detail_layout->addWidget(title);
    const auto section = [detail_layout](const QString & caption, const char * id) {
        auto * heading = wrappedLabel(caption);
        auto font = heading->font(); font.setBold(true); heading->setFont(font);
        detail_layout->addWidget(heading);
        auto * value = wrappedLabel({}); value->setObjectName(id);
        detail_layout->addWidget(value); return value;
      };
    description = section(QStringLiteral("功能"), "simulationAlgorithmDescription");
    inputs = section(QStringLiteral("输入"), "simulationAlgorithmInputs");
    dependencies = section(QStringLiteral("依赖模块"), "simulationAlgorithmDependencies");
    outputs = section(QStringLiteral("输出与就绪检测"), "simulationAlgorithmOutputs");
    notes = section(QStringLiteral("使用说明"), "simulationAlgorithmNotes");
    slam_caption = wrappedLabel(QStringLiteral("当前运行方案"));
    font = slam_caption->font(); font.setBold(true); slam_caption->setFont(font);
    detail_layout->addWidget(slam_caption);
    slam_summary = wrappedLabel({}); slam_summary->setObjectName("slamConfigurationSummary");
    detail_layout->addWidget(slam_summary);
    joint_plan = section(QStringLiteral("联合启动清单"), "simulationJointStartupPlan");
    detail_layout->addStretch();
    details = scrollArea(detail_body);
    details->setObjectName("simulationAlgorithmDetails"); side->addWidget(details);
    parameter_pages = new QStackedWidget;
    parameter_pages->setObjectName("simulationAlgorithmParameterEditor");
    side->addWidget(parameter_pages);

    const auto localization = category(QStringLiteral("定位与建图"));
    addModule("slam", QStringLiteral("SLAM 定位建图"), localization);
    const auto navigation = category(QStringLiteral("导航与规划"));
    addModule("navigation", QStringLiteral("导航策略"), navigation);
    addModule("grid", QStringLiteral("全局路径搜索"), navigation);
    addModule("combined_grid", QStringLiteral("组合路径搜索"), navigation);
    addModule("exploration", QStringLiteral("自主探索"), navigation);
    const auto perception = category(QStringLiteral("图像感知"));
    addModule("input", QStringLiteral("图像输入"), perception);
    addModule("object_detection", QStringLiteral("目标检测"), perception);
    addModule("ellipse", QStringLiteral("椭圆检测"), perception);
    addModule("color_line", QStringLiteral("彩线检测"), perception);
    addModule("tracking", QStringLiteral("多目标跟踪"), perception);
    addModule("single_tracking", QStringLiteral("单目标跟踪"), perception);
    addModule("pixel_to_3d", QStringLiteral("目标三维定位"), perception);
    addModule("target_prediction", QStringLiteral("目标运动预测"), perception);
    addModule("target_tracking", QStringLiteral("目标跟随"), perception);
    const auto mapping = category(QStringLiteral("地图处理"));
    addModule("pointcloud_filter", QStringLiteral("点云滤波"), mapping);
    addModule("semantic_map", QStringLiteral("障碍与占据地图"), mapping);

    for (const auto & value : fields) {
      const auto field = value.toObject();
      const auto key = field["key"].toString();
      if (key.isEmpty() || controls.contains(key)) {continue;}
      QString id = moduleFor(field);
      bool launch_flag = false;
      for (auto it = recipes.cbegin(); it != recipes.cend(); ++it) {
        if (it.value()["flag"].toString() == key && key.startsWith("start_")) {
          id = it.key(); launch_flag = true; break;
        }
      }
      if (!modules.contains(id)) {
        addModule(id, QStringLiteral("其他算法参数"), category(QStringLiteral("其他设置")));
      }
      auto * widget = makeControl(field);
      widget->setObjectName("algorithm_" + key);
      widget->setAccessibleName(field["label"].toString());
      widget->setMinimumWidth(0);
      controls.insert(key, widget); definitions.insert(key, field);
      control_modules.insert(widget, id);
      auto & module = modules[id]; module.keys.append(key);
      if (launch_flag) {widget->setParent(owner); widget->hide();} else {
        auto * form = primaryField(key) ? module.choices : module.parameter_form;
        auto * label = wrappedLabel(field["label"].toString()); label->setBuddy(widget);
        label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        form->addRow(label, widget);
        if (!primaryField(key)) {++module.parameter_count;}
      }
      const auto changed = [this] {refresh();};
      if (auto * check = qobject_cast<QCheckBox *>(widget)) {
        QObject::connect(check, &QCheckBox::toggled, owner, changed);
      } else if (auto * combo = qobject_cast<QComboBox *>(widget)) {
        QObject::connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), owner, changed);
        if (key == "slam" || key == "strategy" || key == "exploration") {
          QObject::connect(combo, qOverload<int>(&QComboBox::activated), owner,
            [this, key] {userSelectorChanged(key);});
        }
      } else if (auto * edit = qobject_cast<QLineEdit *>(widget)) {
        QObject::connect(edit, &QLineEdit::textChanged, owner, changed);
      } else if (auto * number = qobject_cast<QDoubleSpinBox *>(widget)) {
        QObject::connect(number, qOverload<double>(&QDoubleSpinBox::valueChanged), owner, changed);
      }
    }
    for (auto it = modules.begin(); it != modules.end(); ++it) {
      auto & module = it.value();
      module.item->setHidden(module.keys.isEmpty());
      if (recipes.contains(it.key())) {
        module.item->setFlags(module.item->flags() | Qt::ItemIsUserCheckable);
        module.item->setCheckState(0, Qt::Unchecked);
      }
      module.edit->setEnabled(module.parameter_count > 0);
      module.parameter_row->setVisible(module.parameter_count > 0);
      module.parameter_hint->setText(module.parameter_count > 0 ?
        QStringLiteral("点击编辑，在右侧配置参数。") :
        QStringLiteral("当前模块在上方选择配置，无额外参数。"));
      module.edit->setToolTip(module.parameter_count > 0 ? QStringLiteral("在右侧编辑参数") :
        QStringLiteral("当前模块的可选项均在本栏中设置，无额外参数"));
    }
    tree->expandAll();
    QObject::connect(tree, &QTreeWidget::currentItemChanged, owner,
      [this](QTreeWidgetItem * item, QTreeWidgetItem *) {
        if (!item) {return;}
        const auto id = item->data(0, Qt::UserRole).toString();
        if (id.isEmpty()) {
          for (int i = 0; i < item->childCount(); ++i) {
            if (!item->child(i)->isHidden()) {tree->setCurrentItem(item->child(i)); break;}
          }
          return;
        }
        select(id);
      });
    QObject::connect(tree, &QTreeWidget::itemChanged, owner,
      [this](QTreeWidgetItem * item, int column) {
        if (applying || column != 0 || !item) {return;}
        const auto id = item->data(0, Qt::UserRole).toString();
        if (recipes.contains(id)) {toggleModule(id, item->checkState(0) == Qt::Checked);}
      });
    tree->setCurrentItem(modules["slam"].item);
    refresh();
  }

  QTreeWidgetItem * category(const QString & name)
  {
    auto * item = new QTreeWidgetItem(tree, {name});
    auto font = item->font(0); font.setBold(true); item->setFont(0, font);
    return item;
  }

  void addModule(const QString & id, const QString & name, QTreeWidgetItem * category_item)
  {
    Module module; module.title = name;
    module.item = new QTreeWidgetItem(category_item, {name});
    module.item->setData(0, Qt::UserRole, id);
    auto * body = new QWidget;
    auto * layout = new QVBoxLayout(body);
    layout->setContentsMargins(16, 12, 16, 12); layout->setSpacing(10);
    auto * heading = wrappedLabel(name);
    auto font = heading->font(); font.setBold(true); heading->setFont(font);
    layout->addWidget(heading);
    auto * choices = new QWidget;
    module.choices = formLayout(choices);
    module.choices->setRowWrapPolicy(QFormLayout::WrapAllRows); layout->addWidget(choices);
    auto * parameter_row = new QWidget;
    auto * row = new QHBoxLayout(parameter_row); row->setContentsMargins(0, 0, 0, 0);
    row->addWidget(wrappedLabel(QStringLiteral("内置参数")), 1);
    module.edit = new QPushButton(QStringLiteral("编辑"));
    module.edit->setObjectName("simulationEditParameters_" + id);
    module.edit->setAccessibleName(QStringLiteral("编辑%1参数").arg(name));
    row->addWidget(module.edit);
    auto * parameter_form = new QWidget;
    module.parameter_row = parameter_form;
    auto * parameter_layout = formLayout(parameter_form);
    parameter_layout->addRow(QStringLiteral("参数配置"), parameter_row);
    layout->addWidget(parameter_form);
    module.parameter_hint = wrappedLabel(QStringLiteral("点击编辑，在右侧配置参数。"));
    layout->addWidget(module.parameter_hint);
    module.summary = wrappedLabel({});
    module.summary->setObjectName("simulationAlgorithmSummary_" + id);
    layout->addWidget(module.summary); layout->addStretch();
    module.overview = scrollArea(body);
    module.overview->setObjectName("simulationAlgorithmOverview_" + id);
    middle->addWidget(module.overview);

    auto * editor_body = new QWidget;
    auto * editor_layout = new QVBoxLayout(editor_body);
    editor_layout->setContentsMargins(16, 12, 16, 12); editor_layout->setSpacing(10);
    auto * back = new QPushButton(QStringLiteral("返回说明"));
    back->setObjectName("simulationBackToAlgorithmDetails_" + id);
    editor_layout->addWidget(back);
    auto * editor_heading = wrappedLabel(QStringLiteral("参数编辑 · %1").arg(name));
    font = editor_heading->font(); font.setBold(true); editor_heading->setFont(font);
    editor_layout->addWidget(editor_heading);
    auto * parameter_fields = new QWidget;
    module.parameter_form = formLayout(parameter_fields);
    module.parameter_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
    editor_layout->addWidget(parameter_fields); editor_layout->addStretch();
    module.parameters = scrollArea(editor_body);
    module.parameters->setObjectName("simulationAlgorithmParameters_" + id);
    parameter_pages->addWidget(module.parameters);
    modules.insert(id, module);
    QObject::connect(module.edit, &QPushButton::clicked, owner, [this, id] {edit(id);});
    QObject::connect(back, &QPushButton::clicked, owner, [this] {side->setCurrentWidget(details);});
  }

  QWidget * makeControl(const QJsonObject & field)
  {
    const auto kind = field["kind"].toString();
    const auto value = field["default"];
    if (kind == "bool") {
      auto * check = new QCheckBox; check->setChecked(value.toBool()); return check;
    }
    if (kind == "choice") {
      auto * combo = new QComboBox;
      combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
      combo->setMinimumContentsLength(8);
      for (const auto & entry : field["choices"].toArray()) {
        const auto pair = entry.toArray(); combo->addItem(pair[1].toString(), pair[0].toString());
      }
      combo->setCurrentIndex(combo->findData(value.toString())); return combo;
    }
    if (kind == "string" || kind == "topic") {
      auto * edit = new QLineEdit(value.toString());
      edit->setMaxLength(kind == "topic" ? 256 : 128); return edit;
    }
    auto * number = new QDoubleSpinBox;
    number->setDecimals(kind == "int" ? 0 : 3);
    number->setRange(field["min"].toDouble(), field["max"].toDouble());
    number->setSingleStep(kind == "int" ? 1 : .01);
    number->setValue(value.toDouble()); return number;
  }

  QString selection(const QString & key) const
  {
    if (auto * combo = qobject_cast<QComboBox *>(controls.value(key))) {
      return combo->currentData().toString();
    }
    return {};
  }

  QString displayed(const QString & key) const
  {
    auto * widget = controls.value(key);
    if (auto * combo = qobject_cast<QComboBox *>(widget)) {return combo->currentText();}
    if (auto * check = qobject_cast<QCheckBox *>(widget)) {
      return check->isChecked() ? QStringLiteral("启用") : QStringLiteral("关闭");
    }
    if (auto * line = qobject_cast<QLineEdit *>(widget)) {
      return line->text().isEmpty() ? QStringLiteral("默认") : line->text();
    }
    if (auto * number = qobject_cast<QDoubleSpinBox *>(widget)) {return number->text();}
    return {};
  }

  QString profileName() const
  {
    return profile_selector ? profile_selector->currentData().toString() : "navigation_dev";
  }

  QString localization() const
  {
    if (profileName() == "base_sim") {return "disabled";}
    const auto value = selection("slam"); return value == "profile" ? "fast_lio" : value;
  }

  QString strategy() const
  {
    const auto value = selection("strategy");
    return value == "profile" ? (profileName() == "navigation_dev" ? "combined" : "astar") : value;
  }

  QStringList requirements(const QString & id) const
  {
    QStringList result;
    for (const auto & value : recipes.value(id)["requires"].toArray()) {result << value.toString();}
    for (const auto & value : recipes.value(id)["strategy_requires"].toObject()[strategy()].toArray()) {
      if (!result.contains(value.toString())) {result << value.toString();}
    }
    // A forced localization source is also a dependency, even when its data
    // reaches this module through another observation chain.
    if (recipes.value(id)["forces"].toObject().contains("slam") && !result.contains("slam")) {
      result << "slam";
    }
    return result;
  }

  QString moduleTitle(const QString & id) const
  {
    return recipes.value(id)["title"].toString(modules.value(id).title);
  }

  struct Resolved
  {
    QSet<QString> requested, effective, automatic;
    QMap<QString, QSet<QString>> reasons;
    QMap<QString, QString> blocked;
  };

  Resolved resolve() const
  {
    Resolved result;
    for (auto it = recipes.cbegin(); it != recipes.cend(); ++it) {
      const auto row = it.value(); const auto flag = row["flag"].toString();
      bool enabled = false;
      if (it.key() == "slam") {enabled = localization() != "disabled";} else if (
        it.key() == "exploration") {enabled = selection("exploration") != "none";} else if (
        auto * check = qobject_cast<QCheckBox *>(controls.value(flag)))
      {
        enabled = check->isChecked() && (profileName() != "base_sim" || !flag.startsWith("start_"));
      }
      if (enabled) {result.requested.insert(it.key());}
    }
    std::function<bool(const QString &, const QString &, QSet<QString>)> include =
      [&](const QString & id, const QString & parent, QSet<QString> visiting) {
        if (visiting.contains(id)) {result.blocked[id] = QStringLiteral("依赖关系存在循环"); return false;}
        if (id == "slam" && localization() == "disabled") {
          result.blocked[id] = QStringLiteral("当前方案未启动 SLAM"); return false;
        }
        if (id == "target_tracking" && localization() != "fast_lio") {
          result.blocked[id] = QStringLiteral("当前目标跟随适配器需要 FAST-LIO"); return false;
        }
        if (id == "exploration" && (profileName() != "navigation_dev" ||
          localization() != "fast_lio" || strategy() != "gcopter"))
        {
          result.blocked[id] = QStringLiteral("需要导航开发方案、FAST-LIO 与 GCOPTER"); return false;
        }
        visiting.insert(id);
        for (const auto & dependency : requirements(id)) {
          if (!include(dependency, id, visiting)) {
            result.blocked[id] = QStringLiteral("缺少配合项：%1").arg(moduleTitle(dependency)); return false;
          }
        }
        result.effective.insert(id);
        if (!result.requested.contains(id)) {result.automatic.insert(id);}
        if (!parent.isEmpty()) {result.reasons[id].insert(parent);}
        return true;
      };
    for (const auto & id : result.requested) {include(id, {}, {});}
    return result;
  }

  void setSelection(const QString & key, const QString & value)
  {
    auto * combo = qobject_cast<QComboBox *>(controls.value(key));
    if (!combo) {return;}
    const int index = combo->findData(value); if (index >= 0) {combo->setCurrentIndex(index);}
  }

  void setModule(const QString & id, bool enabled)
  {
    const auto row = recipes.value(id); const auto key = row["flag"].toString();
    if (auto * check = qobject_cast<QCheckBox *>(controls.value(key))) {check->setChecked(enabled);}
    const auto selector = row["selector"].toString();
    if (!selector.isEmpty()) {
      if (!enabled) {setSelection(selector, row["disabled"].toString());} else if (
        selection(selector) == row["disabled"].toString() ||
        (id == "slam" && localization() == "disabled"))
      {setSelection(selector, row["default"].toString());}
    }
  }

  void ensureModule(const QString & id, QSet<QString> visiting = {})
  {
    if (visiting.contains(id) || !recipes.contains(id)) {return;}
    visiting.insert(id); const auto row = recipes[id];
    const auto forces = row["forces"].toObject();
    for (auto it = forces.begin(); it != forces.end(); ++it) {setSelection(it.key(), it.value().toString());}
    if (profile_selector && !row["profile"].toString().isEmpty()) {
      const int index = profile_selector->findData(row["profile"].toString());
      if (index >= 0) {profile_selector->setCurrentIndex(index);}
    }
    setModule(id, true);
    for (const auto & required : requirements(id)) {ensureModule(required, visiting);}
  }

  void notice(const QString & text)
  {
    selection_notice->setText(text); selection_notice->setVisible(!text.isEmpty());
  }

  template<typename Action>
  void applySelection(Action action, bool notify = true)
  {
    if (applying) {return;}
    applying = true;
    {
      std::vector<std::unique_ptr<QSignalBlocker>> blockers;
      for (auto * control : controls) {blockers.emplace_back(std::make_unique<QSignalBlocker>(control));}
      if (profile_selector) {blockers.emplace_back(std::make_unique<QSignalBlocker>(profile_selector));}
      action();
    }
    applying = false; refresh(); if (notify) {emit owner->moduleSelectionChanged();}
  }

  void toggleModule(const QString & id, bool checked)
  {
    const auto row = recipes.value(id);
    const auto control_key = row.contains("flag") ? row["flag"].toString() : row["selector"].toString();
    if (!controls.value(control_key) || !controls[control_key]->isEnabled()) {refresh(); return;}
    const auto before = resolve();
    const auto previous_slam = selection("slam"), previous_strategy = selection("strategy");
    applySelection([&] {
      QStringList changes;
      if (checked) {
        if (profileName() == "base_sim") {
          const auto selected_slam = selection("slam");
          for (const auto & row : recipes) {
            const auto flag = row["flag"].toString();
            if (flag.startsWith("start_")) {
              if (auto * control = qobject_cast<QCheckBox *>(controls.value(flag))) {control->setChecked(false);}
            }
          }
          setSelection("slam", "disabled"); setSelection("exploration", "none");
          if (id == "slam" && selected_slam != "disabled" && selected_slam != "profile") {
            setSelection("slam", selected_slam);
          }
        }
        if (profile_selector && (profileName() == "base_sim" || id == "navigation" || id == "exploration")) {
          const auto target = id == "navigation" || id == "exploration" ? "navigation_dev" : "full_perception";
          const int index = profile_selector->findData(target);
          if (index >= 0 && profile_selector->currentIndex() != index) {
            profile_selector->setCurrentIndex(index); changes << QStringLiteral("运行方案已调整为%1").arg(profile_selector->currentText());
          }
        }
        ensureModule(id);
        const auto after = resolve();
        for (const auto & added : after.effective - before.effective) {
          if (added != id) {automatic_added.insert(added); changes << QStringLiteral("自动加入%1").arg(moduleTitle(added));}
        }
        automatic_added.remove(id);
        if (selection("slam") != previous_slam) {
          changes << QStringLiteral("定位算法已选择%1").arg(displayed("slam"));
        }
        if (selection("strategy") != previous_strategy) {
          changes << QStringLiteral("导航策略已选择%1").arg(displayed("strategy"));
        }
        if (id == "exploration") {changes << QStringLiteral("已配好 FAST-LIO + GCOPTER；不会自动起飞");}
      } else {
        QSet<QString> disabled{id}; bool expanded = true;
        while (expanded) {
          expanded = false;
          for (const auto & candidate : before.requested | before.effective) {
            if (disabled.contains(candidate)) {continue;}
            for (const auto & dependency : requirements(candidate)) {
              if (disabled.contains(dependency)) {disabled.insert(candidate); expanded = true; break;}
            }
          }
        }
        for (const auto & disabled_id : disabled) {
          setModule(disabled_id, false); automatic_added.remove(disabled_id);
          changes << moduleTitle(disabled_id);
        }
        changes = QStringList{QStringLiteral("已关闭：%1。其他已勾选项继续保留。").arg(changes.join(QStringLiteral("、")))};
      }
      const auto after = resolve();
      for (auto it = recipes.cbegin(); it != recipes.cend(); ++it) {
        if (auto * flag = qobject_cast<QCheckBox *>(controls.value(it.value()["flag"].toString()))) {
          flag->setChecked(after.effective.contains(it.key()));
        }
      }
      notice(changes.isEmpty() ? QStringLiteral("算法选择已更新，保存后重启仿真生效。") : changes.join(QStringLiteral("；")));
    });
  }

  void userSelectorChanged(const QString & key)
  {
    if (applying) {return;}
    if (key == "exploration") {toggleModule("exploration", selection(key) != "none"); return;}
    if (key == "slam" && selection(key) == "disabled") {toggleModule("slam", false); return;}
    QStringList removed;
    applySelection([&] {
      if (selection("exploration") == "fuel" &&
        ((key == "slam" && selection(key) != "fast_lio" && selection(key) != "profile") ||
        (key == "strategy" && selection(key) != "gcopter")))
      {setModule("exploration", false); removed << QStringLiteral("FUEL 自主探索");}
      if (key == "slam" && selection(key) != "fast_lio" && selection(key) != "profile") {
        auto * flag = qobject_cast<QCheckBox *>(controls.value("start_target_tracking"));
        if (flag && flag->isChecked()) {flag->setChecked(false); removed << QStringLiteral("目标跟随");}
      }
    }, false);
    toggleModule(key == "slam" ? "slam" : "navigation", true);
    if (!removed.isEmpty()) {notice(QStringLiteral("已保留新的算法选择，并关闭不兼容项：%1。").arg(removed.join(QStringLiteral("、"))));}
  }

  void refreshChecks()
  {
    QSignalBlocker guard(tree); const auto resolved = resolve(); QStringList plan;
    for (auto it = recipes.cbegin(); it != recipes.cend(); ++it) {
      if (!modules.contains(it.key())) {continue;}
      auto * item = modules[it.key()].item;
      const auto key = it.value().contains("flag") ? it.value()["flag"].toString() : it.value()["selector"].toString();
      const bool editable = controls.value(key) && controls[key]->isEnabled();
      auto flags = item->flags();
      item->setFlags(editable ? flags | Qt::ItemIsUserCheckable : flags & ~Qt::ItemIsUserCheckable);
      item->setCheckState(0, resolved.effective.contains(it.key()) ? Qt::Checked : Qt::Unchecked);
      QStringList needed;
      for (const auto & parent : resolved.reasons.value(it.key())) {needed << moduleTitle(parent);}
      QString tooltip = moduleTitle(it.key());
      if (!needed.isEmpty()) {tooltip += QStringLiteral("\n由%1需要").arg(needed.join(QStringLiteral("、")));}
      if (resolved.blocked.contains(it.key())) {tooltip += "\n" + resolved.blocked[it.key()];}
      item->setToolTip(0, tooltip);
      if (resolved.effective.contains(it.key())) {
        const bool automatic = resolved.automatic.contains(it.key()) || automatic_added.contains(it.key());
        plan << moduleTitle(it.key()) + (automatic ? QStringLiteral("（自动加入）") : QString());
      }
    }
    joint_plan->setText(plan.isEmpty() ? QStringLiteral("未选择算法；仿真与飞控连接等基础服务仍由平台提供。") :
      plan.join(QStringLiteral("、")) + QStringLiteral("。\n这是启动配置；运行就绪以工作窗口实际数据为准。"));
  }

  void select(const QString & id)
  {
    if (!modules.contains(id)) {return;}
    if (modules.contains(current)) {
      modules[current].edit->setObjectName("simulationEditParameters_" + current);
    }
    current = id;
    modules[id].edit->setObjectName("simulationEditParameters");
    middle->setCurrentWidget(modules[id].overview);
    parameter_pages->setCurrentWidget(modules[id].parameters);
    side->setCurrentWidget(details);
    refresh();
  }

  void edit(const QString & id)
  {
    if (!modules.contains(id)) {return;}
    if (current != id) {tree->setCurrentItem(modules[id].item);}
    parameter_pages->setCurrentWidget(modules[id].parameters);
    side->setCurrentWidget(parameter_pages);
  }

  void refresh()
  {
    refreshChecks();
    const auto resolved = resolve();
    for (auto it = modules.begin(); it != modules.end(); ++it) {
      QStringList values;
      for (const auto & key : it->keys) {
        if (primaryField(key)) {
          values << QStringLiteral("%1：%2").arg(definitions[key]["label"].toString(), displayed(key));
        }
      }
      values << QStringLiteral("可编辑参数：%1 项").arg(it->parameter_count);
      if (recipes.contains(it.key())) {
        values << (resolved.effective.contains(it.key()) ? QStringLiteral("已勾选，随仿真启动") :
          QStringLiteral("未启动；勾选左侧项目启用"));
        QStringList required, needed;
        for (const auto & id : requirements(it.key())) {required << moduleTitle(id);}
        for (const auto & id : resolved.reasons.value(it.key())) {needed << moduleTitle(id);}
        if (!required.isEmpty()) {values << QStringLiteral("联合配合：%1").arg(required.join(QStringLiteral("、")));}
        if (!needed.isEmpty()) {values << QStringLiteral("由%1需要").arg(needed.join(QStringLiteral("、")));}
        if (resolved.blocked.contains(it.key())) {values << resolved.blocked[it.key()];}
      }
      it->summary->setText(values.join('\n'));
    }
    if (current.isEmpty()) {return;}
    title->setText(modules[current].title);
    slam_caption->setVisible(current == "slam");
    slam_summary->setVisible(current == "slam");
    QString function, input, required, output, help;
    if (current == "slam") {
      const auto backend = selection("slam");
      function = QStringLiteral("估计飞机位姿并构建环境地图，为地图显示和导航提供定位与空间数据。");
      input = backend == "openvins" ? QStringLiteral("Gazebo 前视彩色图像与 IMU。") :
        backend == "orbslam3" ? QStringLiteral("Gazebo RGB-D 图像；惯性模式还需 IMU。") :
        backend == "fast_livo2" ? QStringLiteral("彩色图像、Livox 雷达与 IMU。") :
        QStringLiteral("Livox 雷达与 IMU；跟随方案时以运行方案实际选用的算法为准。");
      required = QStringLiteral("匹配的仿真传感器、时间源和坐标变换；导航还需定位与飞控坐标对齐。");
      output = backend == "fast_lio" ? QStringLiteral("/Odometry 和 /cloud_registered；实际新鲜度以工作窗口为准。") :
        QStringLiteral("所选适配器的位姿、地图和状态输出；实际就绪情况以工作窗口为准。");
      help = QStringLiteral("当前输入及采样、地图参数用于 FAST-LIO。其他定位后端使用各自固定仿真标定，不能套用 FAST-LIO 参数。");
      if (backend == "disabled") {
        function = QStringLiteral("当前选择关闭 SLAM，建图和依赖定位的导航功能可能不可用。");
        input = QStringLiteral("不启动 SLAM 输入链路。");
        output = QStringLiteral("不由本项启动定位或地图输出。");
      }
    } else if (current == "navigation") {
      function = QStringLiteral("选择导航任务采用的路径与轨迹规划策略。");
      input = QStringLiteral("飞机状态、定位和所选规划器需要的地图、目标位置。");
      required = QStringLiteral("导航运行方案、定位、地图、飞行控制服务及对应规划器。");
      output = QStringLiteral("导航进度、规划轨迹与执行结果。选择策略不会自动发送飞行目标。");
      help = QStringLiteral("当前策略：%1。全局与组合路径搜索的具体后端在左侧对应栏目设置。")
        .arg(displayed("strategy"));
    } else if (current == "exploration") {
      function = QStringLiteral("由探索算法选择观测点，并通过导航任务逐步获取未知区域信息。");
      input = QStringLiteral("定位、有效的已观测体素地图，以及探索任务指定的空间范围。");
      required = QStringLiteral("FUEL 当前要求导航开发运行方案、FAST-LIO 和 GCOPTER。");
      output = QStringLiteral("/exploration/status、覆盖进度、观测点和探索任务结果。");
      help = QStringLiteral("选择 FUEL 仅随会话启动探索服务；开始探索仍需在任务中心显式提交任务。关闭时不启动该服务。");
    } else if (current == "input") {
      function = QStringLiteral("配置椭圆、彩线等几何检测使用的图像、结果时效与计算资源。");
      input = QStringLiteral("所选前视或下视相机图像及相机标定。");
      required = QStringLiteral("相机与对应光学坐标系；检测模块需要单独启用。");
      output = QStringLiteral("向几何检测模块提供输入约定，本项本身不生成检测结果。");
      help = QStringLiteral("最大结果年龄用于过滤过期检测；坐标系留空时使用相机默认值。");
    } else if (current == "ellipse" || current == "color_line") {
      const bool ellipse = current == "ellipse";
      function = ellipse ? QStringLiteral("检测图像中的椭圆轮廓并输出几何结果。") :
        QStringLiteral("按颜色范围和形态筛选图像中的线状区域。");
      input = QStringLiteral("“图像输入”中选定的相机图像与公共检测参数。");
      required = QStringLiteral("匹配的相机输入和 OpenCV 图像处理。");
      output = QStringLiteral("检测结果与图像叠加预览；数据有效性在相机与感知页面查看。");
      help = ellipse ? QStringLiteral("边缘低阈值必须小于高阈值。几何检测只输出图像结果，不会自动执行降落。") :
        QStringLiteral("色相允许跨越红色边界；饱和度和明度上下界须有序，形态学核须为奇数。");
    } else if (current == "object_detection") {
      function = QStringLiteral("对相机图像执行目标检测，输出类别、置信度和像素框。");
      input = QStringLiteral("相机图像、检测模型和所选后端需要的运行依赖。");
      required = QStringLiteral("目标检测所在运行方案、检测后端与所选计算设备。");
      output = QStringLiteral("目标检测消息及画面叠加；实际运行后端与计算设备以节点状态为准。");
      help = QStringLiteral("当前后端：%1；计算设备：%2。左侧勾选决定是否启动，后端与参数在本栏配置。")
        .arg(displayed("detection_backend"), displayed("provider"));
    } else if (current == "tracking" || current == "single_tracking") {
      const bool multi = current == "tracking";
      function = multi ? QStringLiteral("关联连续检测帧，为多个目标维持跟踪身份与状态。") :
        QStringLiteral("根据显式选定的目标区域执行单目标图像跟踪。");
      input = multi ? QStringLiteral("按时间顺序的目标检测结果及后端所需的附加输入。") :
        QStringLiteral("相机图像与显式目标区域；需要独立初始化跟踪。");
      required = multi ? QStringLiteral("目标检测、匹配的数据格式及多目标跟踪节点。") :
        QStringLiteral("相机、单目标跟踪节点和所选后端。");
      output = QStringLiteral("目标框、跟踪身份或跟踪状态；不会自动触发飞机跟随任务。");
      help = QStringLiteral("当前后端：%1。左侧勾选启动该服务；单目标跟踪仍需选择目标并初始化。")
        .arg(displayed(multi ? "multi_tracking_backend" : "single_tracking_backend"));
    } else if (current == "pixel_to_3d") {
      function = QStringLiteral("将图像中的跟踪目标结合深度信息转换为空间位置。");
      input = QStringLiteral("多目标跟踪结果、对齐深度图、相机标定与坐标变换。");
      required = QStringLiteral("多目标跟踪与目标检测会自动加入；相机标定和坐标变换由平台提供。");
      output = QStringLiteral("目标三维位置观测，供目标预测或跟随任务使用。");
      help = QStringLiteral("此项使用平台内置参数。图像、深度与时间戳必须实际有效后才有三维结果。");
    } else if (current == "target_prediction") {
      function = QStringLiteral("利用连续目标三维观测估计运动趋势和未来位置。");
      input = QStringLiteral("带源时间戳的目标三维位置观测。");
      required = QStringLiteral("目标三维定位、跟踪与检测会自动加入。");
      output = QStringLiteral("目标运动与预测位置；过期或无效输入不会作为有效预测。");
      help = QStringLiteral("此项是可选配合能力，使用平台内置预测参数。");
    } else if (current == "target_tracking") {
      function = QStringLiteral("提供飞行目标跟随服务，持续根据目标位置更新跟随目标。");
      input = QStringLiteral("有效的目标三维位置、飞机状态与定位。");
      required = QStringLiteral("当前适配器需要 FAST-LIO、目标三维定位、跟踪和检测；勾选时自动配好。");
      output = QStringLiteral("跟随任务状态和导航目标；执行层仍负责飞行控制与取消。");
      help = QStringLiteral("勾选只启动服务；飞机开始跟随仍需在任务中心提交明确的跟随任务。");
    } else if (current == "pointcloud_filter") {
      function = QStringLiteral("过滤定位点云，为后续障碍与占据地图提供点云输入。");
      input = QStringLiteral("定位与建图模块输出的注册点云。");
      required = QStringLiteral("定位与建图会自动加入，时间和坐标变换沿用平台配置。");
      output = QStringLiteral("过滤后的点云，供地图处理使用。");
      help = QStringLiteral("此项使用平台内置滤波配置；勾选不代表点云已就绪。");
    } else if (current == "semantic_map") {
      function = QStringLiteral("从过滤点云构建障碍表征与占据地图，为二维路径搜索提供环境约束。");
      input = QStringLiteral("过滤后的点云、定位和场景地面配置。");
      required = QStringLiteral("点云滤波与定位建图会自动加入。");
      output = QStringLiteral("障碍与占据地图；导航仍检查地图有效性和新鲜度。");
      help = QStringLiteral("A* 和组合导航需要此项；GCOPTER/FUEL 使用各自适配器内的体素地图。");
    } else if (current == "grid" || current == "combined_grid") {
      const bool combined = current == "combined_grid";
      function = combined ? QStringLiteral("配置组合导航采用的二维栅格路径搜索后端。") :
        QStringLiteral("配置全局导航采用的二维栅格路径搜索后端。");
      input = QStringLiteral("有效的二维占据栅格、起点和目标位置。");
      required = QStringLiteral("定位、占据地图、对应导航协调器；组合导航还依赖局部规划。");
      output = QStringLiteral("二维路径搜索结果，由导航执行层继续检查并执行。");
      help = QStringLiteral("当前后端：%1。搜索格数限制用于控制资源；此处不配置 EGO、GCOPTER 的三维轨迹参数。")
        .arg(displayed(combined ? "combined_grid_backend" : "global_grid_backend"));
    } else {
      function = QStringLiteral("当前算法配置中登记的其他参数。");
      input = QStringLiteral("由对应算法适配器定义。");
      required = QStringLiteral("由运行方案和适配器检查。");
      output = QStringLiteral("以对应节点实际输出为准。");
      help = QStringLiteral("在右侧编辑已登记参数，保存后重启会话生效。");
    }
    description->setText(function); inputs->setText(input); dependencies->setText(required);
    outputs->setText(output); notes->setText(help);
  }
};

SimulationAlgorithmPanel::SimulationAlgorithmPanel(const QJsonArray & fields, QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this, fields))
{
}

SimulationAlgorithmPanel::~SimulationAlgorithmPanel() = default;

const QMap<QString, QWidget *> & SimulationAlgorithmPanel::controls() const
{
  return impl_->controls;
}

QLabel * SimulationAlgorithmPanel::slamSummary() const
{
  return impl_->slam_summary;
}

void SimulationAlgorithmPanel::revealControl(QWidget * control)
{
  const auto id = impl_->control_modules.value(control);
  if (id.isEmpty()) {return;}
  impl_->tree->setCurrentItem(impl_->modules[id].item);
  const auto key = impl_->controls.key(control);
  if (!primaryField(key)) {
    impl_->edit(id);
    impl_->modules[id].parameters->ensureWidgetVisible(control);
  } else {
    impl_->side->setCurrentWidget(impl_->details);
    qobject_cast<QScrollArea *>(impl_->modules[id].overview)->ensureWidgetVisible(control);
  }
}

void SimulationAlgorithmPanel::refreshSummary()
{
  impl_->refresh();
}

void SimulationAlgorithmPanel::setProfileSelector(QComboBox * selector)
{
  impl_->profile_selector = selector;
  if (selector) {
    QObject::connect(selector, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this] {impl_->automatic_added.clear(); impl_->refresh();});
  }
  impl_->refresh();
}
}  // namespace easyfly_connection_manager
