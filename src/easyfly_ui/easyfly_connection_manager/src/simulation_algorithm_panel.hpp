// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <QJsonArray>
#include <QMap>
#include <QWidget>

#include <memory>

class QLabel;
class QComboBox;

namespace easyfly_connection_manager
{
class SimulationAlgorithmPanel : public QWidget
{
  Q_OBJECT
public:
  explicit SimulationAlgorithmPanel(const QJsonArray & fields, QWidget * parent = nullptr);
  ~SimulationAlgorithmPanel() override;

  // The columns scroll independently; their height follows the available viewport.
  bool hasHeightForWidth() const override {return false;}

  const QMap<QString, QWidget *> & controls() const;
  QLabel * slamSummary() const;
  void revealControl(QWidget * control);
  void refreshSummary();
  void setProfileSelector(QComboBox * selector);

signals:
  void moduleSelectionChanged();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_connection_manager
