#ifndef AutoReadKafkaPropertyWidget_h
#define AutoReadKafkaPropertyWidget_h

#include "pqPropertyWidget.h"
#include <QTimer>
#include <QSpinBox>

class AutoReadKafkaPropertyWidget : public pqPropertyWidget
{
  Q_OBJECT
  typedef pqPropertyWidget Superclass;

public:
  AutoReadKafkaPropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject = 0);
  virtual ~AutoReadKafkaPropertyWidget();

public slots:
  bool AutoReadKafkaChecked(bool checked);
  void SetUpdateInterval(int val) { update_interval_ = val;}
  int GetUpdateInterval() { return update_interval_; }
  void onPollSource();
protected:
  QTimer* Timer;
private:
  int update_interval_;
  QSpinBox* spinbox;
  Q_DISABLE_COPY(AutoReadKafkaPropertyWidget)
};

#endif
