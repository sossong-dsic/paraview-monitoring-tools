#ifndef AutoUpdateRenderPropertyWidget_h
#define AutoUpdateRenderPropertyWidget_h

#include "pqPropertyWidget.h"
#include <QTimer>
#include <QSpinBox>

class AutoUpdateRenderPropertyWidget : public pqPropertyWidget
{
  Q_OBJECT
  typedef pqPropertyWidget Superclass;

public:
  AutoUpdateRenderPropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject = 0);
  virtual ~AutoUpdateRenderPropertyWidget();

public slots:
  bool AutoUpdateRenderChecked(bool checked);
  void SetUpdateInterval(int val) { update_interval_ = val;}
  int GetUpdateInterval() { return update_interval_; }
  void onPollSource();
protected:
  QTimer* Timer;
private:
  int update_interval_;
  QSpinBox* spinbox;
  Q_DISABLE_COPY(AutoUpdateRenderPropertyWidget)
};

#endif
