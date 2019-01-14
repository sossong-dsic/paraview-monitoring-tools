#ifndef UpdateRenderPropertyWidget_h
#define UpdateRenderPropertyWidget_h

#include "pqPropertyWidget.h"
#include <QTimer>

class UpdateRenderPropertyWidget : public pqPropertyWidget
{
  Q_OBJECT
  typedef pqPropertyWidget Superclass;

public:
  UpdateRenderPropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject = 0);
  virtual ~UpdateRenderPropertyWidget();

public slots:
  void UpdateRenderPressed();
private:
  Q_DISABLE_COPY(UpdateRenderPropertyWidget)
};

#endif
