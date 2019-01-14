#include "AutoUpdateRenderPropertyWidget.h"

#include <vtkSMSourceProxy.h>
#include <vtkSMViewProxy.h>
#include <vtkSMIntVectorProperty.h>
#include <vtkRenderWindow.h>

#include "pqPropertiesPanel.h"
#include "pqApplicationCore.h"
#include "pqView.h"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>

//-----------------------------------------------------------------------------
AutoUpdateRenderPropertyWidget::AutoUpdateRenderPropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject)
  : Superclass(smproxy, parentObject)
{
  update_interval_ = 200;

  this->setShowLabel(false);

  QGridLayout* gridLayout = new QGridLayout(this);
  gridLayout->setMargin(pqPropertiesPanel::suggestedMargin());
  gridLayout->setHorizontalSpacing(pqPropertiesPanel::suggestedHorizontalSpacing());
  gridLayout->setVerticalSpacing(pqPropertiesPanel::suggestedVerticalSpacing());
  gridLayout->setColumnStretch(0, 0);
  gridLayout->setColumnStretch(1, 1);
  gridLayout->setColumnStretch(2, 2);

  QCheckBox* checkbox = new QCheckBox("Auto Update Renderer", this);
  checkbox->setObjectName("AutoUpdateCheckbox");
  this->addPropertyLink(checkbox, "checked", SIGNAL(toggled(bool)), smproperty);
  this->connect(checkbox, SIGNAL(toggled(bool)), SLOT(AutoUpdateRenderChecked(bool)));
  gridLayout->addWidget(checkbox);
  // since there's no such thing a "editing" and 'editing done' for checkboxes.
  this->setChangeAvailableAsChangeFinished(true);

  spinbox = new QSpinBox(this);
  spinbox->setObjectName("IntervalSpinBox");
  this->connect(spinbox, SIGNAL(valueChanged(int)), SLOT(SetUpdateInterval(int)));
  spinbox->setRange(100, 10000);
  spinbox->setSingleStep(100);
  spinbox->setValue(update_interval_);
  gridLayout->addWidget(spinbox);
  QLabel* customLabel = new QLabel("(ms)", this);
  gridLayout->addWidget(customLabel);
  //
  std::cout << "Creating Timer with interval " << update_interval_ << std::endl;
  this->Timer = new QTimer(this);
  this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));
  this->Timer->setInterval( update_interval_ );  
  this->addPropertyLink(this->Timer, "timeout", SIGNAL(timeout()), smproperty);

}
bool AutoUpdateRenderPropertyWidget::AutoUpdateRenderChecked(bool checked)
{
  if (checked)
  {
    printf("%s: Automatically Updating Pipeline\n", __PRETTY_FUNCTION__);
    spinbox->setEnabled(false); 
    this->Timer->setInterval( update_interval_ );
    this->Timer->start();
  }
  else
  {
    printf("%s: Stopping Automatically Updating Pipeline\n", __PRETTY_FUNCTION__);
    this->Timer->stop();
    spinbox->setEnabled(true); 
  }
}
void AutoUpdateRenderPropertyWidget::onPollSource()
{
  vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
  if (!sourceProxy)
  {
    return;
  }
  vtkSMIntVectorProperty* prop = vtkSMIntVectorProperty::SafeDownCast(sourceProxy->GetProperty("HasData"));
  if (!prop)
  {
    return;
  }
  printf("%s: Updating Pipeline and View\n", __PRETTY_FUNCTION__);
  sourceProxy->InvokeCommand("UpdatePipeline");  // this is the name in the proxy, not the direct command!
  sourceProxy->UpdateVTKObjects();
  sourceProxy->UpdatePropertyInformation(prop);
  int hasNewData = prop->GetElement(0);
  if ( hasNewData && this->view() )
  {
    // vtkSMViewProxy* viewProxy = vtkSMViewProxy::SafeDownCast(this->proxy());
    // viewProxy->GetRenderWindow()->Render();
    // pqApplicationCore *app = pqApplicationCore::instance();
    // app->render();
    this->view()->forceRender();
  }
}

//-----------------------------------------------------------------------------
AutoUpdateRenderPropertyWidget::~AutoUpdateRenderPropertyWidget()
{
}
