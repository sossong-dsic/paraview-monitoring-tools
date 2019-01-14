#include "AutoReadKafkaPropertyWidget.h"

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
AutoReadKafkaPropertyWidget::AutoReadKafkaPropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject)
  : Superclass(smproxy, parentObject)
{
  std::cout << "Constructor!!!!!!!!!" << std::endl;
  update_interval_ = 30;
  std::cout << "Update Interval:  " << update_interval_ << std::endl;

  this->setShowLabel(false);

  QGridLayout* gridLayout = new QGridLayout(this);
  gridLayout->setMargin(pqPropertiesPanel::suggestedMargin());
  gridLayout->setHorizontalSpacing(pqPropertiesPanel::suggestedHorizontalSpacing());
  gridLayout->setVerticalSpacing(pqPropertiesPanel::suggestedVerticalSpacing());
  gridLayout->setColumnStretch(0, 0);
  gridLayout->setColumnStretch(1, 1);
  gridLayout->setColumnStretch(2, 2);

  QCheckBox* checkbox = new QCheckBox("Auto Read From Kafka", this);
  checkbox->setObjectName("AutoReadKafka");
  this->addPropertyLink(checkbox, "checked", SIGNAL(toggled(bool)), smproperty);
  this->connect(checkbox, SIGNAL(toggled(bool)), SLOT(AutoReadKafkaChecked(bool)));
  gridLayout->addWidget(checkbox);
  // since there's no such thing a "editing" and 'editing done' for checkboxes.
  this->setChangeAvailableAsChangeFinished(true);

  spinbox = new QSpinBox(this);
  spinbox->setObjectName("IntervalSpinBox");
  std::cout << "Setting spin box to " << update_interval_ << " seconds" << std::endl;
  spinbox->setValue(update_interval_);
  spinbox->setRange(1, 60);
  spinbox->setSingleStep(1);
  this->connect(spinbox, SIGNAL(valueChanged(int)), SLOT(SetUpdateInterval(int)));
  gridLayout->addWidget(spinbox);
  QLabel* customLabel = new QLabel("(seconds)", this);
  gridLayout->addWidget(customLabel);
  //
  std::cout << "Creating Timer with interval " << update_interval_ << " seconds" << std::endl;
  this->Timer = new QTimer(this);
  this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));
  this->Timer->setInterval( update_interval_ * 1000 );  
  this->addPropertyLink(this->Timer, "timeout", SIGNAL(timeout()), smproperty);

}
bool AutoReadKafkaPropertyWidget::AutoReadKafkaChecked(bool checked)
{
  if (checked)
  {
    printf("%s: Automatically Reading From Kafka Bus\n", __PRETTY_FUNCTION__);
    spinbox->setEnabled(false); 
    this->Timer->setInterval( update_interval_ * 1000);
    this->Timer->start();
  }
  else
  {
    printf("%s: Stopping Automatically Reading From Kafka Bus\n", __PRETTY_FUNCTION__);
    this->Timer->stop();
    spinbox->setEnabled(true); 
  }
}
void AutoReadKafkaPropertyWidget::onPollSource()
{
  vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
  if (!sourceProxy)
  {
    return;
  }

  printf("%s: Reading from Kafka\n", __PRETTY_FUNCTION__);
  sourceProxy->InvokeCommand("ReadKafkaBus");  // this is the name in the proxy, not the direct command!

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
AutoReadKafkaPropertyWidget::~AutoReadKafkaPropertyWidget()
{
}
