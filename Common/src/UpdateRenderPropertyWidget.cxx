#include "UpdateRenderPropertyWidget.h"

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
#include <QSpinBox>
#include <QPushButton>

//-----------------------------------------------------------------------------
UpdateRenderPropertyWidget::UpdateRenderPropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject)
  : Superclass(smproxy, parentObject)
{
  std::cout << "UpdateRenderPropertyWidget::UpdateRenderPropertyWidget()" << std::endl;
  this->setShowLabel(false);

  QGridLayout* gridLayout = new QGridLayout(this);
  gridLayout->setMargin(pqPropertiesPanel::suggestedMargin());
  gridLayout->setHorizontalSpacing(pqPropertiesPanel::suggestedHorizontalSpacing());
  gridLayout->setVerticalSpacing(pqPropertiesPanel::suggestedVerticalSpacing());
  gridLayout->setColumnStretch(0, 0);
  gridLayout->setColumnStretch(1, 1);

  QPushButton* button = new QPushButton("- Update Renderer -", this);
  button->setObjectName("UpdateButton");
  this->addPropertyLink(button, "released", SIGNAL(released()), smproperty);
  this->connect(button, SIGNAL(released()), SLOT(UpdateRenderPressed()));
  gridLayout->addWidget(button);

}
void UpdateRenderPropertyWidget::UpdateRenderPressed()
{
  std::cout << "UpdateRenderPropertyWidget::UpdateRenderPressed()" << std::endl;
  vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
  if (!sourceProxy)
  {
    return;
  }
  std::cout << "made it past the sourceProxy" << std::endl;
  vtkSMIntVectorProperty* prop = vtkSMIntVectorProperty::SafeDownCast(sourceProxy->GetProperty("HasData"));
  if (!prop)
  {
    return;
  }
  std::cout << "made it past the property" << std::endl;
  sourceProxy->InvokeCommand("UpdatePipeline");  // this is the name in the proxy, not the direct command!
  sourceProxy->UpdateVTKObjects();
  sourceProxy->UpdatePropertyInformation(prop);
  int hasNewData = prop->GetElement(0);
  if ( hasNewData && this->view() )
  {
    std::cout << "HadNewData and the view, updating" << std::endl;
    // vtkSMViewProxy* viewProxy = vtkSMViewProxy::SafeDownCast(this->proxy());
    // viewProxy->GetRenderWindow()->Render();
    // pqApplicationCore *app = pqApplicationCore::instance();
    // app->render();
    this->view()->forceRender();
    // this->view()->render();
  }
}

//-----------------------------------------------------------------------------
UpdateRenderPropertyWidget::~UpdateRenderPropertyWidget()
{
}
