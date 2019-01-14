#include "pqObjectPanel.h"

#include "pqView.h"
#include "pqPipelineRepresentation.h"
#include "pqApplicationCore.h"

#include <vtkSMSourceProxy.h>
#include <vtkSMIntVectorProperty.h>

#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QTimer>

class ConnectToHitPulseData: public pqObjectPanel
{

  Q_OBJECT

public:

  ConnectToHitPulseData(pqProxy* proxy, QWidget* p) : pqObjectPanel(proxy, p)
  {
    QVBoxLayout* layout = new QVBoxLayout(this);

    this->Timer = new QTimer(this);
    this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));

    QCheckBox* check = new QCheckBox("Connect To Zero-MQ");
    this->connect(check, SIGNAL(toggled(bool)), SLOT(connectToServerChecked(bool)));
    layout->addWidget(check);

    QCheckBox* automaticallyUpdatePipelineCheck = new QCheckBox("Automatically Update Pipeline");
    this->connect(automaticallyUpdatePipelineCheck, SIGNAL(toggled(bool)), SLOT(automaticallyUpdatePipelineChecked(bool)));
    layout->addWidget(automaticallyUpdatePipelineCheck);

    QSlider* slider = new QSlider(Qt::Horizontal);
    this->connect(slider, SIGNAL(valueChanged(int)), SLOT(onSliderValueChanged(int)));
    layout->addWidget(slider);
    this->AutomaticallyUpdatePipelineLabel = new QLabel();
    layout->addWidget(this->AutomaticallyUpdatePipelineLabel);

    layout->addWidget(new QLabel);

    QPushButton* refreshButton = new QPushButton("Update");
    this->connect(refreshButton, SIGNAL(clicked()), SLOT(onRefreshClicked()));
    layout->addWidget(refreshButton);

    layout->addStretch();

    slider->setMinimum(1);
    slider->setMaximum(200);
    slider->setValue(100);

    QPushButton* clearButton = new QPushButton("Clear");
    this->connect(clearButton, SIGNAL(clicked()), SLOT(onClearClicked()));
    layout->addWidget(clearButton);
    
    layout->addStretch();

    QCheckBox* subcheck = new QCheckBox("Subtract Background");
    this->connect(subcheck, SIGNAL(toggled(bool)), SLOT(subBackgroundChecked(bool)));
    layout->addWidget(subcheck);


    QPushButton* buildRateButton = new QPushButton("Build Rate Background");
    this->connect(buildRateButton, SIGNAL(clicked()), SLOT(onBuildRateClicked()));
    layout->addWidget(buildRateButton);

    QPushButton* loadRateButton = new QPushButton("Load Rate Background");
    this->connect(loadRateButton, SIGNAL(clicked()), SLOT(onLoadRateClicked()));
    layout->addWidget(loadRateButton);

    QPushButton* saveRateButton = new QPushButton("Save Rate Background");
    this->connect(saveRateButton, SIGNAL(clicked()), SLOT(onSaveRateClicked()));
    layout->addWidget(saveRateButton);


    layout->addStretch();

  }

public slots:

  void onClearClicked()
  {
	vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

	if (!sourceProxy)
	{
	  return;
	}

	sourceProxy->InvokeCommand("Clear");
  }

  void connectToServerChecked(bool checked)
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

    if (!sourceProxy)
    {
      return;
    }

    if (checked)
    {
      sourceProxy->InvokeCommand("Connect");
    }

    if (not checked)
    {
      sourceProxy->InvokeCommand("Disconnect");
    }
  }

  bool automaticallyUpdatePipelineChecked(bool checked)
  {
	printf("%s: Automatically Updating Pipeline\n", __PRETTY_FUNCTION__);
    if (checked)
    {
      this->Timer->start();
    }
    else
    {
      this->Timer->stop();
    }
  }

  void onSliderValueChanged(int sliderValue)
  {
    int timeoutMax = 30000;
    int timeout = timeoutMax * sliderValue / 100.0;
    this->Timer->setInterval(timeout);
    this->AutomaticallyUpdatePipelineLabel->setText(QString("Auto refresh timeout: %0 s").arg(timeout/50000.0,  0, 'f', 2));
  }

  void onRefreshClicked()
  {
    this->onPollSource();
  }

  void onPollSource()
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

    sourceProxy->InvokeCommand("Polling");
    sourceProxy->UpdatePropertyInformation(prop);
    int hasNewData = prop->GetElement(0);

    if ( hasNewData && this->view() )
    {
      this->view()->render();
    }
  }
  
  void subBackgroundChecked(bool checked)
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

    if (!sourceProxy)
    {
      return;
    }

    if (checked)
    {
      sourceProxy->InvokeCommand("SetBackgroundSubtractOn");
    }

    if (not checked)
    {
      sourceProxy->InvokeCommand("SetBackgroundSubtractOff");
    }
  }

  void onBuildRateClicked()
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

    if (!sourceProxy)
    {
      return;
    }

    sourceProxy->InvokeCommand("BuildRateBackground");
  }

  void onLoadRateClicked()
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

    if (!sourceProxy)
    {
      return;
    }

    sourceProxy->InvokeCommand("LoadRateBackgroundFile");
  }
  
  void onSaveRateClicked()
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

    if (!sourceProxy)
    {
      return;
    }

    sourceProxy->InvokeCommand("WriteRateBackgroundFile");
  }



protected:

  QLabel* AutomaticallyUpdatePipelineLabel;
  QTimer* Timer;
};
