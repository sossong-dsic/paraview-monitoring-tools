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

class ConnectToVTPGammaDetected: public pqObjectPanel
{

	Q_OBJECT

public:

	ConnectToVTPGammaDetected(pqProxy* proxy, QWidget* p) : pqObjectPanel(proxy, p)
	{
		QVBoxLayout* layout = new QVBoxLayout(this);

		this->Timer = new QTimer(this);
		this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));

		this->KafkaTimer = new QTimer(this);
		this->connect(this->KafkaTimer, SIGNAL(timeout()), SLOT(readFromKafka()));

		QPushButton* readButton = new QPushButton("Read");
		this->connect(readButton, SIGNAL(clicked()), SLOT(onReadClicked()));
		layout->addWidget(readButton);

		layout->addStretch();

		QCheckBox* automaticallyReadKafkaBusCheck = new QCheckBox("Automatically Read From Kafka");
		this->connect(automaticallyReadKafkaBusCheck, SIGNAL(toggled(bool)), SLOT(automaticallyReadKafkaBusChecked(bool)));
		layout->addWidget(automaticallyReadKafkaBusCheck);

		QSlider* sliderKafka = new QSlider(Qt::Horizontal);
		this->connect(sliderKafka, SIGNAL(valueChanged(int)), SLOT(onKafkaSliderValueChanged(int)));
		layout->addWidget(sliderKafka);
		this->AutomaticallyReadKafkaBusLabel = new QLabel();
		layout->addWidget(this->AutomaticallyReadKafkaBusLabel);

		layout->addWidget(new QLabel);

		sliderKafka->setMinimum(1);
		sliderKafka->setMaximum(200);
		sliderKafka->setValue(100);

		QPushButton* refreshButton = new QPushButton("Update");
		this->connect(refreshButton, SIGNAL(clicked()), SLOT(onRefreshClicked()));
		layout->addWidget(refreshButton);

		layout->addStretch();

		QCheckBox* automaticallyUpdatePipelineCheck = new QCheckBox("Automatically Update Render");
		this->connect(automaticallyUpdatePipelineCheck, SIGNAL(toggled(bool)), SLOT(automaticallyUpdatePipelineChecked(bool)));
		layout->addWidget(automaticallyUpdatePipelineCheck);

		QSlider* slider = new QSlider(Qt::Horizontal);
		this->connect(slider, SIGNAL(valueChanged(int)), SLOT(onSliderValueChanged(int)));
		layout->addWidget(slider);
		this->AutomaticallyUpdatePipelineLabel = new QLabel();
		layout->addWidget(this->AutomaticallyUpdatePipelineLabel);

		layout->addWidget(new QLabel);

		slider->setMinimum(1);
		slider->setMaximum(200);
		slider->setValue(100);

		QPushButton* clearButton = new QPushButton("Clear");
		this->connect(clearButton, SIGNAL(clicked()), SLOT(onClearClicked()));
		layout->addWidget(clearButton);

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
		this->AutomaticallyUpdatePipelineLabel->setText(QString("Auto Refresh Timeout: %0 s").arg(timeout/50000.0,  0, 'f', 2));
	}

	bool automaticallyReadKafkaBusChecked(bool checked)
	{
		printf("%s: Automatically Reading From Kafka\n", __PRETTY_FUNCTION__);
		if (checked)
		{
			this->KafkaTimer->start();
		}
		else
		{
			this->KafkaTimer->stop();
		}
	}

	void onReadClicked()
	{
		vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

		if (!sourceProxy)
		{
			return;
		}

		sourceProxy->InvokeCommand("ReadKafkaBus");
	}

	void onKafkaSliderValueChanged(int sliderValue)
	{
		int timeoutMax = 30000;
		int timeout = timeoutMax * sliderValue / 100.0;
		this->KafkaTimer->setInterval(timeout);
		this->AutomaticallyReadKafkaBusLabel->setText(QString("Auto Refresh Timeout: %0 s").arg(timeout/5000.0,  0, 'f', 2));
	}

	void onRefreshClicked()
	{
		this->onPollSource();
	}

	void readFromKafka()
	{
		vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());

		if (!sourceProxy)
		{
			return;
		}

		sourceProxy->InvokeCommand("ReadKafkaBus");
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

protected:

	QLabel* AutomaticallyUpdatePipelineLabel;
	QTimer* Timer;

	QLabel* AutomaticallyReadKafkaBusLabel;
	QTimer* KafkaTimer;
};
