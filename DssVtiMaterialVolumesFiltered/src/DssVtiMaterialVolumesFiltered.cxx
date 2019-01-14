#include "DssVtiMaterialVolumesFiltered.h"

#include "vtkObjectFactory.h"
#include "vtkImageData.h"
#include "vtkPointData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPVInformationKeys.h"
#include "vtkInformationStringKey.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkSmartPointer.h"
#include "vtkDataArray.h"
#include "vtkDoubleArray.h"
#include "vtkIntArray.h"

// DSIC Code Libraries
#include "dsclib/math/physics/Vector3.h"

// Boost Libraries
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

// other libraries
#include <math.h>
#include <unistd.h>

// namespaces
using namespace inf::icomm;

// Global Defines
const std::string DEFAULT_META_DATA_BROKER_LIST = "192.168.10.47:10092";
const std::string DEFAULT_KAFKA_BROKER_LOCAL_HOST = "127.0.0.1:2181";
const std::string DEFAULT_BROKER_VERSION_FALLBACK = "0.8.0.0";
const std::string DEFAULT_MESSAGE_MAX_BYTES         = "1000000000";
const std::string DEFAULT_RECEIVE_MESSAGE_MAX_BYTES = "1000000000";
const std::string DEFAULT_FETCH_MESSAGE_MAX_BYTES   = "1000000000";
const std::string HMT_KAFKA_BROKER = "10.0.220.60:9092";
const std::string EMT_KAFKA_BROKER = "10.0.220.67:9092";
const std::string SMT_KAFKA_BROKER = "10.200.10.28:9092";
const std::string AMT_KAFKA_BROKER = "10.0.221.27:9092";
const int DEFAULT_KAFKA_PARTITION_ID = 0;
const int DEFAULT_KAFKA_BUFFER_FREQUENCY = 100;
const int DEFAULT_OFFSET = RdKafka::Topic::OFFSET_BEGINNING;
const int DEFAULT_READ_TIMEOUT = 50000;
const int DEFAULT_PRODUCER_POLL_TIMEOUT = 1000;
const int DEFAULT_CONSUME_RETRY_MAX = 2;

/*----------------------------------------------------------------------------*/
class DssVtiMaterialVolumesFiltered::vtkInternal
{
public:
	vtkInternal()
{
		this->NewData = false;
}

	~vtkInternal(){}

	bool HasData()
	{
		boost::lock_guard<boost::mutex> lock(this->mutex);
		return this->NewData;
	}

	boost::mutex mutex;
	bool NewData;
};

/*----------------------------------------------------------------------------*/
vtkStandardNewMacro(DssVtiMaterialVolumesFiltered);
vtkInformationKeyMacro(DssVtiMaterialVolumesFiltered, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
DssVtiMaterialVolumesFiltered::DssVtiMaterialVolumesFiltered() :
  machine_name_("AMT")
  , high_water_mark_offset_(-1)
{
  SetMachineName(4); // 1 = HMT, 2 = EMT, 3 = SMT, 3 = AMT


	this->Internal = new vtkInternal;
	this->connected_to_kafka_ = false;

	this->SetNumberOfInputPorts(0);
	this->SetNumberOfOutputPorts(1);

	vtkSmartPointer<vtkImageData> tmpImage = vtkSmartPointer<vtkImageData>::New();
	tmpImage->SetSpacing(3, 3, 3);
	tmpImage->SetOrigin(175, 100, 110);
	tmpImage->SetDimensions(130, 300, 130);

	vtkSmartPointer<vtkDoubleArray> scalarValues = vtkSmartPointer<vtkDoubleArray>::New();

	for (int scalar_index = 0; scalar_index < tmpImage->GetNumberOfPoints(); scalar_index++)
	{
		scalarValues->InsertNextTuple1(0.0);
	}

	tmpImage->GetPointData()->SetScalars(scalarValues);

	kafka_image_ = tmpImage;
}

/*----------------------------------------------------------------------------*/
DssVtiMaterialVolumesFiltered::~DssVtiMaterialVolumesFiltered()
{
	delete this->Internal;
}

/*----------------------------------------------------------------------------*/
bool DssVtiMaterialVolumesFiltered::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::UpdateRenderer()
{
  // std::cout << "DssVtiMaterialVolumesFiltered::UpdateRenderer() called" << std::endl;
	if (this->HasData())
	{
    // std::cout << "HasData() is true, calling Modified()" << std::endl;
		this->Modified();
    this->Update();
	}
}
/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::Connect()
{

	KafkaConnectionInfo kafkaConnectionInfo(std::string("dss.vti.material.volumes.filtered"),
			kafka_broker_name_,
			DEFAULT_BROKER_VERSION_FALLBACK,
			DEFAULT_FETCH_MESSAGE_MAX_BYTES,
			DEFAULT_RECEIVE_MESSAGE_MAX_BYTES,
			DEFAULT_MESSAGE_MAX_BYTES,
			0,
			RD_KAFKA_CONSUMER,
			RdKafka::Consumer::OffsetTail(1),
			0,
			0,
			DEFAULT_READ_TIMEOUT,
			0);
	try
	{
		pKafkaVTKDataTransport_ = new KafkaVTKDataTransport();
		pKafkaVTKDataTransport_->Initialize(kafkaConnectionInfo);
	}
	catch (std::runtime_error &e)
	{
		DSCLIB_LOG(LOG_INFO,"KafkaVTKDataTransportReadWriteTest unit test failed -> %s", e.what());
	}
	connected_to_kafka_ = true;
  int64_t lowWatermarkOffset=-1;
  int64_t highWaterMarkOffset=-1;
  pKafkaVTKDataTransport_->GetKafkaAPI()->GetWatermarkOffsets(lowWatermarkOffset,highWaterMarkOffset);
  high_water_mark_offset_ -= 1;
}
void DssVtiMaterialVolumesFiltered::Disconnect()
{
  if(connected_to_kafka_)
  {
	  delete pKafkaVTKDataTransport_;
    connected_to_kafka_ = false;
  }
}
/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::ReadKafkaBus()
{
  if(connected_to_kafka_)
  {
    int64_t lowWatermarkOffset=-1;
    int64_t highWaterMarkOffset=-1;
    pKafkaVTKDataTransport_->GetKafkaAPI()->GetWatermarkOffsets(lowWatermarkOffset,highWaterMarkOffset);
    std::cout << "last high_water_mark_offset_:  " << high_water_mark_offset_ << " HighWaterMarkOffset: " << highWaterMarkOffset << std::endl;
    if(highWaterMarkOffset > high_water_mark_offset_ && highWaterMarkOffset > 0)
    {
    vtkSmartPointer<vtkImageData> imageData;
		std::cout << "reading vtk data transport from Kafka\n" << std::endl;
		imageData = pKafkaVTKDataTransport_->Read();
		std::cout << "read file from transport\n" << std::endl;
    if(imageData == NULL)
    {
      std::cout << "got the image but it was NULL!!!!" << std::endl;
    }
    else
    {
		  UpdatePipelineWithImage(imageData);
      high_water_mark_offset_ = highWaterMarkOffset;
    }
    }
    else
    {
      std::cout << "high_water_mark_offset_ is " << high_water_mark_offset_ << ".  Didn't read data" << std::endl;
    }
  }
  else
  {
    std::cout << "Not connected to Kafka, didn't read" << std::endl;
  }
}
void DssVtiMaterialVolumesFiltered::ForceReadKafkaBus()
{
  bool was_connected = false;
  if(connected_to_kafka_)
  {
    was_connected = true;
    Disconnect();
  }
  Connect();
  ReadKafkaBus();
  if(!was_connected)
  {
    Disconnect();
  }


}

/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::Clear()
{
	if(kafka_image_ != NULL)
  {
    vtkSmartPointer<vtkImageData> tmpImage = vtkSmartPointer<vtkImageData>::New();
    tmpImage->SetSpacing(kafka_image_->GetSpacing());
    tmpImage->SetOrigin(kafka_image_->GetOrigin());
    tmpImage->SetDimensions(kafka_image_->GetDimensions());

    vtkSmartPointer<vtkDoubleArray> scalarValues = vtkSmartPointer<vtkDoubleArray>::New();

    for (int scalar_index = 0; scalar_index < tmpImage->GetNumberOfPoints(); scalar_index++)
    {
      scalarValues->InsertNextTuple1(0.0);
    }

    tmpImage->GetPointData()->SetScalars(scalarValues);

    kafka_image_ = tmpImage;
  }
  else
  {
    vtkSmartPointer<vtkImageData> tmpImage = vtkSmartPointer<vtkImageData>::New();
	  tmpImage->SetSpacing(3, 3, 3);
	  tmpImage->SetOrigin(171.5, 111.5, 111.5);
	  tmpImage->SetDimensions(137, 270, 143);

    vtkSmartPointer<vtkDoubleArray> scalarValues = vtkSmartPointer<vtkDoubleArray>::New();

    for (int scalar_index = 0; scalar_index < tmpImage->GetNumberOfPoints(); scalar_index++)
    {
      scalarValues->InsertNextTuple1(0.0);
    }

    tmpImage->GetPointData()->SetScalars(scalarValues);

    kafka_image_ = tmpImage;
  }
}

void DssVtiMaterialVolumesFiltered::UpdatePipelineWithImage(vtkImageData* image)
{
	if (image != NULL)
	{
		kafka_image_ = image;
    std::cout << "got the image and set it to kafka_image_, updating pipeline" << std::endl;
		UpdatePipeline();

		this->Internal->NewData = true;
	}
}

/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::UpdatePipeline()
{
	DssVtiMaterialVolumesFiltered::UpdateInformation();
  
	this->Update();
  this->Modified();
}

/*----------------------------------------------------------------------------*/
int DssVtiMaterialVolumesFiltered::RequestInformation(vtkInformation*,
		vtkInformationVector**,
		vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  if (kafka_image_ == NULL)
  {
    int extents[6] = {0, 300, 0, 300, 0, 300};
    outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extents, 6);
    outInfo->Set(vtkDataObject::ORIGIN(), 0.0, 0.0, 0.0);
    outInfo->Set(vtkDataObject::SPACING(), 3, 3, 3);
  }

  else
  {
    int* extents = kafka_image_->GetExtent();
    outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extents, 6);
    double* origin = kafka_image_->GetOrigin();
    double* spacing = kafka_image_->GetSpacing();
    outInfo->Set(vtkDataObject::ORIGIN(), origin, 3);
    outInfo->Set(vtkDataObject::SPACING(), spacing[0], spacing[1], spacing[2]);
  }

  /* Set the Information, Array Type, and Number of Components to comprise the Data Object */
  vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 1);

  /* Make an output port capable of producing an arbitrary subextent of the whole extent */
  outInfo->Set(CAN_PRODUCE_SUB_EXTENT(), 1);
  
	return 1;
}

/*----------------------------------------------------------------------------*/
int DssVtiMaterialVolumesFiltered::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation* outInfo = outputVector->GetInformationObject(0);
	outInfo->Set(DssVtiMaterialVolumesFiltered::DESCRIPTIVE_NAME(), "DssVtiMaterialVolumesFiltered");
	vtkSmartPointer<vtkImageData> recon_output = vtkImageData::GetData(outputVector);

	if (kafka_image_ != NULL)
	{
		recon_output->DeepCopy(kafka_image_);
	}

	return 1;
}

/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
/*----------------------------------------------------------------------------*/
void DssVtiMaterialVolumesFiltered::SetMachineName(int name_enum)
{
  switch (name_enum)
  {
    case 1:
      machine_name_ = "HMT";
      kafka_broker_name_ = HMT_KAFKA_BROKER;
      break;
    case 2:
      machine_name_ = "EMT";
      kafka_broker_name_ = EMT_KAFKA_BROKER;
      break;
    case 3:
      machine_name_ = "SMT";
      kafka_broker_name_ = SMT_KAFKA_BROKER;
      break;
    case 4:
      machine_name_ = "AMT";
      kafka_broker_name_ = AMT_KAFKA_BROKER;
      break;
    default:
      printf("That machine name wasn't on my list!  Defaulting to HMT\n");
      machine_name_ = "HMT";
      kafka_broker_name_ = HMT_KAFKA_BROKER;
      break;
  }
}
