// Header
#include "DssVtpMaterialVolumesClassified.h"

// VTK Libraries
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
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
#include "vtkCubeSource.h"
#include "vtkFieldData.h"
#include "vtkCellData.h"
#include "vtkAppendPolyData.h"

// DSIC Code Libraries
#include "dsclib/math/physics/Vector3.h"

// Boost Libraries
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

// other libraries
#include <math.h>
#include <unistd.h>
#include <time.h>


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
class DssVtpMaterialVolumesClassified::vtkInternal
{
public:
	vtkInternal()
	{
		this->NewData = false;
	}

	~vtkInternal(){}

	vtkSmartPointer<vtkPolyData> GetLatestPolyData()
	{
		boost::lock_guard<boost::mutex> lock(this->mutex);
		vtkSmartPointer<vtkPolyData> polyData = this->PolyData;
		this->PolyData = NULL;
		this->NewData = false;
		return polyData;
	}

	bool HasData()
	{
		boost::lock_guard<boost::mutex> lock(this->mutex);
		return this->NewData;
	}

	boost::mutex mutex;
	vtkSmartPointer<vtkPolyData> PolyData;
	bool NewData;
};

/*----------------------------------------------------------------------------*/
vtkStandardNewMacro(DssVtpMaterialVolumesClassified);
vtkInformationKeyMacro(DssVtpMaterialVolumesClassified, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
DssVtpMaterialVolumesClassified::DssVtpMaterialVolumesClassified() :
  machine_name_("AMT")
  , high_water_mark_offset_(-1)
{
  SetMachineName(4); // 1 = HMT, 2 = EMT, 3 = SMT

	this->Internal = new vtkInternal;
	this->connected_to_kafka_ = false;

	this->SetNumberOfInputPorts(0);
	this->SetNumberOfOutputPorts(1);
}

/*----------------------------------------------------------------------------*/
DssVtpMaterialVolumesClassified::~DssVtpMaterialVolumesClassified()
{
	delete this->Internal;
}

/*----------------------------------------------------------------------------*/
bool DssVtpMaterialVolumesClassified::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::UpdatePipeline()
{
	if (this->HasData())
	{
		this->Modified();
    this->Update();
	}
}

/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::Connect()
{

  KafkaConnectionInfo kafkaConnectionInfo(std::string("dss.vtp.material.volumes.classified"),
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
  high_water_mark_offset_ = highWaterMarkOffset - 1;
}
void DssVtpMaterialVolumesClassified::Disconnect()
{
  if(connected_to_kafka_)
  {
	  delete pKafkaVTKDataTransport_;
    connected_to_kafka_ = false;
  }
}
/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::ReadKafkaBus()
{
  if(connected_to_kafka_)
  {
    int64_t lowWatermarkOffset=-1;
    int64_t highWaterMarkOffset=-1;
    pKafkaVTKDataTransport_->GetKafkaAPI()->GetWatermarkOffsets(lowWatermarkOffset,highWaterMarkOffset);
    std::cout << "last high_water_mark_offset_:  " << high_water_mark_offset_ << " HighWaterMarkOffset: " << highWaterMarkOffset << std::endl;
    if(highWaterMarkOffset > high_water_mark_offset_ && highWaterMarkOffset > 0)
    {
      vtkSmartPointer<vtkPolyData> polyData;
      std::cout << "reading vtk data transport from Kafka\n" << std::endl;
      polyData = pKafkaVTKDataTransport_->ReadPolyData();
      std::cout << "read file from transport\n" << std::endl;
      if(polyData == NULL)
      {
        std::cout << "got the polyData but it was NULL!!!!" << std::endl;
      }
      else
      {
        UpdatePipelineWithPolyData(polyData);
        high_water_mark_offset_ = highWaterMarkOffset;
      }
    }
    else
    {
      std::cout << "high_water_mark_offset_ is " << high_water_mark_offset_ << " and got HighWaterMarkOffset from Kafka of: " << highWaterMarkOffset << ".  Didn't read data" << std::endl;
    }
  }
  else
  {
    std::cout << "Not connected to Kafka, didn't read" << std::endl;
  }
}
void DssVtpMaterialVolumesClassified::ForceReadKafkaBus()
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
void DssVtpMaterialVolumesClassified::Clear()
{
	vtkSmartPointer<vtkPolyData> emptyPolyData = vtkSmartPointer<vtkPolyData>::New();

	this->Internal->PolyData = emptyPolyData;
	this->Internal->NewData = true;
}

void DssVtpMaterialVolumesClassified::UpdatePipelineWithPolyData(vtkPolyData* polyData)
{
	if (polyData != NULL)
	{
		vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();

    std::cout << "polydata has " << polyData->GetFieldData()->GetArray("XMin")->GetNumberOfTuples() << " field data rows (objects)" << std::endl;
		for (int row = 0; row < polyData->GetFieldData()->GetArray("XMin")->GetNumberOfTuples(); row++)
		{
			vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
			cube->SetBounds(polyData->GetFieldData()->GetArray("XMin")->GetTuple1(row),
							polyData->GetFieldData()->GetArray("XMax")->GetTuple1(row),
							polyData->GetFieldData()->GetArray("YMin")->GetTuple1(row),
							polyData->GetFieldData()->GetArray("YMax")->GetTuple1(row),
							polyData->GetFieldData()->GetArray("ZMin")->GetTuple1(row),
							polyData->GetFieldData()->GetArray("ZMax")->GetTuple1(row));
			cube->Update();
      std::cout << "created object " << row << std::endl;

			for (int field = 0; field < polyData->GetFieldData()->GetNumberOfArrays()-1; field++)
			{
				vtkSmartPointer<vtkDoubleArray> feature = vtkSmartPointer<vtkDoubleArray>::New();
				feature->SetName(polyData->GetFieldData()->GetArray(field)->GetName());
        std::cout << "adding feature:" << feature->GetName() << " with " << polyData->GetFieldData()->GetArray(field)->GetNumberOfTuples() << " object entries" << std::endl;

				for (int cubeFace = 0; cubeFace < 6; cubeFace++)
				{
          if(polyData->GetFieldData()->GetArray(field)->GetTuple1(row) > 0)
          {
					  feature->InsertNextTuple1(polyData->GetFieldData()->GetArray(field)->GetTuple1(row));
          }
          else
          {
            feature->InsertNextTuple1(0);
          }
				}

				cube->GetOutput()->GetCellData()->AddArray(feature);
			}

			appendPolyData->AddInputData(cube->GetOutput());
		}

		appendPolyData->Update();

		this->Internal->PolyData = appendPolyData->GetOutput();
		this->Internal->NewData = true;
	}
}

/*----------------------------------------------------------------------------*/
int DssVtpMaterialVolumesClassified::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation* outInfo = outputVector->GetInformationObject(0);
	outInfo->Set(DssVtpMaterialVolumesClassified::DESCRIPTIVE_NAME(), "DssVtpMaterialVolumesClassified");

	vtkSmartPointer<vtkPolyData> kafka_output = vtkPolyData::GetData(outputVector);

	if (!this->HasData())
	{
		return 1;
	}

	kafka_output->ShallowCopy(this->Internal->GetLatestPolyData());
}

/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
/*----------------------------------------------------------------------------*/
void DssVtpMaterialVolumesClassified::SetMachineName(int name_enum)
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
