#include "ThroughTrackViewer.h"

#include "vtkObjectFactory.h"
#include "vtkAppendPolyData.h"
#include "vtkPolyData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPVInformationKeys.h"
#include "vtkInformationStringKey.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkSmartPointer.h"
#include "vtkAppendFilter.h"
#include "vtkGeometryFilter.h"
#include "vtkDoubleArray.h"
#include "vtkIntArray.h"
#include "vtkUnstructuredGrid.h"
#include "vtkCellData.h"

#include "ThroughTrackPacket.hh"

#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkPointSource.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>


const std::string HMT_ICOMM_CONNECTION_URI = "tcp://compute2.fmt.red.dsic.com:10005";
const std::string HMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute2.fmt.red.dsic.com:10007";
const std::string EMT_ICOMM_CONNECTION_URI = "tcp://compute4.fmt.red.dsic.com:10005";
const std::string EMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute4.fmt.red.dsic.com:10007";
const std::string SMT_ICOMM_CONNECTION_URI = "tcp://compute2.sgppt0401.dsic.lan:10005";
const std::string SMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute2.sgppt0401.dsic.lan:10007";
const std::string AMT_ICOMM_CONNECTION_URI = "tcp://compute2.cttso.dsic.com:10005";
const std::string AMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute2.cttso.dsic.com:10007";
const std::string NACHO_ICOMM_CONNECTION_URI = "tcp://compute2.cttso.dsic.com:10005";
const std::string NACHO_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute2.cttso.dsic.com:10007";

// Read from Kafka instead
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
const std::string NACHO_KAFKA_BROKER = "10.203.10.27:9092";
const int DEFAULT_KAFKA_PARTITION_ID = 0;
const int DEFAULT_KAFKA_BUFFER_FREQUENCY = 100;
const int DEFAULT_OFFSET = RdKafka::Topic::OFFSET_BEGINNING;
const int DEFAULT_READ_TIMEOUT = 50000;
const int DEFAULT_PRODUCER_POLL_TIMEOUT = 1000;
const int DEFAULT_CONSUME_RETRY_MAX = 2;

/*----------------------------------------------------------------------------*/
class ThroughTrackViewer::vtkInternal
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
vtkStandardNewMacro(ThroughTrackViewer);
vtkInformationKeyMacro(ThroughTrackViewer, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
ThroughTrackViewer::ThroughTrackViewer() :
  running_(false),
  display_interval_(10),
  packet_count_(0),
  min_scattering_angle_cut_(0.050),
  machine_name_("NACHO"),
  filtered_(false)
{
  running_ = false;
  
  SetMachineName(4); // 1 = HMT, 2 = EMT, 3 = SMT

	this->Internal = new vtkInternal;

	this->SetNumberOfInputPorts(0);
	this->SetNumberOfOutputPorts(2);
  
  // initialize the polydata buffers	
  vtkSmartPointer<vtkAppendPolyData> inAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	incomingParticleTrajectoryBuffer_ = inAppendPolyData;
	vtkSmartPointer<vtkAppendPolyData> outAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	outgoingParticleTrajectoryBuffer_ = outAppendPolyData;
	vtkSmartPointer<vtkAppendPolyData> sphereAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	sphereBuffer_ = sphereAppendPolyData;

}

/*----------------------------------------------------------------------------*/
ThroughTrackViewer::~ThroughTrackViewer()
{
	if (running_)
	{
		running_ = false;
		sending_thread_.waitForFinished();
	}

	if (this->Internal != NULL)
	{
    delete this->Internal;
  }
}

/*----------------------------------------------------------------------------*/
bool ThroughTrackViewer::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::UpdatePipeline()
{
	if (this->HasData())
	{
		this->Modified();
    this->Update();
	}
}

/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::Connect()
{
    std::string packet_name = "ThroughTrackPacket";
    if(filtered_)
    {
      packet_name = "FilteredThroughTrackPacket";
    }

	// if(true)
  // {
    std::cout << "connecting to kafka looking for packet_name: " << packet_name << " on broker: " << kafka_broker_name_ << std::endl; 
  KafkaConnectionInfo kafkaConnectionInfo(
      packet_name,
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
		pKafkaTrackReader_ = new KafkaTrackReader();
		pKafkaTrackReader_->Initialize(kafkaConnectionInfo);
	}
	catch (std::runtime_error &e)
	{
		DSCLIB_LOG(LOG_INFO,"KafkaTrackReaderReadWriteTest unit test failed -> %s", e.what());
	}
	connected_to_kafka_ = true;
  int64_t lowWatermarkOffset=-1;
  int64_t highWaterMarkOffset=-1;
  pKafkaTrackReader_->GetKafkaAPI()->GetWatermarkOffsets(lowWatermarkOffset,highWaterMarkOffset);
  high_water_mark_offset_ -= 1;
	sending_thread_ = QtConcurrent::run(this, &ThroughTrackViewer::PullData);
	running_ = true;
  // }
  // else
  // {
  //   if (sending_thread_.isFinished())
	// {
  //   std::cout << "trying to connect to ZeroMQ" << std::endl;
	//   running_ = false;
  //
	//   inf::icomm::ReactorFactory reactor_factory;
  //   reactor_factory.SetConnectionUri(zeromq_connection_uri_);
	//   reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
	//   reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
	//   reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);
  //
	//   // bind a function pointer to a topic data type combination
  //   reactor_factory.AddHandler<ThroughTrackPacketAvroAdapter>(packet_name,
	//       boost::bind(&ThroughTrackViewer::HandleIncomingParticleTracks, this, _1), true );
  //
	//   // zmq_reactor_.reset();
	//   zmq_reactor_ = reactor_factory.MakeReactor();
  //
	//   running_ = true;
	//   sending_thread_ = QtConcurrent::run(this, &ThroughTrackViewer::PullData);
  //   std::cout << "ZeroMQ thread is running" << std::endl;
	// }
	// else
	// {
	//   running_ = false;
  //   std::cout << "Already connected To ZMQ, waiting to finish" << std::endl;
	//   sending_thread_.waitForFinished();
  //
	//   inf::icomm::ReactorFactory reactor_factory;
  //   reactor_factory.SetConnectionUri(zeromq_connection_uri_);
	//   reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
	//   reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
	//   reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);
  //
	//   // bind a function pointer to a topic data type combination
	//   reactor_factory.AddHandler<ThroughTrackPacketAvroAdapter>(packet_name,
	//       boost::bind(&ThroughTrackViewer::HandleIncomingParticleTracks, this, _1), true );
  //
	//   running_ = true;
	//   sending_thread_ = QtConcurrent::run(this, &ThroughTrackViewer::PullData);
	// }
  // }
}

/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::PullData()
{
	while (running_)
	{
    printf("trying to pull through track packet\n");
    AvroAdapter<ThroughTrackPacket> tt_pack_aa;     
    tt_pack_aa = pKafkaTrackReader_->ReadNextThroughTrackPacket();
    if(tt_pack_aa.GetData() != NULL)
    {
		  HandleIncomingParticleTracks(tt_pack_aa.GetData());
    }
    // zmq_reactor_->HandleEvents(100);
	}
}


/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::Disconnect()
{
  running_ = false;
  if(connected_to_kafka_)
  {
	  delete pKafkaTrackReader_;
    connected_to_kafka_ = false;
  }
	// if (zmq_reactor_ != NULL)
	// {
	//   running_ = false;
	//   if (!sending_thread_.isFinished())
	//   {
	//     sending_thread_.waitForFinished();
	//   }
  //
	//   zmq_reactor_.reset();
	// }
}

void ThroughTrackViewer::Clear()
{
	vtkSmartPointer<vtkAppendPolyData> inAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	incomingParticleTrajectoryOutput_ = inAppendPolyData;
	vtkSmartPointer<vtkAppendPolyData> outAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	outgoingParticleTrajectoryOutput_ = outAppendPolyData;
	vtkSmartPointer<vtkAppendPolyData> sphereAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	sphereOutput_ = sphereAppendPolyData;

	this->Internal->PolyData = incomingParticleTrajectoryOutput_->GetOutput();
	this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
void ThroughTrackViewer::HandleIncomingParticleTracks(
		ThroughTrackPacketSharedPtr through_track_packet)
{
	// assert(through_track_packet);
  printf("got particle track packet with %d tracks\n", through_track_packet->tracks.size());
	this->AppendToPolyData(through_track_packet);  // This appends the tracks to the buffers
  
  if(packet_count_ >= display_interval_)
  {
    printf("reached packet count, updating outputs\n");

    vtkSmartPointer<vtkAppendPolyData> inAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
    incomingParticleTrajectoryOutput_ = inAppendPolyData;
    vtkSmartPointer<vtkAppendPolyData> outAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
    outgoingParticleTrajectoryOutput_ = outAppendPolyData;
    
    vtkSmartPointer<vtkPolyData> inPolyData = vtkSmartPointer<vtkPolyData>::New();
    inPolyData->DeepCopy(incomingParticleTrajectoryBuffer_->GetOutput());
    incomingParticleTrajectoryOutput_->AddInputData(inPolyData);
    
    vtkSmartPointer<vtkPolyData> outPolyData = vtkSmartPointer<vtkPolyData>::New();
    outPolyData->DeepCopy(outgoingParticleTrajectoryBuffer_->GetOutput());
    outgoingParticleTrajectoryOutput_->AddInputData(outPolyData);
    
    incomingParticleTrajectoryOutput_->Update();
    outgoingParticleTrajectoryOutput_->Update();
    
    this->Internal->PolyData = incomingParticleTrajectoryOutput_->GetOutput();
    this->Internal->NewData = true;

    // now clear the buffers 
    vtkSmartPointer<vtkAppendPolyData> inTrajBuffer = vtkSmartPointer<vtkAppendPolyData>::New();
    incomingParticleTrajectoryBuffer_ = inTrajBuffer;
    vtkSmartPointer<vtkAppendPolyData> outTrajBuffer = vtkSmartPointer<vtkAppendPolyData>::New();
    outgoingParticleTrajectoryBuffer_ = outTrajBuffer;
    
    packet_count_ = 0;
  }
  packet_count_++;
}

void ThroughTrackViewer::AppendToPolyData(
		ThroughTrackPacketSharedPtr through_track_packet)
{
	std::vector<das::ThroughTrack>::const_iterator iter = through_track_packet->tracks.begin();

	for (; iter != through_track_packet->tracks.end(); ++iter)
	{
      if((*iter).scattering_angle < min_scattering_angle_cut_) continue;
      const das::Line3& incoming_line = (*iter).incoming_trajectory.trajectory;
      const das::Line3& outgoing_line = (*iter).outgoing_trajectory.trajectory;

      das::Vector3 incoming_tip_point = GetTip(incoming_line.point, incoming_line.direction, incoming_line.length);

      vtkSmartPointer<vtkLineSource> vtk_line_source_incoming = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source_incoming->SetPoint1(
          incoming_line.point.x_coordinate,
          incoming_line.point.y_coordinate,
          incoming_line.point.z_coordinate);

      double ztip = incoming_tip_point.z_coordinate;
      if(ztip > incoming_line.point.z_coordinate) ztip = incoming_line.point.z_coordinate;
      if(ztip < 0) ztip = 0;
      vtk_line_source_incoming->SetPoint2(
          incoming_tip_point.x_coordinate,
          incoming_tip_point.y_coordinate,
          // incoming_tip_point.z_coordinate);
          ztip);
      vtk_line_source_incoming->Update();

      vtkSmartPointer<vtkLineSource> vtk_line_source_outgoing = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source_outgoing->SetPoint1(
          outgoing_line.point.x_coordinate,
          outgoing_line.point.y_coordinate,
          outgoing_line.point.z_coordinate);

      das::Vector3 outgoing_tip_point = GetTip(outgoing_line.point, outgoing_line.direction, outgoing_line.length);
      
      ztip = outgoing_tip_point.z_coordinate;
      if(ztip > incoming_line.point.z_coordinate) ztip = incoming_line.point.z_coordinate;
      if(ztip < 0) ztip = 0;

      vtk_line_source_outgoing->SetPoint2(
          outgoing_tip_point.x_coordinate,
          outgoing_tip_point.y_coordinate,
          // outgoing_tip_point.z_coordinate);
          ztip);
      vtk_line_source_outgoing->Update();


		vtkSmartPointer<vtkDoubleArray> momentum = vtkSmartPointer<vtkDoubleArray>::New();
		momentum->SetName("Momentum");
		momentum->InsertNextValue((*iter).momentum);

		vtkSmartPointer<vtkDoubleArray> scatteringAngle = vtkSmartPointer<vtkDoubleArray>::New();
		scatteringAngle->SetName("Scattering Angle");
		scatteringAngle->InsertNextValue((*iter).scattering_angle);

		vtkSmartPointer<vtkDoubleArray> IncomingTrackLength = vtkSmartPointer<vtkDoubleArray>::New();
		IncomingTrackLength->SetName("Incoming Track Length");
		IncomingTrackLength->InsertNextValue(incoming_line.length);
      
    vtkSmartPointer<vtkDoubleArray> OutgoingTrackLength = vtkSmartPointer<vtkDoubleArray>::New();
    OutgoingTrackLength->SetName("Outgoing Track Length");
    OutgoingTrackLength->InsertNextValue(outgoing_line.length);

		vtkSmartPointer<vtkDoubleArray> DOCA = vtkSmartPointer<vtkDoubleArray>::New();
		DOCA->SetName("DOCA");
		DOCA->InsertNextValue((*iter).closest_approach_line.length);

		vtkSmartPointer<vtkDoubleArray> xcom = vtkSmartPointer<vtkDoubleArray>::New();
		xcom->SetName("PoCA X Coordinate");
		xcom->InsertNextValue((*iter).poca.x_coordinate);

		vtkSmartPointer<vtkDoubleArray> ycom = vtkSmartPointer<vtkDoubleArray>::New();
		ycom->SetName("PoCA Y Coordinate");
		ycom->InsertNextValue((*iter).poca.y_coordinate);

		vtkSmartPointer<vtkDoubleArray> zcom = vtkSmartPointer<vtkDoubleArray>::New();
		zcom->SetName("PoCA Z Coordinate");
		zcom->InsertNextValue((*iter).poca.z_coordinate);

    vtkSmartPointer<vtkPolyData> incomingTrajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
    incomingTrajectoryPolydata = vtk_line_source_incoming->GetOutput();
    incomingTrajectoryPolydata->GetCellData()->AddArray(momentum);
    incomingTrajectoryPolydata->GetCellData()->AddArray(scatteringAngle);
    incomingTrajectoryPolydata->GetCellData()->AddArray(IncomingTrackLength);
    incomingTrajectoryPolydata->GetCellData()->AddArray(OutgoingTrackLength);
    incomingTrajectoryPolydata->GetCellData()->AddArray(DOCA);
    incomingTrajectoryPolydata->GetCellData()->AddArray(xcom);
    incomingTrajectoryPolydata->GetCellData()->AddArray(ycom);
    incomingTrajectoryPolydata->GetCellData()->AddArray(zcom);

    vtkSmartPointer<vtkPolyData> outgoingTrajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
    outgoingTrajectoryPolydata = vtk_line_source_outgoing->GetOutput();
    outgoingTrajectoryPolydata->GetCellData()->AddArray(momentum);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(scatteringAngle);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(IncomingTrackLength);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(OutgoingTrackLength);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(DOCA);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(xcom);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(ycom);
    outgoingTrajectoryPolydata->GetCellData()->AddArray(zcom);

    incomingParticleTrajectoryBuffer_->AddInputData(incomingTrajectoryPolydata);
    outgoingParticleTrajectoryBuffer_->AddInputData(outgoingTrajectoryPolydata);
  }

    incomingParticleTrajectoryBuffer_->Update();
    outgoingParticleTrajectoryBuffer_->Update();

}

/*----------------------------------------------------------------------------*/
int ThroughTrackViewer::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
  vtkInformation* outInfoIncomingParticleTrack = outputVector->GetInformationObject(0);
  outInfoIncomingParticleTrack->Set(ThroughTrackViewer::DESCRIPTIVE_NAME(), "Incoming Particle Trajectory");
  vtkSmartPointer<vtkDataSet> incomingParticleTrackOutput = vtkDataSet::SafeDownCast(outInfoIncomingParticleTrack->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkInformation* outInfoOutgoingParticleTrack = outputVector->GetInformationObject(1);
  outInfoOutgoingParticleTrack->Set(ThroughTrackViewer::DESCRIPTIVE_NAME(), "Outgoing Particle Trajectory");
  vtkSmartPointer<vtkDataSet> outgoingParticleTrackOutput = vtkDataSet::SafeDownCast(outInfoOutgoingParticleTrack->Get(vtkDataObject::DATA_OBJECT()));

	if (!this->HasData())
	{
		return 1;
	}

  incomingParticleTrackOutput->ShallowCopy(incomingParticleTrajectoryOutput_->GetOutput());
  outgoingParticleTrackOutput->ShallowCopy(outgoingParticleTrajectoryOutput_->GetOutput());

	return 1;
}

void ThroughTrackViewer::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}

das::Vector3 ThroughTrackViewer::GetTip(
		const das::Vector3& point, const das::Vector3& direction, double length)
{
	das::Vector3 tip_point;
	tip_point.x_coordinate = (point.x_coordinate + direction.x_coordinate * length);
	tip_point.y_coordinate = (point.y_coordinate + direction.y_coordinate * length);
	tip_point.z_coordinate = (point.z_coordinate + direction.z_coordinate * length);

	return tip_point;
}
void ThroughTrackViewer::SetUpdateTimeInterval(int packets)
{
  display_interval_ = packets;  // 10 packets per second
  Clear();
}
void ThroughTrackViewer::SetMachineName(int name_enum)
{
  switch (name_enum)
  {
    case 1:
      machine_name_ = "HMT";
      kafka_broker_name_ = HMT_KAFKA_BROKER;
      if(filtered_)
        zeromq_connection_uri_ = HMT_FILTERED_ICOMM_CONNECTION_URI;
      else
        zeromq_connection_uri_ = HMT_ICOMM_CONNECTION_URI;
      break;
    case 2:
      machine_name_ = "EMT";
      kafka_broker_name_ = EMT_KAFKA_BROKER;
      if(filtered_)
        zeromq_connection_uri_ = EMT_FILTERED_ICOMM_CONNECTION_URI;
      else
        zeromq_connection_uri_ = EMT_ICOMM_CONNECTION_URI;
      break;
    case 3:
      machine_name_ = "SMT";
      kafka_broker_name_ = SMT_KAFKA_BROKER;
      printf("Setting system to WOOKIE\n");
      if(filtered_)
        zeromq_connection_uri_ = SMT_FILTERED_ICOMM_CONNECTION_URI;
      else
        zeromq_connection_uri_ = SMT_ICOMM_CONNECTION_URI;
      break;
    case 4:
      machine_name_ = "AMT";
      kafka_broker_name_ = AMT_KAFKA_BROKER;
      printf("Setting system to SoulCrusher!\n");
      if(filtered_)
        zeromq_connection_uri_ = AMT_FILTERED_ICOMM_CONNECTION_URI;
      else
        zeromq_connection_uri_ = AMT_ICOMM_CONNECTION_URI;
      break;
    case 5:
      machine_name_ = "NACHO";
      kafka_broker_name_ = NACHO_KAFKA_BROKER;
      printf("Setting system to Nacho!\n");
      if(filtered_)
        zeromq_connection_uri_ = NACHO_FILTERED_ICOMM_CONNECTION_URI;
      else
        zeromq_connection_uri_ = NACHO_ICOMM_CONNECTION_URI;
      break;
    default:
      printf("That machine name wasn't on my list!  Defaulting to HMT\n");
      machine_name_ = "HMT";
      zeromq_connection_uri_ = HMT_ICOMM_CONNECTION_URI;
      break;
  }
  if(running_ == true)
  {
    Disconnect();
    Clear();
    Connect();
  }
  // else
  //   Clear();
}
