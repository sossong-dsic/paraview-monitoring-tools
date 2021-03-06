#include "OutlierTubes.h"

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
#include "vtkTubeFilter.h"

#include "ThroughTrackEventPacket.hh"

#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkPointSource.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QString>

const std::string HMT_ICOMM_CONNECTION_URI = "tcp://compute2.fmt.red.dsic.com:10005";
const std::string EMT_ICOMM_CONNECTION_URI = "tcp://compute4.fmt.red.dsic.com:10005";
const std::string SMT_ICOMM_CONNECTION_URI = "tcp://compute2.cttso.dsic.com:10005";
/*----------------------------------------------------------------------------*/
class OutlierTubes::vtkInternal
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
vtkStandardNewMacro(OutlierTubes);
vtkInformationKeyMacro(OutlierTubes, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
OutlierTubes::OutlierTubes():
  current_comp_(0),
  num_channels_(0),
  num_components_(100),
  num_faces_(20),
  max_packet_rate_(1),
  machine_name_("HMT"),
  normalize_by_ntracks_(true)
{
	running_ = false;

  SetMachineName(1); // 1 = HMT, 2 = EMT, 3 = SMT

	this->Internal = new vtkInternal;

	this->SetNumberOfInputPorts(0);
	this->SetNumberOfOutputPorts(1);

  LoadGeometryFile();

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
OutlierTubes::~OutlierTubes()
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

void OutlierTubes::LoadGeometryFile()
{
	QString fileName = QFileDialog::getOpenFileName(QWidget::find(0),
													QObject::tr("Open Geometry"),
                          "/opt/dsc/config/scanner_info/",
													QObject::tr("Geometry Files (*.bin)"));

	if (fileName.toStdString().length() > 1)
	{
		mtlib::GeometryInfoSharedPtr GeometryInfo(mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fileName.toStdString()));
		GeometryInfo_ = GeometryInfo;
  }

  CreateDriftTubes();
}

void OutlierTubes::CreateDriftTubes()
{
  if(GeometryInfo_ == NULL)
  {
    printf("%s::No geometry defined, unable to create drift tubes\n", __PRETTY_FUNCTION__);
    return;
  }
  num_channels_ = GeometryInfo_->NumDriftTubes();

  InitializeOutlierCountArrays();
  
  vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
  vtk_append_poly_data_ = appendPolyData;


  for (int channel = 0; channel < num_channels_; channel++)
		{
			vtkSmartPointer<vtkLineSource> TubeSection = vtkSmartPointer<vtkLineSource>::New();
    TubeSection->SetPoint1(GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointX(),
                 GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointY(),
                 GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointZ());
    TubeSection->SetPoint2(GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetX(),
                 GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetY(),
                 GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetZ());
			TubeSection->Update();

			vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
			tube_filter->SetInputConnection(TubeSection->GetOutputPort());
			tube_filter->SetRadius(2.54);
    tube_filter->SetNumberOfSides(num_faces_);
			tube_filter->Update();

    vtk_append_poly_data_->AddInputData(tube_filter->GetOutput());
  }
  vtk_append_poly_data_->Update();

  int num_cells = vtk_append_poly_data_->GetOutput()->GetNumberOfCells();
  vtkSmartPointer<vtkDoubleArray> outlier_rate_vtk_array = vtkSmartPointer<vtkDoubleArray>::New();
  outlier_rate_vtk_array->SetName("Outlier Rate");
  outlier_rate_vtk_array->SetNumberOfTuples(num_cells);
  outlier_rate_vtk_array->vtkAbstractArray::SetNumberOfComponents(1);
  vtk_append_poly_data_->GetOutput()->GetCellData()->AddArray(outlier_rate_vtk_array);
  display_outlier_rate_array_ = outlier_rate_vtk_array->WritePointer(0, num_cells);

  // initialize the display_outlier_rate_array_
  for(int tube = 0; tube<num_channels_; tube++)
	{
	  for(int tcell = 0; tcell < num_faces_; tcell++)
		{
		  int gcell = tube * num_faces_ + tcell;
		  display_outlier_rate_array_[gcell] = max_packet_rate_ * (double)tube / num_channels_;
		}
	}
  vtk_append_poly_data_->Update();

  printf("created the drift tubes, updating polydata\n");

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
bool OutlierTubes::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void OutlierTubes::UpdatePipeline()
{
	if (this->HasData())
	{
		this->Modified();
    this->Update();
	}
}

/*----------------------------------------------------------------------------*/
void OutlierTubes::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void OutlierTubes::Connect()
{
	if (sending_thread_.isFinished())
	{
		running_ = false;

		inf::icomm::ReactorFactory reactor_factory;
    reactor_factory.SetConnectionUri(zeromq_connection_uri_);
		reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
		reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
		reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);

		// bind a function pointer to a topic data type combination
		reactor_factory.AddHandler<ThroughTrackEventPacketAvroAdapter>("ThroughTrackEventPacket",
				boost::bind(&OutlierTubes::HandleIncomingParticleTrackEvent, this, _1), true );

		zmq_reactor_.reset();
		zmq_reactor_ = reactor_factory.MakeReactor();

		running_ = true;
		sending_thread_ = QtConcurrent::run(this, &OutlierTubes::PullData);
	}
	else
	{
		running_ = false;
    std::cout << "Already connected To ZMQ, waiting to finish" << std::endl;
		sending_thread_.waitForFinished();

		inf::icomm::ReactorFactory reactor_factory;
    reactor_factory.SetConnectionUri(zeromq_connection_uri_);
		reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
		reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
		reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);

		// bind a function pointer to a topic data type combination
		reactor_factory.AddHandler<ThroughTrackEventPacketAvroAdapter>("ThroughTrackEventPacket",
				boost::bind(&OutlierTubes::HandleIncomingParticleTrackEvent, this, _1), true );

		running_ = true;
		sending_thread_ = QtConcurrent::run(this, &OutlierTubes::PullData);
	}
}

void OutlierTubes::InitializeOutlierCountArrays()

{
  // Initialize the double array to store the last 10 packet counts
  std::cout << "number of channels is " << num_channels_ << std::endl;
  outlier_rate_comp_array_.clear();
  outlier_rate_array_.clear();
  for(int chan=0; chan<num_channels_; chan++)
  {
    std::vector<double> comps;
    for(int j=0; j<num_components_; j++)
    {
      comps.push_back(0.0);
    }
    outlier_rate_comp_array_.push_back(comps);
    outlier_rate_array_.push_back(0.0);
  }
}


/*----------------------------------------------------------------------------*/
void OutlierTubes::PullData()
{
	while (running_)
	{
		zmq_reactor_->HandleEvents(100);
	}
}


/*----------------------------------------------------------------------------*/
void OutlierTubes::Disconnect()
{
	if (zmq_reactor_ != NULL)
	{
		running_ = false;
		if (!sending_thread_.isFinished())
		{
			sending_thread_.waitForFinished();
		}

    zmq_reactor_.reset();
	}
}

void OutlierTubes::Clear()
{
  vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
  vtk_append_poly_data_ = appendPolyData;

  CreateDriftTubes();
  current_comp_ = 0;
}

/*----------------------------------------------------------------------------*/
void OutlierTubes::HandleIncomingParticleTrackEvent(
		ThroughTrackEventPacketSharedPtr through_track_event_packet)
{
	assert(through_track_event_packet);
	std::vector<das::ThroughTrackEvent>::const_iterator iter = through_track_event_packet->tracks.begin();
  std::cout << "Got ThroughTrackEventPacket: " << current_comp_ << ".  Updating at " << num_components_ << std::endl;
  int channel_index = 0;
  // set up the number of tracks per tube array
  std::vector<uint32_t> number_of_tracks_through_tube(num_channels_, 0);
  
	for (; iter != through_track_event_packet->tracks.end(); ++iter)
	{
		std::vector<das::TrackHit>::const_iterator hit_iter = (*iter).hits.begin();
		for (; hit_iter != (*iter).hits.end(); ++hit_iter)
		{
      number_of_tracks_through_tube[(*hit_iter).index] +=1;
			if ((*hit_iter).is_outlier)
			{
        outlier_rate_comp_array_[(*hit_iter).index][current_comp_] += 1;
			}
		}
	}
  // now divide the components by the number of tracks to normalize for track numbers
  if(normalize_by_ntracks_)
  {
      for(int chan=0; chan<num_channels_; chan++)
      {
        for(int j=0; j<num_components_; j++)
        {
           if(number_of_tracks_through_tube[chan] != 0)
           {
             outlier_rate_comp_array_[chan][current_comp_] /= number_of_tracks_through_tube[chan];
           }
        }
      }
  }

  current_comp_++;
  if(current_comp_ >= num_components_)
  {
    std::cout << "summing " << num_components_ << " of through_track_event_packets with time: " << through_track_event_packet->header.start_time_seconds << std::endl;
    UpdateOutlierRateArray();
    UpdatePolyData();
    current_comp_ = 0;
    InitializeOutlierCountArrays();
  }
}

void OutlierTubes::UpdateOutlierRateArray()
{
  for(int chan = 0; chan<num_channels_; chan++)
  {
    double outlier_sum = 0;
    for(int comp=0; comp<num_components_; comp++)
    {
      outlier_sum += outlier_rate_comp_array_[chan][comp];
    }
    outlier_rate_array_[chan] = (double)outlier_sum / (double)num_components_ / GeometryInfo_->GetDriftTube(chan)->GetExtents().GetLength() * 100.0;
  }
}
void OutlierTubes::UpdatePolyData()
{
    std::cout << "Updating PolyData Output..." << std::endl;
  for(int chan = 0; chan<num_channels_; chan++)
  {
    for(int cell=0; cell<(num_faces_); cell++)
    {
      display_outlier_rate_array_[chan * num_faces_ + cell] = outlier_rate_array_[chan];
    }
  }

  vtk_append_poly_data_->Update();

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
int OutlierTubes::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation* outInfo = outputVector->GetInformationObject(0);
	outInfo->Set(OutlierTubes::DESCRIPTIVE_NAME(), "Outlier Tubes");
	vtkSmartPointer<vtkDataSet> output0 = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

	vtkSmartPointer<vtkPolyData> polyData = vtk_append_poly_data_->GetOutput();

	if (!this->HasData())
	{
		return 1;
	}

	output0->ShallowCopy(polyData);

	return 1;
}

void OutlierTubes::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
void OutlierTubes::SetUpdateTimeInterval(int seconds)
{
  num_components_ = seconds * 10;  // 10 packets per second
  Clear();
}
void OutlierTubes::SetMachineName(int name_enum)
{
  switch (name_enum)
  {
    case 1:
      machine_name_ = "HMT";
      zeromq_connection_uri_ = HMT_ICOMM_CONNECTION_URI;
      break;
    case 2:
      machine_name_ = "EMT";
      zeromq_connection_uri_ = EMT_ICOMM_CONNECTION_URI;
      break;
    case 3:
      machine_name_ = "SMT";
      zeromq_connection_uri_ = SMT_ICOMM_CONNECTION_URI;
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
}
