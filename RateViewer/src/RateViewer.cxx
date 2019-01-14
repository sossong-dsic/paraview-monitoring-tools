#include "RateViewer.h"

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

#include "HitPulsePacket.hh"

#include "vtkTubeFilter.h"
#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkPointSource.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QString>

const std::string HMT_ICOMM_CONNECTION_URI = "tcp://compute1.fmt.red.dsic.com:10002";
const std::string EMT_ICOMM_CONNECTION_URI = "tcp://compute3.fmt.red.dsic.com:10002";
const std::string SMT_ICOMM_CONNECTION_URI = "tcp://10.200.10.20:10002";
const std::string AMT_ICOMM_CONNECTION_URI = "tcp://10.0.221.20:10002";
/*----------------------------------------------------------------------------*/
class RateViewer::vtkInternal
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
vtkStandardNewMacro(RateViewer);
vtkInformationKeyMacro(RateViewer, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
RateViewer::RateViewer():
  current_comp_(0),
  num_channels_(0),
  num_components_(100),
  num_faces_(20),
  max_packet_rate_(5),
  subtract_background_(false),
  machine_name_("AMT")
{
  running_ = false;

  SetMachineName(4); // 1 = HMT, 2 = EMT, 3 = SMT

  this->Internal = new vtkInternal;

  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);

  LoadGeometryFile();

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
RateViewer::~RateViewer()
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

void RateViewer::LoadGeometryFile()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0),
                          QObject::tr("Open Geometry"),
                          "/home/export/",
                          QObject::tr("Geometry Files (*.bin)"));

  if (fileName.toStdString().length() > 1)
  {
    mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fileName.toStdString());
    GeometryInfo_ = GeometryInfo;
  }

  CreateDriftTubes();
}

void RateViewer::CreateDriftTubes()
{
  if(GeometryInfo_ == NULL)
  {
    printf("%s::No geometry defined, unable to create drift tubes\n", __PRETTY_FUNCTION__);
    return;
  }
  num_channels_ = GeometryInfo_->NumDriftTubes();

  InitializeHitCountArrays();
  
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
  vtkSmartPointer<vtkDoubleArray> rate_vtk_array = vtkSmartPointer<vtkDoubleArray>::New();
  rate_vtk_array->SetName("Hit Rate");
  rate_vtk_array->SetNumberOfTuples(num_cells);
  rate_vtk_array->vtkAbstractArray::SetNumberOfComponents(1);
  vtk_append_poly_data_->GetOutput()->GetCellData()->AddArray(rate_vtk_array);
  display_rate_array_ = rate_vtk_array->WritePointer(0, num_cells);

  // initialize the display_rate_array_
  for(int tube = 0; tube<num_channels_; tube++)
  {
    for(int tcell=0; tcell<num_faces_; tcell++)
    {
      int gcell = tube*num_faces_ + tcell;
      display_rate_array_[gcell] = max_packet_rate_ * (double)tube / num_channels_;
    }
  }
  vtk_append_poly_data_->Update();

  printf("created the drift tubes, updating polydata\n");

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
bool RateViewer::HasData()
{
  return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void RateViewer::UpdatePipeline()
{
  // printf("%s: Updating Renderer\n", __PRETTY_FUNCTION__);
  if (this->HasData())
  {
    this->Modified();
    this->Update();
  }
}

/*----------------------------------------------------------------------------*/
void RateViewer::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void RateViewer::Connect()
{
  if (sending_thread_.isFinished())
  {
    std::cout << "trying to connect to ZeroMQ" << std::endl;
    running_ = false;

    inf::icomm::ReactorFactory reactor_factory;
    reactor_factory.SetConnectionUri(zeromq_connection_uri_);
    reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
    reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
    reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);

    // bind a function pointer to a topic data type combination
    reactor_factory.AddHandler<HitPulsePacketAvroAdapter>("HitPulsePacket",
        boost::bind(&RateViewer::HandleIncomingHitPulsePacket, this, _1), true );

    zmq_reactor_.reset();
    zmq_reactor_ = reactor_factory.MakeReactor();

    running_ = true;
    sending_thread_ = QtConcurrent::run(this, &RateViewer::PullData);
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
    reactor_factory.AddHandler<HitPulsePacketAvroAdapter>("HitPulsePacket",
        boost::bind(&RateViewer::HandleIncomingHitPulsePacket, this, _1), true );

    running_ = true;
    sending_thread_ = QtConcurrent::run(this, &RateViewer::PullData);
  }
}

void RateViewer::InitializeHitCountArrays()

{
  // Initialize the double array to store the last 10 packet counts
  std::cout << "number of channels is " << num_channels_ << std::endl;
  rate_comp_array_.clear();
  rate_array_.clear();
  bkgd_rate_array_.clear();
  for(int chan=0; chan<num_channels_; chan++)
  {
    std::vector<double> comps;
    for(int j=0; j<num_components_; j++)
    {
      comps.push_back(0.0);
    }
    rate_comp_array_.push_back(comps);
    bkgd_rate_array_.push_back(0.0);
    rate_array_.push_back(0.0);
  }
}

/*----------------------------------------------------------------------------*/
void RateViewer::PullData()
{
  while (running_)
  {
    zmq_reactor_->HandleEvents(100);
  }
}


/*----------------------------------------------------------------------------*/
void RateViewer::Disconnect()
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

void RateViewer::Clear()
{
  vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
  vtk_append_poly_data_ = appendPolyData;

  CreateDriftTubes();
  current_comp_ = 0;
}

/*----------------------------------------------------------------------------*/
void RateViewer::HandleIncomingHitPulsePacket(
    HitPulsePacketSharedPtr hit_pulse_packet)
{
  assert(hit_pulse_packet);

  std::vector<das::HitPulse>::const_iterator iter = hit_pulse_packet->hit_pulses.begin();

  // int channel_index = 0;
  for (; iter != hit_pulse_packet->hit_pulses.end(); ++iter)
  {
      // std::cout << "got hit_pulse_packet on channel " << (*iter).global_channel_id << std::endl;
      rate_comp_array_[(*iter).global_channel_id][current_comp_] += 1; 
  }

  current_comp_++;
  if(current_comp_ >= num_components_)
  {
    std::cout << "summing " << num_components_ << " of hit_pulse_packets with time: " << hit_pulse_packet->header.start_time_seconds << std::endl;
    UpdateRateArray();
    UpdatePolyData();
    current_comp_ = 0;
  }
}

void RateViewer::UpdateRateArray()
{
  for(int chan = 0; chan<num_channels_; chan++)
  {
    double sum = 0;
    for(int comp=0; comp<num_components_; comp++)
    {
      sum += rate_comp_array_[chan][comp];
      // reset the rate_comp_array to zero once we have it added
      rate_comp_array_[chan][comp] = 0;
    }
    rate_array_[chan] = (double)sum / (double)num_components_ / GeometryInfo_->GetDriftTube(chan)->GetExtents().GetLength() * 100.0; // rate per meter, length is in cm
  }
}
void RateViewer::UpdatePolyData()
{
    std::cout << "Updating PolyData Output..." << std::endl;
  for(int chan = 0; chan<num_channels_; chan++)
  {
    for(int cell=0; cell<(num_faces_); cell++)
    {
      if(subtract_background_)
        display_rate_array_[chan * num_faces_ + cell] = rate_array_[chan] - bkgd_rate_array_[chan];
      else
        display_rate_array_[chan * num_faces_ + cell] = rate_array_[chan];
    }
  }

  vtk_append_poly_data_->Update();

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
int RateViewer::RequestData(vtkInformation *vtkNotUsed(request),
    vtkInformationVector **vtkNotUsed(inputVector),
    vtkInformationVector *outputVector)
{
  // printf("%s::Called, getting information object\n",__PRETTY_FUNCTION__);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(RateViewer::DESCRIPTIVE_NAME(), "Drift Tubes");
  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  if (!this->HasData())
  {
    return 1;
  }

  // printf("%s::Had Data, copying latest polydata to output\n",__PRETTY_FUNCTION__);
  output->ShallowCopy(this->Internal->GetLatestPolyData());
  // printf("%s::HasData is now %s\n",__PRETTY_FUNCTION__, (this->HasData() ? "true" : "false"));

  return 1;
}

void RateViewer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

void RateViewer::LoadRateBackgroundFile()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0), QObject::tr("Open Rate Background"), "/home/export/", QObject::tr("Rate Background Files (*.rbg)"));
  QFile file( fileName );
  if(!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(0,"error",file.errorString());
}
  QTextStream in(&file);
  int chan = 0;
  while(!in.atEnd())
  {
     QString line = file.readLine();
     bkgd_rate_array_[chan++] = line.toDouble();
  }  
  file.close();
}
void RateViewer::WriteRateBackgroundFile()
{
  std::cout << "Opening file dialog to save the Background" << std::endl;
  QString fileName = QFileDialog::getSaveFileName(QWidget::find(0), QObject::tr("Save Background"), "/home/export/", QObject::tr("Background Files (*.rbg)"));
  QFileInfo fi( fileName );
	if (fi.suffix().isEmpty())
	{
		fileName += ".rbg";
	}
  QFile file( fileName );
  QTextStream out(&file);
  if(!file.open(QIODevice::WriteOnly)) {
    QMessageBox::information(0,"error",file.errorString());
}
  for(int chan=0; chan<num_channels_; chan++)
  {
    out << bkgd_rate_array_[chan] << endl;
  }  
  file.close();
}
void RateViewer::BuildRateBackground()
{
  std::cout << "Building background array" << std::endl;
//  for now, just pull in the current rate_array_, 
//  reduced to num_channels_ from num_channels_ * num_faces_ * 2
    
  for(int chan=0; chan<num_channels_; chan++)
  {
    bkgd_rate_array_[chan] = rate_array_[chan];
  }

}
void RateViewer::SetBackgroundSubtractOn()
{
    subtract_background_ = true;
}
void RateViewer::SetBackgroundSubtractOff()
{
    subtract_background_ = false;
}
void RateViewer::ToggleBackgroundSubtract(bool checked)
{
  if(checked) SetBackgroundSubtractOn();
  else SetBackgroundSubtractOff();
}
void RateViewer::SetUpdateTimeInterval(int seconds)
{
  num_components_ = seconds * 10;  // 10 packets per second
  Clear();
}
void RateViewer::SetMachineName(int name_enum)
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
    case 4:
      machine_name_ = "AMT";
      zeromq_connection_uri_ = AMT_ICOMM_CONNECTION_URI;
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
  else
  {
    // Clear();
  }
}
