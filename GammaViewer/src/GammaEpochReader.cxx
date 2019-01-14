#include "GammaEpochReader.h"

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

#include "RawGammaDataPacket.hh"

#include "vtkTubeFilter.h"
#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkPointSource.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QString>

const std::string HMT_ICOMM_CONNECTION_URI = "tcp://compute1.fmt.red.dsic.com:10003";
const std::string EMT_ICOMM_CONNECTION_URI = "tcp://compute3.fmt.red.dsic.com:10003";
const std::string SMT_ICOMM_CONNECTION_URI = "tcp://10.200.10.20:10003";
const std::string AMT_ICOMM_CONNECTION_URI = "tcp://10.0.221.20:10003";
/*----------------------------------------------------------------------------*/
class GammaEpochReader::vtkInternal
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
vtkStandardNewMacro(GammaEpochReader);
vtkInformationKeyMacro(GammaEpochReader, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
GammaEpochReader::GammaEpochReader():
  current_comp_(0),
  num_channels_(0),
  num_components_(100),
  num_faces_(20),
  max_packet_rate_(1),
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
GammaEpochReader::~GammaEpochReader()
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

void GammaEpochReader::LoadGeometryFile()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0),
                          QObject::tr("Open Geometry"),
                          "/opt/dsc/config/scanner_info/",
                          QObject::tr("Geometry Files (*.bin)"));

  if (fileName.toStdString().length() > 1)
  {
    mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fileName.toStdString());
    GeometryInfo_ = GeometryInfo;
  }

  CreateDriftTubes();
}

void GammaEpochReader::CreateDriftTubes()
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
  vtkSmartPointer<vtkDoubleArray> gamma_rate_vtk_array = vtkSmartPointer<vtkDoubleArray>::New();
  gamma_rate_vtk_array->SetName("Gamma Rate");
  gamma_rate_vtk_array->SetNumberOfTuples(num_cells);
  gamma_rate_vtk_array->vtkAbstractArray::SetNumberOfComponents(1);
  vtk_append_poly_data_->GetOutput()->GetCellData()->AddArray(gamma_rate_vtk_array);
  display_gamma_rate_array_ = gamma_rate_vtk_array->WritePointer(0, num_cells);

  // initialize the display_gamma_rate_array_
  for(int tube = 0; tube<num_channels_; tube++)
  {
    for(int tcell = 0; tcell < num_faces_; tcell++)
    {
      int gcell = tube*num_faces_ + tcell;
      display_gamma_rate_array_[gcell] = max_packet_rate_ * (double)tube / num_channels_;
    }
  }
  vtk_append_poly_data_->Update();

  printf("created the drift tubes, updating polydata\n");

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
bool GammaEpochReader::HasData()
{
  return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void GammaEpochReader::UpdatePipeline()
{
  // printf("%s: Updating Renderer\n", __PRETTY_FUNCTION__);
  if (this->HasData())
  {
    this->Modified();
    this->Update();
  }
}
// int GammaEpochReader::RequestInformation(vtkInformation*,
//     vtkInformationVector**,
//     vtkInformationVector* outputVector)
// {
//   vtkInformation* outInfo = outputVector->GetInformationObject(0);
//
//
//   [> Set the Information, Array Type, and Number of Components to comprise the Data Object <]
//   vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 1);
//
//   [> Make an output port capable of producing an arbitrary subextent of the whole extent <]
//   outInfo->Set(CAN_PRODUCE_SUB_EXTENT(), 1);
//
//   return 1;
// }

/*----------------------------------------------------------------------------*/
void GammaEpochReader::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void GammaEpochReader::Connect()
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
    reactor_factory.AddHandler<RawGammaDataPacketAvroAdapter>("GammaEventPacket",
        boost::bind(&GammaEpochReader::HandleIncomingRawGammaDataPacket, this, _1), true );

    zmq_reactor_.reset();
    zmq_reactor_ = reactor_factory.MakeReactor();

    running_ = true;
    sending_thread_ = QtConcurrent::run(this, &GammaEpochReader::PullData);
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
    reactor_factory.AddHandler<RawGammaDataPacketAvroAdapter>("GammaEventPacket",
        boost::bind(&GammaEpochReader::HandleIncomingRawGammaDataPacket, this, _1), true );

    running_ = true;
    sending_thread_ = QtConcurrent::run(this, &GammaEpochReader::PullData);
  }
}

void GammaEpochReader::InitializeHitCountArrays()

{
  // Initialize the double array to store the last 10 packet counts
  std::cout << "number of channels is " << num_channels_ << std::endl;
  gamma_rate_comp_array_.clear();
  gamma_rate_array_.clear();
  bkgd_rate_array_.clear();
  for(int chan=0; chan<num_channels_; chan++)
  {
    std::vector<double> comps;
    for(int j=0; j<num_components_; j++)
    {
      comps.push_back(0.0);
    }
    gamma_rate_comp_array_.push_back(comps);
    bkgd_rate_array_.push_back(0.0);
    gamma_rate_array_.push_back(0.0);
  }
}

/*----------------------------------------------------------------------------*/
void GammaEpochReader::PullData()
{
  while (running_)
  {
    zmq_reactor_->HandleEvents(100);
  }
}


/*----------------------------------------------------------------------------*/
void GammaEpochReader::Disconnect()
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

void GammaEpochReader::Clear()
{
  vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
  vtk_append_poly_data_ = appendPolyData;

  CreateDriftTubes();
  current_comp_ = 0;
}

/*----------------------------------------------------------------------------*/
void GammaEpochReader::HandleIncomingRawGammaDataPacket(
    RawGammaDataPacketSharedPtr raw_gamma_data_packet)
{
  assert(raw_gamma_data_packet);
  std::vector<das::RawGammaData>::const_iterator iter = raw_gamma_data_packet->raw_gamma_hits.begin();
  std::cout << "Got RawGammaDataPacket: " << current_comp_ << ".  Updating at " << num_components_ << std::endl;
  int channel_index = 0;
  for (; iter != raw_gamma_data_packet->raw_gamma_hits.end(); ++iter)
  {
    // if((*iter).raw_gamma_hits<max_packet_rate_)
      gamma_rate_comp_array_[channel_index][current_comp_] = (*iter).raw_gamma_hits;
    // else
    //   gamma_rate_comp_array_[channel_index][current_comp_] = max_packet_rate_;
    channel_index++;
  }

  current_comp_++;
  if(current_comp_ >= num_components_)
  {
    std::cout << "summing " << num_components_ << " of raw_gamma_data_packets with time: " << raw_gamma_data_packet->header.start_time_seconds << std::endl;
    UpdateGammaRateArray();
    UpdatePolyData();
    current_comp_ = 0;
  }
}

void GammaEpochReader::UpdateGammaRateArray()
{
  for(int chan = 0; chan<num_channels_; chan++)
  {
    double gamma_sum = 0;
    for(int comp=0; comp<num_components_; comp++)
    {
      gamma_sum += gamma_rate_comp_array_[chan][comp];
      // reset the rate_comp_array to zero once we have it added
      gamma_rate_comp_array_[chan][comp] = 0;
    }
    gamma_rate_array_[chan] = (double)gamma_sum / (double)num_components_ / GeometryInfo_->GetDriftTube(chan)->GetExtents().GetLength() * 100.0; // rate per meter, length is in cm
  }
}
void GammaEpochReader::UpdatePolyData()
{
    std::cout << "Updating PolyData Output..." << std::endl;
  for(int chan = 0; chan<num_channels_; chan++)
  {
    for(int cell=0; cell<(num_faces_); cell++)
    {
      if(subtract_background_)
        display_gamma_rate_array_[chan * num_faces_ + cell] = gamma_rate_array_[chan] - bkgd_rate_array_[chan];
      else
        display_gamma_rate_array_[chan * num_faces_ + cell] = gamma_rate_array_[chan];
    }
  }

  vtk_append_poly_data_->Update();

  this->Internal->PolyData = vtk_append_poly_data_->GetOutput();
  this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
int GammaEpochReader::RequestData(vtkInformation *vtkNotUsed(request),
    vtkInformationVector **vtkNotUsed(inputVector),
    vtkInformationVector *outputVector)
{
  // printf("%s::Called, getting information object\n",__PRETTY_FUNCTION__);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(GammaEpochReader::DESCRIPTIVE_NAME(), "Drift Tubes");
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

void GammaEpochReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

void GammaEpochReader::LoadGammaBackgroundFile()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0), QObject::tr("Open Gamma Background"), "/home/export/", QObject::tr("Gamma Background Files (*.gbg)"));
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
void GammaEpochReader::WriteGammaBackgroundFile()
{
  std::cout << "Opening file dialog to save the Gamma Background" << std::endl;
  QString fileName = QFileDialog::getSaveFileName(QWidget::find(0), QObject::tr("Save Gamma Background"), "/home/export/", QObject::tr("Gamma Background Files (*.gbg)"));
  QFileInfo fi( fileName );
	if (fi.suffix().isEmpty())
	{
		fileName += ".gbg";
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
void GammaEpochReader::BuildGammaBackground()
{
  std::cout << "Building gamma background array" << std::endl;
//  for now, just pull in the current gamma_rate_array_, 
//  reduced to num_channels_ from num_channels_ * num_faces_ * 2
    
  for(int chan=0; chan<num_channels_; chan++)
  {
    bkgd_rate_array_[chan] = gamma_rate_array_[chan];
  }

}
void GammaEpochReader::SetBackgroundSubtractOn()
{
    subtract_background_ = true;
}
void GammaEpochReader::SetBackgroundSubtractOff()
{
    subtract_background_ = false;
}
void GammaEpochReader::ToggleBackgroundSubtract(bool checked)
{
  if(checked) SetBackgroundSubtractOn();
  else SetBackgroundSubtractOff();
}
void GammaEpochReader::SetUpdateTimeInterval(int seconds)
{
  num_components_ = seconds * 10;  // 10 packets per second
  Clear();
}
void GammaEpochReader::SetMachineName(int name_enum)
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
