#include "EventAvroViewer.h"

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

#include "EventPacket.hh"

#include "vtkTubeFilter.h"
#include "vtkCylinderSource.h"
#include "vtkBooleanOperationPolyDataFilter.h"

#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkPointSource.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QString>

#include <Eigen/Dense>

/*----------------------------------------------------------------------------*/
class EventAvroViewer::vtkInternal
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
vtkStandardNewMacro(EventAvroViewer);
vtkInformationKeyMacro(EventAvroViewer, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
EventAvroViewer::EventAvroViewer():
  num_faces_(20)
{

	this->Internal = new vtkInternal;

	this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);

  LoadGeometryFile();
  
}

/*----------------------------------------------------------------------------*/
EventAvroViewer::~EventAvroViewer()
{
	if (this->Internal != NULL)
	{
	  delete this->Internal;
	}
}

void EventAvroViewer::LoadGeometryFile()
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
}
void EventAvroViewer::DrawDriftTubes(std::vector<das::HitPulse> event)
{
    vtkSmartPointer<vtkAppendPolyData> driftTubeOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftTubeOutput_ = driftTubeOutputAppend;

    std::vector<das::HitPulse>::const_iterator hit_iter = event.begin();
    for (; hit_iter != event.end(); ++hit_iter)
    {
        uint32_t channel = (*hit_iter).global_channel_id;
        vtkSmartPointer<vtkLineSource> TubeSection = vtkSmartPointer<vtkLineSource>::New();
        TubeSection->SetPoint1(GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointX(),
                     GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointY(),
                     GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointZ());
        
        // std::cout << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointX() << ", "
        //           << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointY() << ", "
        //           << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetPointZ() << ", "
        //           << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetDirection().GetX() << ", "
        //           << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetDirection().GetY() << ", "
        //           << GeometryInfo_->GetDriftTube(channel)->GetExtents().GetDirection().GetZ() << ", "
        //           << (*hit_iter).drift << std::endl;

        TubeSection->SetPoint2(GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetX(),
                     GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetY(),
                     GeometryInfo_->GetDriftTube(channel)->GetExtents().GetTip().GetZ());
        TubeSection->Update();
        
        vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
        tube_filter->SetInputConnection(TubeSection->GetOutputPort());
        tube_filter->SetRadius(2.54);
        tube_filter->SetNumberOfSides(num_faces_);
        tube_filter->SetCapping(1);
        tube_filter->Update();


        vtkSmartPointer<vtkIntArray> TubeIndex = vtkSmartPointer<vtkIntArray>::New();
        TubeIndex->SetName("Tube Index");
        
        for (int cell = 0; cell < tube_filter->GetOutput()->GetNumberOfCells(); cell++)
        {
          TubeIndex->InsertNextTuple1((*hit_iter).global_channel_id);
        }
        tube_filter->GetOutput()->GetCellData()->AddArray(TubeIndex);
        
        tube_filter->Update();

        driftTubeOutput_->AddInputData(tube_filter->GetOutput());
       
    }
    driftTubeOutput_->Update();
    this->Modified();
}

/*----------------------------------------------------------------------------*/
bool EventAvroViewer::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void EventAvroViewer::UpdatePipeline()
{
	if (this->HasData())
	{
		this->Modified();
	}
}

/*----------------------------------------------------------------------------*/
void EventAvroViewer::OpenAvro()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0), 
                          QObject::tr("Open EventPacket Avro File "),
                          "/home/sossong/data/",
                          QObject::tr("EventPacket Files (*.avro)"));

  if (fileName.toStdString().length() > 1)
  {
    ttep_reader_.AppendFilePath(fileName.toStdString());
  }


}
/*----------------------------------------------------------------------------*/
void EventAvroViewer::PullNextEvent()
{
    if(current_event_packet_ == NULL || // first time
        current_event_indices_ == current_event_packet_->event_indices.end()
        ) // past the end of the packet
    {
      current_event_packet_ = ttep_reader_.Read();
      current_event_indices_ = current_event_packet_->event_indices.begin();
    }
    std::vector<das::HitPulse> event;
    for(int hit_index = (*current_event_indices_).start_index; hit_index <= (*current_event_indices_).stop_index; hit_index++)
    {
      event.push_back(current_event_packet_->pulses.at(hit_index));
    }
	  this->AppendToPolyData(event);
    ++current_event_indices_;  // increment the iter
}
/*----------------------------------------------------------------------------*/
void EventAvroViewer::Clear()
{
    vtkSmartPointer<vtkAppendPolyData> driftTubeOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftTubeOutput_ = driftTubeOutputAppend;


	this->Internal->PolyData = driftTubeOutput_->GetOutput();
	this->Internal->NewData = true;
}

void EventAvroViewer::AppendToPolyData(
		std::vector<das::HitPulse> event)
{
  if (GeometryInfo_)
  {
    // only do the first track for now.  Later change it to a loop over the desired number of tracks
    // for (; iter != through_track_event_packet->tracks.end(); ++iter)
    // {
      DrawDriftTubes(event);

    // this->Internal->PolyData = particleTrajectoryOutput_->GetOutput();
    this->Internal->NewData = true;
    //
    // vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
    // particleTrajectoryOutput_ = append;
  }
}

/*----------------------------------------------------------------------------*/
int EventAvroViewer::RequestData(vtkInformation *request,
    vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  vtkInformation* outInfoDriftTubes = outputVector->GetInformationObject(0);
  outInfoDriftTubes->Set(EventAvroViewer::DESCRIPTIVE_NAME(), "Drift Tubes");
  vtkSmartPointer<vtkDataSet> driftTubeOutput = vtkDataSet::SafeDownCast(outInfoDriftTubes->Get(vtkDataObject::DATA_OBJECT()));
  
  if (!this->HasData() || driftTubeOutput_ == NULL)
  {
    return 1;
  }

  driftTubeOutput->ShallowCopy(driftTubeOutput_->GetOutput());

  return 1;
}

void EventAvroViewer::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}

das::Vector3 EventAvroViewer::GetTip(const das::Vector3& point,
    const das::Vector3& direction, double length)
{
	das::Vector3 tip_point;
	tip_point.x_coordinate = (point.x_coordinate + direction.x_coordinate * length);
	tip_point.y_coordinate = (point.y_coordinate + direction.y_coordinate * length);
	tip_point.z_coordinate = (point.z_coordinate + direction.z_coordinate * length);

	return tip_point;
}
