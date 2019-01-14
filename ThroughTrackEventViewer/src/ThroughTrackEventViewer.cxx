#include "ThroughTrackEventViewer.h"

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
const std::string HMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute2.fmt.red.dsic.com:10007";
const std::string EMT_ICOMM_CONNECTION_URI = "tcp://compute4.fmt.red.dsic.com:10005";
const std::string EMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://compute4.fmt.red.dsic.com:10007";
const std::string SMT_ICOMM_CONNECTION_URI = "tcp://10.200.10.21:10005";
const std::string SMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://10.200.10.21:10007";
// const std::string AMT_ICOMM_CONNECTION_URI = "tcp://10.0.221.21:10005";
// const std::string AMT_FILTERED_ICOMM_CONNECTION_URI = "tcp://10.0.221.21:10007";
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

#include <Eigen/Dense>

/*----------------------------------------------------------------------------*/
class ThroughTrackEventViewer::vtkInternal
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
vtkStandardNewMacro(ThroughTrackEventViewer);
vtkInformationKeyMacro(ThroughTrackEventViewer, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
ThroughTrackEventViewer::ThroughTrackEventViewer():
  num_faces_(20),
  machine_name_("NACHO"),
  filtered_(false)
{
	running_ = false;

  SetMachineName(4); // 1 = HMT, 2 = EMT, 3 = SMT
  busy_handling_ = false;

  // std::cout << "made it past set machine name" << std::endl;
	this->Internal = new vtkInternal;

	this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(6);

  // std::cout << " loading geometry file" << std::endl;
  LoadGeometryFile();
  
}

/*----------------------------------------------------------------------------*/
ThroughTrackEventViewer::~ThroughTrackEventViewer()
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

void ThroughTrackEventViewer::LoadGeometryFile()
{
  QString fileName = QFileDialog::getOpenFileName(QWidget::find(0), 
                          QObject::tr("Open Geometry"),
                          "/opt/dsc/",
                          QObject::tr("Geometry Files (*.bin)"));

  if (fileName.toStdString().length() > 1)
  {
    mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fileName.toStdString());
    GeometryInfo_ = GeometryInfo;
  }
  std::cout << "finished loading geometry" << std::endl;
}
void ThroughTrackEventViewer::DrawKalmanTracks(ThroughTrackEventSharedPtr through_track_event)
{
  vtkSmartPointer<vtkAppendPolyData> kalAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
  kalmanTracksOutput_ = kalAppendPolyData;
  // first get the lists of incoming and outgoing hits
  std::vector<das::TrackHit> in_hit_list;
  std::vector<das::TrackHit> out_hit_list;
  std::vector<das::TrackHit>::const_iterator hit_iter = (*through_track_event).hits.begin();
  for (; hit_iter != through_track_event->hits.end(); ++hit_iter)
  {
    if((*hit_iter).wire.point.z_coordinate > 250)
    {
      in_hit_list.push_back(*hit_iter);
    }
    else
    {
      out_hit_list.push_back(*hit_iter);
    }
  }
  // sort them in descending z order for the incoming track 
  sort(in_hit_list.begin(), in_hit_list.end(), 
      [ ] (const das::TrackHit& lhs, const das::TrackHit& rhs)
      {
        return 
          (lhs.hit_line.point.z_coordinate + lhs.hit_line.length * lhs.hit_line.direction.z_coordinate) >
          (rhs.hit_line.point.z_coordinate + rhs.hit_line.length * rhs.hit_line.direction.z_coordinate);
      });
  // sort them in ascending z order for the outgoing track 
  sort(out_hit_list.begin(), out_hit_list.end(), 
      [ ] (const das::TrackHit& lhs, const das::TrackHit& rhs)
      {
        return 
          (lhs.hit_line.point.z_coordinate + lhs.hit_line.length * lhs.hit_line.direction.z_coordinate) <
          (rhs.hit_line.point.z_coordinate + rhs.hit_line.length * rhs.hit_line.direction.z_coordinate);
      });
     // create the Kalman filter 
  int n = 4; // Number of state elements (x, y, dx/dz, dy/dz)
  int m = 2; // Number of measurements (x, y)  // really only measuring radius so need to update the uncertainty covariance for the non-measurement direction 

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  double dz = 1;
  
  // dynamics matrix is simple linear projection 
  A << 1, 0, dz,  0,
       0, 1,  0, dz,
       0, 0,  1,  0, 
       0, 0,  0,  1;
 
  C << 1, 0, 0, 0,  // output matrix
       0, 1, 0, 0;

  // noise covariance matrices (mm and mrad)
  double posn = 0.000;  // position difference between measurements // should only be due to angular change
  double angn = 0.0005;  // scattering angle between measurements
  Q <<  posn, 0.000, angn*dz,    0.000,
       0.000,  posn,   0.000,  angn*dz,
       0.000, 0.000,    angn,    0.000,
       0.000, 0.000,    0.000,    angn;
  // measurement uncertainty covariance (independent) 
  // this needs to be updated for each measurement depending on the direction of the tube
  R << 0.00250, 0.000,
       0.000, 0.00250;    

  // estimate error covariance
  P <<  0.050, 0.000, 1.0e-3,  0.000,
        0.000, 0.050,  0.000, 1.0e-3,
        0.000, 0.000, 1.0e-3,  0.001,
        0.000, 0.000,  0.000, 1.0e-3;
  
  KalmanFilter kf(A, C, Q, R, P);
  
    // initialize the kalman filter state for the incoming track
  const das::Line3& incoming_line = through_track_event->through_track.incoming_trajectory.trajectory;
  das::Vector3 incoming_tip_point = GetTip(incoming_line.point, incoming_line.direction, incoming_line.length);

  double dx = incoming_line.point.x_coordinate - incoming_tip_point.x_coordinate;
  double dy = incoming_line.point.y_coordinate - incoming_tip_point.y_coordinate;
  dz = incoming_line.point.z_coordinate - incoming_tip_point.z_coordinate;
  
  // std::cout << "Track Starting Estimate: " << std::endl;
  // std::cout << incoming_line.point.x_coordinate << ", "
  //           << incoming_line.point.y_coordinate << ", "
  //           << incoming_line.point.z_coordinate << ", "
  //           << dx/dz << ", "
  //           << dy/dz << ", "
  //           << std::endl;
  
  Eigen::VectorXd x0(n);
  x0 << incoming_line.point.x_coordinate,
        incoming_line.point.y_coordinate,
        dx/dz,
        dy/dz;
  double z0 = incoming_line.point.z_coordinate;
  double z = z0;
  kf.init(z0, x0);

  vtkSmartPointer<vtkSphereSource> sphere_source = vtkSmartPointer<vtkSphereSource>::New();
  vtkSmartPointer<vtkPolyData> pointPolydata = vtkSmartPointer<vtkPolyData>::New();
  // now loop over the incoming hits and filter to the bottom of the top SM
  std::vector<das::TrackHit>::const_iterator shit_iter = in_hit_list.begin();
  int i = 0;
  int num_passes = 5;
  for(int pass = 0; pass < num_passes; pass++)
  {
    // std::cout << "PASS NUMBER:  " << pass << std::endl;
    shit_iter = in_hit_list.begin();
    for(; shit_iter != in_hit_list.end(); ++shit_iter)
    {
      vtkSmartPointer<vtkLineSource> vtk_line_source = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source->SetPoint1( kf.state()[0], kf.state()[1], kf.zpos());

      sphere_source = vtkSmartPointer<vtkSphereSource>::New();
      sphere_source->SetCenter(kf.state()[0], kf.state()[1], kf.zpos());
      sphere_source->SetRadius(0.5);
      sphere_source->Update();

      kf.update((*shit_iter)); // projects to the given hit and updates its state
      // std::cout << "index = " << i << ", "
      //           << "channel = " << (*shit_iter).index << ", "
      //           << "wire.z = " << (*shit_iter).wire.point.z_coordinate << ", "
      //           << ", x_hat = " << kf.state().transpose() << std::endl;

      vtk_line_source->SetPoint2( kf.state()[0], kf.state()[1], kf.zpos());
      vtk_line_source->Update();

      vtkSmartPointer<vtkPolyData> trajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
      trajectoryPolydata = vtk_line_source->GetOutput();
      
      vtkSmartPointer<vtkPolyData> pointPolydata = vtkSmartPointer<vtkPolyData>::New();
      pointPolydata = sphere_source->GetOutput();
      
      kalmanTracksOutput_->AddInputData(trajectoryPolydata);
      kalmanTracksOutput_->AddInputData(pointPolydata);
      kalmanTracksOutput_->Update();
      
      i++;
    }
  }
  // this next part adds the final dot on the kalman trajectory 
  sphere_source = vtkSmartPointer<vtkSphereSource>::New();
  sphere_source->SetCenter(kf.state()[0], kf.state()[1], kf.zpos());
  sphere_source->SetRadius(0.5);
  sphere_source->Update();
  
  pointPolydata = vtkSmartPointer<vtkPolyData>::New();
  pointPolydata = sphere_source->GetOutput();
  kalmanTracksOutput_->AddInputData(pointPolydata);
  kalmanTracksOutput_->Update();
  
  dsclib::Vector3 in_kal_point(kf.state()[0], kf.state()[1], kf.zpos());
  dsclib::Vector3 in_kal_dir(kf.state()[2], kf.state()[3], 1);
  
  // initialize the kalman filter state for the outgoing track
  const das::Line3& outgoing_line = through_track_event->through_track.outgoing_trajectory.trajectory;
  das::Vector3 outgoing_tip_point = GetTip(outgoing_line.point, outgoing_line.direction, outgoing_line.length);

  dx = outgoing_line.point.x_coordinate - outgoing_tip_point.x_coordinate;
  dy = outgoing_line.point.y_coordinate - outgoing_tip_point.y_coordinate;
  dz = outgoing_line.point.z_coordinate - outgoing_tip_point.z_coordinate;
  
  // std::cout << "Track Starting Estimate: " << std::endl;
  // std::cout << outgoing_line.point.x_coordinate << ", "
  //           << outgoing_line.point.y_coordinate << ", "
  //           << outgoing_line.point.z_coordinate << ", "
  //           << dx/dz << ", "
  //           << dy/dz << ", "
  //           << std::endl;
  
  x0 << outgoing_line.point.x_coordinate,
        outgoing_line.point.y_coordinate,
        dx/dz,
        dy/dz;
  z0 = outgoing_line.point.z_coordinate;
  z = z0;
  kf.init(z0, x0);
  // now loop over the outgoing hits and filter to the top of the bottom SM
  shit_iter = out_hit_list.begin();
  i = 0;
  for(int pass = 0; pass < num_passes; pass++)
  {
    // std::cout << "PASS NUMBER:  " << pass << std::endl;
    shit_iter = out_hit_list.begin();
    for(; shit_iter != out_hit_list.end(); ++shit_iter)
    {
      vtkSmartPointer<vtkLineSource> vtk_line_source = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source->SetPoint1( kf.state()[0], kf.state()[1], kf.zpos());

      sphere_source = vtkSmartPointer<vtkSphereSource>::New();
      sphere_source->SetCenter(kf.state()[0], kf.state()[1], kf.zpos());
      sphere_source->SetRadius(0.5);
      sphere_source->Update();

      kf.update((*shit_iter)); // projects to the given hit and updates its state
      // std::cout << "index = " << i << ", "
      //           << "channel = " << (*shit_iter).index << ", "
      //           << "wire.z = " << (*shit_iter).wire.point.z_coordinate << ", "
      //           << ", x_hat = " << kf.state().transpose() << std::endl;

      vtk_line_source->SetPoint2( kf.state()[0], kf.state()[1], kf.zpos());
      vtk_line_source->Update();

      vtkSmartPointer<vtkPolyData> trajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
      trajectoryPolydata = vtk_line_source->GetOutput();
      
      vtkSmartPointer<vtkPolyData> pointPolydata = vtkSmartPointer<vtkPolyData>::New();
      pointPolydata = sphere_source->GetOutput();
      
      kalmanTracksOutput_->AddInputData(trajectoryPolydata);
      kalmanTracksOutput_->AddInputData(pointPolydata);
      kalmanTracksOutput_->Update();
      
      i++;
    }
  }
  // this next part adds the final dot on the kalman trajectory 
  sphere_source = vtkSmartPointer<vtkSphereSource>::New();
  sphere_source->SetCenter(kf.state()[0], kf.state()[1], kf.zpos());
  sphere_source->SetRadius(0.5);
  sphere_source->Update();
  
  pointPolydata = vtkSmartPointer<vtkPolyData>::New();
  pointPolydata = sphere_source->GetOutput();
  kalmanTracksOutput_->AddInputData(pointPolydata);
  kalmanTracksOutput_->Update();
  
  dsclib::Vector3 out_kal_point(kf.state()[0], kf.state()[1], kf.zpos());
  dsclib::Vector3 out_kal_dir(kf.state()[2], kf.state()[3], 1);

  // now calculate the rest of the track and draw lines
  dsclib::Line3 in_kal_line(in_kal_point, in_kal_dir, 100);
  dsclib::Line3 out_kal_line(out_kal_point, out_kal_dir, 100);

  dsclib::Line3 poca_line = in_kal_line.ClosestApproach(out_kal_line);
  double poca_z = poca_line.GetPointZ() + poca_line.GetDirectionZ() * poca_line.GetLength() / 2.0;
  
  dsclib::Vector3 in_kal_tip(in_kal_line.GetXatZ(poca_z), in_kal_line.GetYatZ(poca_z), poca_z);
  dsclib::Vector3 out_kal_tip(out_kal_line.GetXatZ(poca_z), out_kal_line.GetYatZ(poca_z), poca_z);

  double scat_ang = through_track_event->through_track.scattering_angle;
  double kal_scat_ang = acos(in_kal_dir.UnitVector().DotProduct(out_kal_dir.UnitVector()));
  std::cout << "standard: scattering angle: " << scat_ang << "\t doca:  " << through_track_event->through_track.closest_approach_line.length << std::endl;
  std::cout << "  kalman: scattering angle: " << kal_scat_ang << "\t doca:  " << poca_line.GetLength() << std::endl;
  
  vtkSmartPointer<vtkLineSource> vtk_line_source_kal_incoming = vtkSmartPointer<vtkLineSource>::New();
  vtk_line_source_kal_incoming->SetPoint1(
      in_kal_point.GetX(),
      in_kal_point.GetY(),
      in_kal_point.GetZ());
  vtk_line_source_kal_incoming->SetPoint2(
      in_kal_tip.GetX(),
      in_kal_tip.GetY(),
      in_kal_tip.GetZ());
  vtk_line_source_kal_incoming->Update();
  
  vtkSmartPointer<vtkLineSource> vtk_line_source_kal_outgoing = vtkSmartPointer<vtkLineSource>::New();
  vtk_line_source_kal_outgoing->SetPoint1(
      out_kal_point.GetX(),
      out_kal_point.GetY(),
      out_kal_point.GetZ());
  vtk_line_source_kal_outgoing->SetPoint2(
      out_kal_tip.GetX(),
      out_kal_tip.GetY(),
      out_kal_tip.GetZ());
  vtk_line_source_kal_outgoing->Update();
  
   

  vtkSmartPointer<vtkPolyData> inKalmanTrajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
  inKalmanTrajectoryPolydata = vtk_line_source_kal_incoming->GetOutput();
  vtkSmartPointer<vtkPolyData> outKalmanTrajectoryPolydata = vtkSmartPointer<vtkPolyData>::New();
  outKalmanTrajectoryPolydata = vtk_line_source_kal_outgoing->GetOutput();
  kalmanTracksOutput_->AddInputData(inKalmanTrajectoryPolydata);
  kalmanTracksOutput_->AddInputData(outKalmanTrajectoryPolydata);
  kalmanTracksOutput_->Update();
    this->Modified();
    this->Update();

}
void ThroughTrackEventViewer::DrawTrajectories(ThroughTrackEventSharedPtr through_track_event)
{
      vtkSmartPointer<vtkAppendPolyData> inAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
      incomingParticleTrajectoryOutput_ = inAppendPolyData;
      vtkSmartPointer<vtkAppendPolyData> outAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
      outgoingParticleTrajectoryOutput_ = outAppendPolyData;
      
      const das::Line3& incoming_line = through_track_event->through_track.incoming_trajectory.trajectory;
      const das::Line3& outgoing_line = through_track_event->through_track.outgoing_trajectory.trajectory;
      
      das::Vector3 incoming_tip_point = GetTip(incoming_line.point, incoming_line.direction, incoming_line.length);
      dsclib::Vector3 inpoint(incoming_line.point.x_coordinate, incoming_line.point.y_coordinate, incoming_line.point.z_coordinate);
      dsclib::Vector3 intip(incoming_tip_point.x_coordinate, incoming_tip_point.y_coordinate, incoming_tip_point.z_coordinate);
      dsclib::Line3 incoming_line3(inpoint, intip);
      
      vtkSmartPointer<vtkLineSource> vtk_line_source_incoming = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source_incoming->SetPoint1(
          incoming_line.point.x_coordinate,
          incoming_line.point.y_coordinate,
          incoming_line.point.z_coordinate);

      double ztip = incoming_tip_point.z_coordinate;
      if(ztip > incoming_line.point.z_coordinate) ztip = incoming_line.point.z_coordinate;
      if(ztip < 0) ztip = 0;

      // vtk_line_source_incoming->SetPoint2(
      //     incoming_tip_point.x_coordinate,
      //     incoming_tip_point.y_coordinate,
      //     incoming_tip_point.z_coordinate);

      vtk_line_source_incoming->SetPoint2(
          incoming_line3.GetXatZ(ztip),
          incoming_line3.GetYatZ(ztip),
          ztip);
      vtk_line_source_incoming->Update();

      vtkSmartPointer<vtkLineSource> vtk_line_source_outgoing = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source_outgoing->SetPoint1(
          outgoing_line.point.x_coordinate,
          outgoing_line.point.y_coordinate,
          outgoing_line.point.z_coordinate);

      das::Vector3 outgoing_tip_point = GetTip(outgoing_line.point, outgoing_line.direction, outgoing_line.length);
      
      dsclib::Vector3 outpoint(outgoing_line.point.x_coordinate, outgoing_line.point.y_coordinate, outgoing_line.point.z_coordinate);
      dsclib::Vector3 outtip(outgoing_tip_point.x_coordinate, outgoing_tip_point.y_coordinate, outgoing_tip_point.z_coordinate);
      dsclib::Line3 outgoing_line3(outpoint, outtip);
      
      ztip = outgoing_tip_point.z_coordinate;
      if(ztip > incoming_line.point.z_coordinate) ztip = incoming_line.point.z_coordinate;
      if(ztip < 0) ztip = 0;

      // vtk_line_source_outgoing->SetPoint2(
      //     outgoing_tip_point.x_coordinate,
      //     outgoing_tip_point.y_coordinate,
      //     outgoing_tip_point.z_coordinate);
      
      vtk_line_source_outgoing->SetPoint2(
          outgoing_line3.GetXatZ(ztip),
          outgoing_line3.GetYatZ(ztip),
          ztip);
      vtk_line_source_outgoing->Update();

      vtkSmartPointer<vtkDoubleArray> momentum = vtkSmartPointer<vtkDoubleArray>::New();
      momentum->SetName("Momentum");
      momentum->InsertNextValue(through_track_event->through_track.momentum);

      vtkSmartPointer<vtkDoubleArray> scatteringAngle = vtkSmartPointer<vtkDoubleArray>::New();
      scatteringAngle->SetName("Scattering Angle");
      scatteringAngle->InsertNextValue(through_track_event->through_track.scattering_angle);

      vtkSmartPointer<vtkDoubleArray> IncomingTrackLength = vtkSmartPointer<vtkDoubleArray>::New();
      IncomingTrackLength->SetName("Incoming Track Length");
      IncomingTrackLength->InsertNextValue(incoming_line.length);

      vtkSmartPointer<vtkDoubleArray> OutgoingTrackLength = vtkSmartPointer<vtkDoubleArray>::New();
      OutgoingTrackLength->SetName("Outgoing Track Length");
      OutgoingTrackLength->InsertNextValue(outgoing_line.length);

      vtkSmartPointer<vtkDoubleArray> DOCA = vtkSmartPointer<vtkDoubleArray>::New();
      DOCA->SetName("DOCA");
      DOCA->InsertNextValue(through_track_event->through_track.closest_approach_line.length);

      vtkSmartPointer<vtkDoubleArray> xcom = vtkSmartPointer<vtkDoubleArray>::New();
      xcom->SetName("PoCA X Coordinate");
      xcom->InsertNextValue(through_track_event->through_track.poca.x_coordinate);

      vtkSmartPointer<vtkDoubleArray> ycom = vtkSmartPointer<vtkDoubleArray>::New();
      ycom->SetName("PoCA Y Coordinate");
      ycom->InsertNextValue(through_track_event->through_track.poca.y_coordinate);

      vtkSmartPointer<vtkDoubleArray> zcom = vtkSmartPointer<vtkDoubleArray>::New();
      zcom->SetName("PoCA Z Coordinate");
      zcom->InsertNextValue(through_track_event->through_track.poca.z_coordinate);

      vtkSmartPointer<vtkIntArray> TubeIndex = vtkSmartPointer<vtkIntArray>::New();
      TubeIndex->SetName("Tube Index");

      vtkSmartPointer<vtkIntArray> Ambiguity = vtkSmartPointer<vtkIntArray>::New();
      Ambiguity->SetName("Ambiguity");

      vtkSmartPointer<vtkDoubleArray> Drift = vtkSmartPointer<vtkDoubleArray>::New();
      Drift->SetName("Drift");

      vtkSmartPointer<vtkDoubleArray> DriftTime = vtkSmartPointer<vtkDoubleArray>::New();
      DriftTime->SetName("Drift Trime");
      
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

      incomingParticleTrajectoryOutput_->AddInputData(incomingTrajectoryPolydata);
      outgoingParticleTrajectoryOutput_->AddInputData(outgoingTrajectoryPolydata);
    // }

    incomingParticleTrajectoryOutput_->Update();
    outgoingParticleTrajectoryOutput_->Update();

    // this->Modified();

}
void ThroughTrackEventViewer::DrawDriftTubes(ThroughTrackEventSharedPtr through_track_event)
{
    vtkSmartPointer<vtkAppendPolyData> driftTubeOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftTubeOutput_ = driftTubeOutputAppend;

    std::vector<das::TrackHit>::const_iterator hit_iter = (*through_track_event).hits.begin();
    for (; hit_iter != through_track_event->hits.end(); ++hit_iter)
    {
        uint32_t channel = (*hit_iter).index;
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
        vtkSmartPointer<vtkIntArray> Ambiguity = vtkSmartPointer<vtkIntArray>::New();
        Ambiguity->SetName("Ambiguity");
        vtkSmartPointer<vtkDoubleArray> Drift = vtkSmartPointer<vtkDoubleArray>::New();
        Drift->SetName("Drift Radius");
        vtkSmartPointer<vtkDoubleArray> DriftTime = vtkSmartPointer<vtkDoubleArray>::New();
        DriftTime->SetName("Drift Time");
        vtkSmartPointer<vtkIntArray> Outlier = vtkSmartPointer<vtkIntArray>::New();
        Outlier->SetName("Outlier");
        vtkSmartPointer<vtkDoubleArray> Residual = vtkSmartPointer<vtkDoubleArray>::New();
        DriftTime->SetName("Residual");
        
        for (int cell = 0; cell < tube_filter->GetOutput()->GetNumberOfCells(); cell++)
        {
          TubeIndex->InsertNextTuple1((*hit_iter).index);
          Ambiguity->InsertNextTuple1((*hit_iter).ambiguity);
          Drift->InsertNextTuple1((*hit_iter).drift);
          DriftTime->InsertNextTuple1((*hit_iter).drift_time);
          if ((*hit_iter).is_outlier == true) { Outlier->InsertNextValue(0); }
          else { Outlier->InsertNextValue(1); }
          Residual->InsertNextTuple1(((*hit_iter).drift) - ((*hit_iter).hit_line.length));
        }
        tube_filter->GetOutput()->GetCellData()->AddArray(TubeIndex);
        tube_filter->GetOutput()->GetCellData()->AddArray(Ambiguity);
        tube_filter->GetOutput()->GetCellData()->AddArray(Drift);
        tube_filter->GetOutput()->GetCellData()->AddArray(DriftTime);
        tube_filter->GetOutput()->GetCellData()->AddArray(Outlier);
        tube_filter->GetOutput()->GetCellData()->AddArray(Residual);
        
        tube_filter->Update();

        driftTubeOutput_->AddInputData(tube_filter->GetOutput());
       
        // vtkSmartPointer<vtkCylinderSource> tube_source = vtkSmartPointer<vtkCylinderSource>::New();
        // tube_source->SetRadius(2.54);
        // tube_source->SetHeight(18*12*2.54);
        // tube_source->SetCenter(0,0,0);
        // tube_source->SetResolution(num_faces_);
        // tube_source->SetCapping(1);
        // tube_source->Update();
        //
        // vtkSmartPointer<vtkCylinderSource> gas_source = vtkSmartPointer<vtkCylinderSource>::New();
        // gas_source->SetRadius(1.45);
        // gas_source->SetHeight(18*12*2.54 + 1);
        // gas_source->SetCenter(0,0,0);
        // gas_source->SetResolution(num_faces_);
        // gas_source->SetCapping(1);
        // gas_source->Update();
  //
        // vtkSmartPointer<vtkBooleanOperationPolyDataFilter> booleanOperation =
        //     vtkSmartPointer<vtkBooleanOperationPolyDataFilter>::New();
        // booleanOperation->SetOperationToDifference();
        // booleanOperation->ReorientDifferenceCellsOff();
        // booleanOperation->SetInputData( 1, tube_source->GetOutput());
        // booleanOperation->SetInputData( 0, gas_source->GetOutput());
        // tube_source->Update();
        // gas_source->Update();
        // booleanOperation->Update();
        //
        // // driftTubeOutput_->AddInputData(tube_source->GetOutput());
        // driftTubeOutput_->AddInputData(booleanOperation->GetOutput(0));

    }
    driftTubeOutput_->Update();
    this->Modified();
}
void ThroughTrackEventViewer::DrawDriftCylinders(ThroughTrackEventSharedPtr through_track_event)
{
    vtkSmartPointer<vtkAppendPolyData> driftCylinderOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftCylinderOutput_ = driftCylinderOutputAppend;

    std::vector<das::TrackHit>::const_iterator hit_iter = (*through_track_event).hits.begin();
    for (; hit_iter != through_track_event->hits.end(); ++hit_iter)
    {
        uint32_t channel = (*hit_iter).index;
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
        tube_filter->SetRadius((*hit_iter).drift);
        tube_filter->SetNumberOfSides(num_faces_);
        tube_filter->SetCapping(1);
        tube_filter->Update();
        
        vtkSmartPointer<vtkIntArray> TubeIndex = vtkSmartPointer<vtkIntArray>::New();
        TubeIndex->SetName("Tube Index");
        vtkSmartPointer<vtkIntArray> Ambiguity = vtkSmartPointer<vtkIntArray>::New();
        Ambiguity->SetName("Ambiguity");
        vtkSmartPointer<vtkDoubleArray> Drift = vtkSmartPointer<vtkDoubleArray>::New();
        Drift->SetName("Drift Radius");
        vtkSmartPointer<vtkDoubleArray> DriftTime = vtkSmartPointer<vtkDoubleArray>::New();
        DriftTime->SetName("Drift Time");
        vtkSmartPointer<vtkIntArray> Outlier = vtkSmartPointer<vtkIntArray>::New();
        Outlier->SetName("Outlier");
        vtkSmartPointer<vtkDoubleArray> Residual = vtkSmartPointer<vtkDoubleArray>::New();
        DriftTime->SetName("Residual");
        
        for (int cell = 0; cell < tube_filter->GetOutput()->GetNumberOfCells(); cell++)
        {
          TubeIndex->InsertNextTuple1((*hit_iter).index);
          Ambiguity->InsertNextTuple1((*hit_iter).ambiguity);
          Drift->InsertNextTuple1((*hit_iter).drift);
          DriftTime->InsertNextTuple1((*hit_iter).drift_time);
          if ((*hit_iter).is_outlier == true) { Outlier->InsertNextValue(0); }
          else { Outlier->InsertNextValue(1); }
          Residual->InsertNextTuple1(((*hit_iter).drift) - ((*hit_iter).hit_line.length));
        }
        tube_filter->GetOutput()->GetCellData()->AddArray(TubeIndex);
        tube_filter->GetOutput()->GetCellData()->AddArray(Ambiguity);
        tube_filter->GetOutput()->GetCellData()->AddArray(Drift);
        tube_filter->GetOutput()->GetCellData()->AddArray(DriftTime);
        tube_filter->GetOutput()->GetCellData()->AddArray(Outlier);
        tube_filter->GetOutput()->GetCellData()->AddArray(Residual);
        
        tube_filter->Update();
        
        driftCylinderOutput_->AddInputData(tube_filter->GetOutput());
    }
    driftCylinderOutput_->Update();
    this->Modified();
}
void ThroughTrackEventViewer::DrawHitLines(ThroughTrackEventSharedPtr through_track_event)
{
    vtkSmartPointer<vtkAppendPolyData> hitLinesOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    hitLinesOutput_ = hitLinesOutputAppend;

    std::vector<das::TrackHit>::const_iterator hit_iter = (*through_track_event).hits.begin();
    for (; hit_iter != through_track_event->hits.end(); ++hit_iter)
    {
      const das::Line3& hit_line = (*hit_iter).hit_line;

      vtkSmartPointer<vtkLineSource> vtk_line_source = vtkSmartPointer<vtkLineSource>::New();
      vtk_line_source->SetPoint1(
          hit_line.point.x_coordinate,
          hit_line.point.y_coordinate,
          hit_line.point.z_coordinate);

      das::Vector3 tip_point = GetTip(hit_line.point, hit_line.direction, hit_line.length);

      vtk_line_source->SetPoint2(
          tip_point.x_coordinate,
          tip_point.y_coordinate,
          tip_point.z_coordinate);

      vtk_line_source->Update();
      vtkSmartPointer<vtkPolyData> hitLinePolydata = vtkSmartPointer<vtkPolyData>::New();
      hitLinePolydata = vtk_line_source->GetOutput();
      hitLinesOutput_->AddInputData(hitLinePolydata);
    }
    hitLinesOutput_->Update();
    this->Modified();
}

/*----------------------------------------------------------------------------*/
bool ThroughTrackEventViewer::HasData()
{
	return this->Internal->HasData();
}

/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::UpdatePipeline()
{
	if (this->HasData())
	{
		this->Modified();
	}
}

/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::ToggleConnection(bool checked)
{
  if(checked) Connect();
  else Disconnect();
}
/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::Connect()
{
    std::string packet_name = "ThroughTrackEventPacket";
    if(filtered_)
    {
      packet_name = "FilteredThroughTrackEventPacket";
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
	// sending_thread_ = QtConcurrent::run(this, &ThroughTrackEventViewer::PullData);
	running_ = true;
  /*
	if (sending_thread_.isFinished())
	{
	  running_ = false;
      
	  inf::icomm::ReactorFactory reactor_factory;
    reactor_factory.SetConnectionUri(zeromq_connection_uri_);
	  reactor_factory.SetSocketPolicy(inf::icomm::eSocketPolicy::SUB);
	  reactor_factory.SetConnectionPolicy(inf::icomm::eConnectionPolicy::CLIENT);
	  reactor_factory.SetMessagePolicy(inf::icomm::eMessagePolicy::ENVELOPE);
    std::string topic_name = "ThroughTrackEventPacket";
    if(filtered_)
    {
      topic_name = "FilteredThroughTrackEventPacket";
    }
    std::cout << "Connecting to:  " << zeromq_connection_uri_ << std::endl;
    std::cout << "Binding to topic:  " << topic_name << std::endl;
    // bind a function pointer to a topic data type combination
	  reactor_factory.AddHandler<ThroughTrackEventPacketAvroAdapter>(topic_name,
	          boost::bind(&ThroughTrackEventViewer::HandleIncomingParticleTrackEvent, this, _1), true );

	  zmq_reactor_.reset();
	  zmq_reactor_ = reactor_factory.MakeReactor();

      running_ = true;
      // the following is commented to allow manual pulling of data
		// sending_thread_ = QtConcurrent::run(this, &ThroughTrackEventViewer::PullData);
    //   std::cout << "Connected To ZMQ" << std::endl;
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
    std::string topic_name = "ThroughTrackEventPacket";
    if(filtered_)
    {
      topic_name = "FilteredThroughTrackEventPacket";
    }

    // bind a function pointer to a topic data type combination
    reactor_factory.AddHandler<ThroughTrackEventPacketAvroAdapter>(topic_name,
        boost::bind(&ThroughTrackEventViewer::HandleIncomingParticleTrackEvent, this, _1), true );

    running_ = true;
      // the following is commented to allow manual pulling of data
    // sending_thread_ = QtConcurrent::run(this, &ThroughTrackEventViewer::PullData);
    // std::cout << "Connected To ZMQ" << std::endl;
	}
  */
}
void ThroughTrackEventViewer::PullNextTrack()
{
  Connect();
  PullData(); // pull manually, rather than using QtConcurrent
  // wait until it's done handling
  // while(busy_handling_){printf("waiting for HandleIncomingParticleTrackEvent to finish\n");}
  Disconnect();
}
/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::PullData()
{
    printf("trying to pull through track event packet\n");
    AvroAdapter<ThroughTrackEventPacket> tt_pack_aa;     
    tt_pack_aa = pKafkaTrackReader_->ReadNextThroughTrackEventPacket();
    if(tt_pack_aa.GetData() != NULL)
    {
		  HandleIncomingParticleTrackEvent(tt_pack_aa.GetData());
    }


  // int num_calls = 0;
	// while (running_ && num_calls++<100)  // keep looping until we get data and HandleIncomingTrackEvent is called
	// {
  //   std::cout << "trying to pull data" << std::endl;
	//   zmq_reactor_->HandleEvents(100);
  //   // running_ = false;
	// }
}


/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::Disconnect()
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

void ThroughTrackEventViewer::Clear()
{
	vtkSmartPointer<vtkAppendPolyData> inAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	incomingParticleTrajectoryOutput_ = inAppendPolyData;
	vtkSmartPointer<vtkAppendPolyData> outAppendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	outgoingParticleTrajectoryOutput_ = outAppendPolyData;
    vtkSmartPointer<vtkAppendPolyData> driftTubeOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftTubeOutput_ = driftTubeOutputAppend;
    vtkSmartPointer<vtkAppendPolyData> driftCylinderOutputAppend = vtkSmartPointer<vtkAppendPolyData>::New();
    driftCylinderOutput_ = driftCylinderOutputAppend;
	vtkSmartPointer<vtkAppendPolyData> hitLinePolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	hitLinesOutput_ = hitLinePolyData;
	vtkSmartPointer<vtkAppendPolyData> kalmanTracksPolyData = vtkSmartPointer<vtkAppendPolyData>::New();
	kalmanTracksOutput_ = kalmanTracksPolyData;


	this->Internal->PolyData = incomingParticleTrajectoryOutput_->GetOutput();
	this->Internal->NewData = true;
}

/*----------------------------------------------------------------------------*/
void ThroughTrackEventViewer::HandleIncomingParticleTrackEvent(
		ThroughTrackEventPacketSharedPtr through_track_event_packet)
{
  running_ = false;
  busy_handling_ = true;
  std::cout << "handling track event packet" << std::endl;
	assert(through_track_event_packet);
    std::cout << "Got through_track_event_packet with " << through_track_event_packet->tracks.size() << " tracks" << std::endl;
	this->AppendToPolyData(through_track_event_packet);
  busy_handling_ = false;
}

void ThroughTrackEventViewer::AppendToPolyData(
		ThroughTrackEventPacketSharedPtr through_track_event_packet)
{
  if (GeometryInfo_)
  {
    // only do the first track for now.  Later change it to a loop over the desired number of tracks
	std::vector<das::ThroughTrackEvent>::const_iterator track_event_iter = through_track_event_packet->tracks.begin();
    // for (; iter != through_track_event_packet->tracks.end(); ++iter)
    // {
      ThroughTrackEventSharedPtr track_event_shared_ptr(new das::ThroughTrackEvent(*track_event_iter));
      DrawTrajectories(track_event_shared_ptr);
      DrawDriftTubes(track_event_shared_ptr);
      DrawDriftCylinders(track_event_shared_ptr);
      DrawHitLines(track_event_shared_ptr);
      DrawKalmanTracks(track_event_shared_ptr);


    // this->Internal->PolyData = particleTrajectoryOutput_->GetOutput();
    this->Internal->NewData = true;
    //
    // vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
    // particleTrajectoryOutput_ = append;
  }
}

/*----------------------------------------------------------------------------*/
int ThroughTrackEventViewer::RequestData(vtkInformation *request,
    vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  vtkInformation* outInfoIncomingParticleTrack = outputVector->GetInformationObject(0);
  outInfoIncomingParticleTrack->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Incoming Particle Trajectory");
  vtkSmartPointer<vtkDataSet> incomingParticleTrackOutput = vtkDataSet::SafeDownCast(outInfoIncomingParticleTrack->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkInformation* outInfoOutgoingParticleTrack = outputVector->GetInformationObject(1);
  outInfoOutgoingParticleTrack->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Outgoing Particle Trajectory");
  vtkSmartPointer<vtkDataSet> outgoingParticleTrackOutput = vtkDataSet::SafeDownCast(outInfoOutgoingParticleTrack->Get(vtkDataObject::DATA_OBJECT()));


  vtkInformation* outInfoDriftTubes = outputVector->GetInformationObject(2);
  outInfoDriftTubes->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Drift Tubes");
  vtkSmartPointer<vtkDataSet> driftTubeOutput = vtkDataSet::SafeDownCast(outInfoDriftTubes->Get(vtkDataObject::DATA_OBJECT()));

  vtkInformation* outInfoDriftCylinders = outputVector->GetInformationObject(3);
  outInfoDriftCylinders->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Drift Cylinders");
  vtkSmartPointer<vtkDataSet> driftCylinderOutput = vtkDataSet::SafeDownCast(outInfoDriftCylinders->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkInformation* outInfoHitLines = outputVector->GetInformationObject(4);
  outInfoHitLines->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Hit Lines");
  vtkSmartPointer<vtkDataSet> hitLinesOutput = vtkDataSet::SafeDownCast(outInfoHitLines->Get(vtkDataObject::DATA_OBJECT()));

  vtkInformation* outInfoKalmanTracks = outputVector->GetInformationObject(5);
  outInfoKalmanTracks->Set(ThroughTrackEventViewer::DESCRIPTIVE_NAME(), "Kalman Tracks");
  vtkSmartPointer<vtkDataSet> kalmanTracksOutput = vtkDataSet::SafeDownCast(outInfoKalmanTracks->Get(vtkDataObject::DATA_OBJECT()));


  if (!this->HasData() || driftTubeOutput_ == NULL || driftCylinderOutput_ == NULL || hitLinesOutput_ == NULL || kalmanTracksOutput_ == NULL)
  {
    return 1;
  }

  incomingParticleTrackOutput->ShallowCopy(incomingParticleTrajectoryOutput_->GetOutput());
  outgoingParticleTrackOutput->ShallowCopy(outgoingParticleTrajectoryOutput_->GetOutput());
  driftTubeOutput->ShallowCopy(driftTubeOutput_->GetOutput());
  driftCylinderOutput->ShallowCopy(driftCylinderOutput_->GetOutput());
  hitLinesOutput->ShallowCopy(hitLinesOutput_->GetOutput());
  kalmanTracksOutput->ShallowCopy(kalmanTracksOutput_->GetOutput());

  return 1;
}

void ThroughTrackEventViewer::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}

das::Vector3 ThroughTrackEventViewer::GetTip(const das::Vector3& point,
    const das::Vector3& direction, double length)
{
	das::Vector3 tip_point;
	tip_point.x_coordinate = (point.x_coordinate + direction.x_coordinate * length);
	tip_point.y_coordinate = (point.y_coordinate + direction.y_coordinate * length);
	tip_point.z_coordinate = (point.z_coordinate + direction.z_coordinate * length);

	return tip_point;
}
void ThroughTrackEventViewer::SetMachineName(int name_enum)
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
}
