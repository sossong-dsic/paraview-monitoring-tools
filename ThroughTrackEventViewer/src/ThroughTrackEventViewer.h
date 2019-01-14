#ifndef __ThroughTrackEventViewer_h
#define __ThroughTrackEventViewer_h

#include <QtCore>
#if QT_VERSION >= 0x050000
#include <QtConcurrent/QtConcurrentRun>
#else
#include <QtCore/QtConcurrentRun>
#endif

#include "vtkPVVTKExtensionsDefaultModule.h"
#include "vtkPolyDataAlgorithm.h"

#include "vtkAppendPolyData.h"
#include "vtkSmartPointer.h"
#include <vtkSMSourceProxy.h>
#include <vtkSMIntVectorProperty.h>

// BOOST INCLUDE FILES
#include <boost/shared_ptr.hpp>

#include "ThroughTrackEventPacket.hh"
#include "mtlib-types/scanner/GeometryInfoIOBin.h"

#include <icomm/zmq_interface.h>
#include <icomm/marshal/AvroAdapter.h>

// DSIC Include Files
#include "icomm/transport/KafkaAPI.h"
#include "icomm/marshal/buffer/IOBuffer.h"

#include "icomm/transport/KafkaTrackReader.h"

#include "kalman-tracker/kalman.hpp"

/** Boost shared pointer wrapper for EventPackets */
typedef boost::shared_ptr<das::ThroughTrackEventPacket> ThroughTrackEventPacketSharedPtr;
typedef boost::shared_ptr<das::ThroughTrackEvent> ThroughTrackEventSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::ThroughTrackEventPacket> ThroughTrackEventPacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT ThroughTrackEventViewer : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(ThroughTrackEventViewer, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static ThroughTrackEventViewer *New();
  static vtkInformationStringKey *DESCRIPTIVE_NAME();

  void HandleIncomingParticleTrackEvent(ThroughTrackEventPacketSharedPtr polydata_objects);
  void AppendToPolyData(ThroughTrackEventPacketSharedPtr through_track_event_packet);
  void LoadGeometryFile();
  /*  brief:  draw the drift tubes for an event
   *
   */
  void DrawDriftTubes(ThroughTrackEventSharedPtr through_track_event);
  /*  brief:  draw the drift radii cylinders for an event
   *
   */
  void DrawDriftCylinders(ThroughTrackEventSharedPtr through_track_event);
  /*  brief:  draw the trajectories for an event
   *
   */
  void DrawTrajectories(ThroughTrackEventSharedPtr through_track_event);
  /*  brief:  draw the hit lines for an event
   *
   */
  void DrawHitLines(ThroughTrackEventSharedPtr through_track_event);
  /*  brief:  draw the track segments for the kalman filtered trajectories
   *
   */
  void DrawKalmanTracks(ThroughTrackEventSharedPtr through_track_event);
  
  bool HasData();

  void UpdatePipeline();
  void Connect();
  void Disconnect();
  void PullNextTrack();
  void ToggleConnection(bool checked);
  void Clear();
  void PullData();
 

  void SetMachineName(int name_enum);
  mtlib::GeometryInfoSharedPtr GeometryInfo_;
  void SetFiltered(bool checked) { filtered_ = checked; }
  vtkSmartPointer<vtkAppendPolyData> incomingParticleTrajectoryOutput_;
  vtkSmartPointer<vtkAppendPolyData> outgoingParticleTrajectoryOutput_;
  vtkSmartPointer<vtkAppendPolyData> driftTubeOutput_;
  vtkSmartPointer<vtkAppendPolyData> driftCylinderOutput_;
  vtkSmartPointer<vtkAppendPolyData> hitLinesOutput_;
  vtkSmartPointer<vtkAppendPolyData> kalmanTracksOutput_;

  das::Vector3 GetTip(const das::Vector3& point, const das::Vector3& direction, double length);

protected:

  virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

  ThroughTrackEventViewer();
  virtual ~ThroughTrackEventViewer();


private:
  /** Will handle waiting for data and calling the appropriate function when data comes in */
  inf::icomm::ZmqReactorSharedPtr zmq_reactor_;
  /** The result of running the QtConcurrent thread */
  QFuture<void> sending_thread_;

  /** */
  bool running_;
  bool busy_handling_;

  ThroughTrackEventViewer(const ThroughTrackEventViewer&);
  void operator=(const ThroughTrackEventViewer&);
  int num_faces_;
  
  std::string machine_name_;
  std::string zeromq_connection_uri_;
  std::string kafka_broker_name_;
  KafkaTrackReader *pKafkaTrackReader_;
  bool connected_to_kafka_;
  int64_t high_water_mark_offset_;

  bool filtered_;


  class vtkInternal;
  vtkInternal *Internal;
};

#endif /* __ThroughTrackEventViewer_h */
