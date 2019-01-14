#ifndef __ThroughTrackEventAvroViewer_h
#define __ThroughTrackEventAvroViewer_h

#include <QtConcurrent>

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

#include <icomm/marshal/AvroAdapter.h>
#include "AvroTrackEventReader.h"

#include "kalman-tracker/kalman.hpp"

/** Boost shared pointer wrapper for EventPackets */
typedef boost::shared_ptr<das::ThroughTrackEventPacket> ThroughTrackEventPacketSharedPtr;
typedef boost::shared_ptr<das::ThroughTrackEvent> ThroughTrackEventSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::ThroughTrackEventPacket> ThroughTrackEventPacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT ThroughTrackEventAvroViewer : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(ThroughTrackEventAvroViewer, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static ThroughTrackEventAvroViewer *New();
  static vtkInformationStringKey *DESCRIPTIVE_NAME();

  void AppendToPolyData(das::ThroughTrackEvent through_track_event);
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
  void OpenAvro();
  void CloseAvro();
  void PullNextTrack();
  void Clear();
 

  mtlib::GeometryInfoSharedPtr GeometryInfo_;
  vtkSmartPointer<vtkAppendPolyData> incomingParticleTrajectoryOutput_;
  vtkSmartPointer<vtkAppendPolyData> outgoingParticleTrajectoryOutput_;
  vtkSmartPointer<vtkAppendPolyData> driftTubeOutput_;
  vtkSmartPointer<vtkAppendPolyData> driftCylinderOutput_;
  vtkSmartPointer<vtkAppendPolyData> hitLinesOutput_;
  vtkSmartPointer<vtkAppendPolyData> kalmanTracksOutput_;

  das::Vector3 GetTip(const das::Vector3& point, const das::Vector3& direction, double length);

protected:

  virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

  ThroughTrackEventAvroViewer();
  virtual ~ThroughTrackEventAvroViewer();


private:
  /** The result of running the QtConcurrent thread */
  QFuture<void> sending_thread_;

  ThroughTrackEventAvroViewer(const ThroughTrackEventAvroViewer&);
  void operator=(const ThroughTrackEventAvroViewer&);
  int num_faces_;
  
  AvroTrackEventReader<das::ThroughTrackEventPacket> ttep_reader_;
  ThroughTrackEventPacketSharedPtr current_through_track_event_packet_;
	std::vector<das::ThroughTrackEvent>::const_iterator current_through_track_event_iter_;
	ThroughTrackEventSharedPtr current_through_track_event_;
  class vtkInternal;
  vtkInternal *Internal;
};

#endif /* __ThroughTrackEventAvroViewer_h */
