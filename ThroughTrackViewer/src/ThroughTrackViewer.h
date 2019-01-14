#ifndef __ThroughTrackViewer_h
#define __ThroughTrackViewer_h

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

#include "ThroughTrackPacket.hh"

#include <icomm/zmq_interface.h>
#include <icomm/marshal/AvroAdapter.h>

// DSIC Include Files
#include "icomm/transport/KafkaAPI.h"
#include "icomm/marshal/buffer/IOBuffer.h"

#include "icomm/transport/KafkaTrackReader.h"

/** Boost shared pointer wrapper for ThroughTrackPackets */
typedef boost::shared_ptr<das::ThroughTrackPacket> ThroughTrackPacketSharedPtr;
typedef boost::shared_ptr<das::ThroughTrack> ThroughTrackSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::ThroughTrackPacket> ThroughTrackPacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT ThroughTrackViewer : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(ThroughTrackViewer, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static ThroughTrackViewer *New();
  static vtkInformationStringKey *DESCRIPTIVE_NAME();

  void HandleIncomingParticleTracks(ThroughTrackPacketSharedPtr polydata_objects);
  void AppendToPolyData(ThroughTrackPacketSharedPtr through_track_packet);
  
  bool HasData();

  void UpdatePipeline();
  void Connect();
  void Disconnect();
  void ToggleConnection(bool checked);
  void Clear();
  void PullData();

  
  void SetMachineName(int name_enum);
  void SetFiltered(bool checked) { filtered_ = checked; }
  
  vtkSmartPointer<vtkAppendPolyData> incomingParticleTrajectoryBuffer_;
  vtkSmartPointer<vtkAppendPolyData> outgoingParticleTrajectoryBuffer_;
  vtkSmartPointer<vtkAppendPolyData> incomingParticleTrajectoryOutput_;
  vtkSmartPointer<vtkAppendPolyData> outgoingParticleTrajectoryOutput_;
  
  vtkSmartPointer<vtkAppendPolyData> sphereBuffer_; // for the PoCA 
  vtkSmartPointer<vtkAppendPolyData> sphereOutput_; // for the PoCA 

  das::Vector3 GetTip(const das::Vector3& point, const das::Vector3& direction, double length);
  void SetUpdateTimeInterval(int seconds);
  void SetMinScatteringAngleCut(double cut) {min_scattering_angle_cut_ = cut;}

protected:

  virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

  ThroughTrackViewer();
  virtual ~ThroughTrackViewer();


private:
  /** Will handle waiting for data and calling the appropriate function when data comes in */
  inf::icomm::ZmqReactorSharedPtr zmq_reactor_;
  /** The result of running the QtConcurrent thread */
  QFuture<void> sending_thread_;

  /** */
  bool running_;
  int display_interval_;
  int packet_count_;
  double min_scattering_angle_cut_;

  std::string machine_name_;
  std::string zeromq_connection_uri_;
  std::string kafka_broker_name_;
  KafkaTrackReader *pKafkaTrackReader_;
  bool connected_to_kafka_;
  int64_t high_water_mark_offset_;

  bool filtered_;

  ThroughTrackViewer(const ThroughTrackViewer&);
  void operator=(const ThroughTrackViewer&);

  class vtkInternal;
  vtkInternal * Internal;
};

#endif /* __ThroughTrackViewer_h */
