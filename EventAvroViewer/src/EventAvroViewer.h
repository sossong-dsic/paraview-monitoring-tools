#ifndef __EventAvroViewer_h
#define __EventAvroViewer_h

#include <QtConcurrent>

#include "vtkPVVTKExtensionsDefaultModule.h"
#include "vtkPolyDataAlgorithm.h"

#include "vtkAppendPolyData.h"
#include "vtkSmartPointer.h"
#include <vtkSMSourceProxy.h>
#include <vtkSMIntVectorProperty.h>

// BOOST INCLUDE FILES
#include <boost/shared_ptr.hpp>

#include "EventPacket.hh"
#include "mtlib-types/scanner/GeometryInfoIOBin.h"

#include <icomm/marshal/AvroAdapter.h>
#include "AvroEventReader.h"

#include "kalman-tracker/kalman.hpp"

/** Boost shared pointer wrapper for EventPackets */
typedef boost::shared_ptr<das::EventPacket> EventPacketSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::EventPacket> EventPacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT EventAvroViewer : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(EventAvroViewer, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static EventAvroViewer *New();
  static vtkInformationStringKey *DESCRIPTIVE_NAME();

  void AppendToPolyData(std::vector<das::HitPulse> event);
  void LoadGeometryFile();
  /*  brief:  draw the drift tubes for an event
   *
   */
  void DrawDriftTubes(std::vector<das::HitPulse> event);
  /*  brief:  draw the drift radii cylinders for an event
   *
   */
  
  bool HasData();

  void UpdatePipeline();
  void OpenAvro();
  void CloseAvro();
  void PullNextEvent();
  void Clear();
 

  mtlib::GeometryInfoSharedPtr GeometryInfo_;
  vtkSmartPointer<vtkAppendPolyData> driftTubeOutput_;

  das::Vector3 GetTip(const das::Vector3& point, const das::Vector3& direction, double length);

protected:

  virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

  EventAvroViewer();
  virtual ~EventAvroViewer();


private:
  /** The result of running the QtConcurrent thread */
  QFuture<void> sending_thread_;

  EventAvroViewer(const EventAvroViewer&);
  void operator=(const EventAvroViewer&);
  int num_faces_;
  
  AvroEventReader<das::EventPacket> ttep_reader_;
  EventPacketSharedPtr current_event_packet_;
  std::vector<das::EventIndices>::const_iterator current_event_indices_;
  class vtkInternal;
  vtkInternal *Internal;
};

#endif /* __EventAvroViewer_h */
