#ifndef __RateViewer_h
#define __RateViewer_h

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

#include <vector>
// BOOST INCLUDE FILES
#include <boost/shared_ptr.hpp>

#include "HitPulsePacket.hh"
#include "mtlib-types/scanner/GeometryInfoIOBin.h"

#include <icomm/zmq_interface.h>
#include <icomm/marshal/AvroAdapter.h>

/** Boost shared pointer wrapper for EventPackets */
typedef boost::shared_ptr<das::HitPulsePacket> HitPulsePacketSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::HitPulsePacket> HitPulsePacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT RateViewer : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(RateViewer, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static RateViewer *New();
  static vtkInformationStringKey *DESCRIPTIVE_NAME();

  void HandleIncomingHitPulsePacket(HitPulsePacketSharedPtr polydata_objects);

  void InitializeHitCountArrays();
  void LoadGeometryFile();
  void CreateDriftTubes();
  void UpdateRateArray();
  void UpdatePolyData();

  bool HasData();

  void UpdatePipeline();
  void Connect();
  void Disconnect();
  void ToggleConnection(bool checked);
  void Clear();
  void PullData();

  void SetBackgroundSubtractOn();
  void SetBackgroundSubtractOff();
  void ToggleBackgroundSubtract(bool checked);
  void LoadRateBackgroundFile();
  void WriteRateBackgroundFile();
  void BuildRateBackground();
  void SetUpdateTimeInterval(int seconds);
  void SetMachineName(int name_enum);
  
  mtlib::GeometryInfoSharedPtr GeometryInfo_;


  vtkSmartPointer<vtkAppendPolyData> vtk_append_poly_data_;
  vtkSmartPointer<vtkAppendPolyData> sphereOutput;


protected:

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);
  

  RateViewer();
  virtual ~RateViewer();


private:
  /** Will handle waiting for data and calling the appropriate function when data comes in */
  inf::icomm::ZmqReactorSharedPtr zmq_reactor_;

  /** The result of running the QtConcurrent thread */
  QFuture<void> sending_thread_;

  /** */
  bool running_;
  /**
   * A WritePointer to the array in the tube polydata
   * Note this array has 50 times the elements as the number
   * of tube sections and 100 times the number of ch
   * because it is associated with the cells in
   * the tube polydata, not the tube sections.
   * */
  double * display_rate_array_;
  
  std::vector<double> rate_array_;
  /**
   * An array of the background rates per *_channel_* per *_packet_*.
   * */
  std::vector<double> bkgd_rate_array_;
  bool subtract_background_;

  RateViewer(const RateViewer&);
  void operator=(const RateViewer&);

  std::vector<std::vector<double> > rate_comp_array_; 
  int current_comp_;
  int num_channels_;
  int num_components_;
  int num_faces_;
  double max_packet_rate_;

  std::string machine_name_;
  std::string zeromq_connection_uri_;

  class vtkInternal;
  vtkInternal * Internal;
};

#endif /* __RateViewer_h */
