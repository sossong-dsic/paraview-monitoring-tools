#ifndef __TrackEventTubePropertyViewer_h
#define __TrackEventTubePropertyViewer_h

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

#include "ThroughTrackEventPacket.hh"
#include "mtlib-types/scanner/GeometryInfoIOBin.h"

#include <icomm/zmq_interface.h>
#include <icomm/marshal/AvroAdapter.h>

/** Boost shared pointer wrapper for EventPackets */
typedef boost::shared_ptr<das::ThroughTrackEventPacket> ThroughTrackEventPacketSharedPtr;
/** Typedef for the AvroAdapter and what type of AVRO object it will handle */
typedef inf::icomm::AvroAdapter<das::ThroughTrackEventPacket> ThroughTrackEventPacketAvroAdapter;

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT TrackEventTubePropertyViewer : public vtkPolyDataAlgorithm
{

public:
	vtkTypeMacro(TrackEventTubePropertyViewer, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static TrackEventTubePropertyViewer *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	void HandleIncomingParticleTrackEvent(ThroughTrackEventPacketSharedPtr polydata_objects);

  void InitializeCountArrays();
	void LoadGeometryFile();
	void CreateDriftTubes();
  void UpdateArrays();
  void UpdatePolyData();

	bool HasData();

  void UpdatePipeline();
	void Connect();
	void Disconnect();
  void ToggleConnection(bool checked);
	void Clear();
	void PullData();

  void SetUpdateTimeInterval(int seconds);
  void SetMachineName(int name_enum);

	mtlib::GeometryInfoSharedPtr GeometryInfo_;
  void SetFiltered(bool checked)
  { 
    filtered_ = checked;
    if(machine_name_ == "HMT")
    { 
      SetMachineName(1);
    }
    else if(machine_name_ == "EMT")
    {
      SetMachineName(2);
    }
    else
    {
      SetMachineName(3);
    }
  }


	vtkSmartPointer<vtkAppendPolyData> vtk_append_poly_data_;

protected:

	virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

	TrackEventTubePropertyViewer();
	virtual ~TrackEventTubePropertyViewer();


private:
	/** Will handle waiting for data and calling the appropriate function when data comes in */
	//boost::shared_ptr<ZmqReactor> zmq_reactor_;
	inf::icomm::ZmqReactorSharedPtr zmq_reactor_;

	/** The result of running the QtConcurrent thread */
	QFuture<void> sending_thread_;

	double *display_array_;
  std::vector<double> array_;

	/** */
	bool running_;

  bool filtered_;
	
  TrackEventTubePropertyViewer(const TrackEventTubePropertyViewer&);
	void operator=(const TrackEventTubePropertyViewer&);
  std::vector<std::vector<double> > comp_array_; 
  int current_comp_;
  int num_channels_;
  int num_components_;
  int num_faces_;
  double max_packet_rate_;

  std::string machine_name_;
  std::string zeromq_connection_uri_;

  bool normalize_by_ntracks_;

	class vtkInternal;
	vtkInternal * Internal;
};

#endif /* __TrackEventTubePropertyViewer_h */
