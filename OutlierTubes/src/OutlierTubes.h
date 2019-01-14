#ifndef __OutlierTubes_h
#define __OutlierTubes_h

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

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT OutlierTubes : public vtkPolyDataAlgorithm
{

public:
	vtkTypeMacro(OutlierTubes, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static OutlierTubes *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	void HandleIncomingParticleTrackEvent(ThroughTrackEventPacketSharedPtr polydata_objects);

  void InitializeOutlierCountArrays();
	void LoadGeometryFile();
	void CreateDriftTubes();
  void UpdateOutlierRateArray();
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


	vtkSmartPointer<vtkAppendPolyData> vtk_append_poly_data_;
	// vtkSmartPointer<vtkAppendPolyData> good_tubes_;
	// vtkSmartPointer<vtkAppendPolyData> outlier_tubes_;

protected:

	virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

	OutlierTubes();
	virtual ~OutlierTubes();


private:
	/** Will handle waiting for data and calling the appropriate function when data comes in */
	//boost::shared_ptr<ZmqReactor> zmq_reactor_;
	inf::icomm::ZmqReactorSharedPtr zmq_reactor_;

	/** The result of running the QtConcurrent thread */
	QFuture<void> sending_thread_;

	double *display_outlier_rate_array_;
  std::vector<double> outlier_rate_array_;

	/** */
	bool running_;
	
  OutlierTubes(const OutlierTubes&);
	void operator=(const OutlierTubes&);
  std::vector<std::vector<double> > outlier_rate_comp_array_; 
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

#endif /* __OutlierTubes_h */
