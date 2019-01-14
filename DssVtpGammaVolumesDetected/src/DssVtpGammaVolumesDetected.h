#ifndef __DssVtpGammaVolumesDetected_h
#define __DssVtpGammaVolumesDetected_h

#include <QtCore>
#if QT_VERSION >= 0x050000
#include <QtConcurrent/QtConcurrentRun>
#else
#include <QtCore/QtConcurrentRun>
#endif

#include "vtkPVVTKExtensionsDefaultModule.h"
#include "vtkPolyDataAlgorithm.h"

#include "vtkPolyData.h"
#include "vtkDoubleArray.h"
#include "vtkIdList.h"
#include "vtkCellLocator.h"
#include "vtkSmartPointer.h"
#include <vtkSMSourceProxy.h>
#include <vtkSMIntVectorProperty.h>

// DSIC Include Files
#include "icomm/transport/KafkaAPI.h"
#include "icomm/marshal/buffer/IOBuffer.h"

#include "icomm/transport/KafkaVTKDataTransport.h"

// BOOST INCLUDE FILES
#include <boost/shared_ptr.hpp>

// namespaces
using namespace inf::icomm;

#include <unistd.h>
#include <time.h>

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT DssVtpGammaVolumesDetected : public vtkPolyDataAlgorithm
{

public:
	vtkTypeMacro(DssVtpGammaVolumesDetected, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static DssVtpGammaVolumesDetected *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	void UpdatePipelineWithPolyData(vtkPolyData* polyData);

	bool HasData();

	void Clear();
	void UpdatePipeline();
  void ToggleConnection(bool checked);
  void Connect();
  void Disconnect();
	void ReadKafkaBus();
	void ForceReadKafkaBus();
  void SetMachineName(int name_enum);


protected:

	virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

	DssVtpGammaVolumesDetected();
	virtual ~DssVtpGammaVolumesDetected();


private:

	DssVtpGammaVolumesDetected(const DssVtpGammaVolumesDetected&);
	void operator=(const DssVtpGammaVolumesDetected&);

  bool connected_to_kafka_;
	KafkaVTKDataTransport *pKafkaVTKDataTransport_;
  int64_t high_water_mark_offset_;
	vtkSmartPointer<vtkPolyData> kafka_poly_data_;
  std::string kafka_broker_name_;
  std::string machine_name_;

	class vtkInternal;
	vtkInternal * Internal;
};

#endif /* __DssVtpGammaVolumesDetected_h */
