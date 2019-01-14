#ifndef __DssVtiMaterialVolumesFiltered_h
#define __DssVtiMaterialVolumesFiltered_h

#include <QtCore>
#if QT_VERSION >= 0x050000
#include <QtConcurrent/QtConcurrentRun>
#else
#include <QtCore/QtConcurrentRun>
#endif

#include "vtkPVVTKExtensionsDefaultModule.h"
#include "vtkImageAlgorithm.h"

#include "vtkImageData.h"
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

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT DssVtiMaterialVolumesFiltered : public vtkImageAlgorithm
{

public:
	vtkTypeMacro(DssVtiMaterialVolumesFiltered, vtkImageAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static DssVtiMaterialVolumesFiltered *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	void UpdatePipelineWithImage(vtkImageData* image);

	bool HasData();

	void UpdateRenderer();
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
	int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector* outputVector);

	DssVtiMaterialVolumesFiltered();
	virtual ~DssVtiMaterialVolumesFiltered();


private:

	DssVtiMaterialVolumesFiltered(const DssVtiMaterialVolumesFiltered&);
	void operator=(const DssVtiMaterialVolumesFiltered&);
	
  bool connected_to_kafka_;
	KafkaVTKDataTransport *pKafkaVTKDataTransport_;
  int64_t high_water_mark_offset_;
	vtkSmartPointer<vtkImageData> kafka_image_;
  std::string kafka_broker_name_;
  std::string machine_name_;

	class vtkInternal;
	vtkInternal * Internal;
};

#endif /* __DssVtiMaterialVolumesFiltered_h */
