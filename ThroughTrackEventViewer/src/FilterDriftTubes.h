#ifndef __FilterDriftTubes_h
#define __FilterDriftTubes_h

#include "vtkPVVTKExtensionsDefaultModule.h"
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "mtlib-types/scanner/GeometryInfoIOBin.h"

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT FilterDriftTubes : public vtkPolyDataAlgorithm
{
public:
	vtkTypeMacro(FilterDriftTubes, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static FilterDriftTubes *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	void SetTubeRange(double first, double last);
	vtkGetVectorMacro(TubeRange, double, 2);

	bool isTubeWithinRange(int tube_index);

	vtkGetMacro(Outlier, int);
	void SetOutliers(int outlier);

	vtkGetMacro(Ambiguity, int);
	void SetAmbiguity(int ambiguity);

	vtkGetMacro(Drift, double);
	void SetDrift(double drift);

	vtkGetMacro(DriftTime, double);
	void SetDriftTime(double drifttime);

	void LoadGeometryFile();
	mtlib::GeometryInfoSharedPtr GeometryInfo_;

protected:

	virtual int RequestData(vtkInformation *request,
			vtkInformationVector **inputVector,
			vtkInformationVector *outputVector);

	FilterDriftTubes();
	virtual ~FilterDriftTubes();

	double TubeRange[2];
	int Outlier;
	int Ambiguity;
	double Drift;
	double DriftTime;

private:
	FilterDriftTubes(const FilterDriftTubes&);  // Not implemented.
	void operator=(const FilterDriftTubes&);  // Not implemented.
};

#endif


