#ifndef __TrackManipulation_h
#define __TrackManipulation_h

#include "vtkPVVTKExtensionsDefaultModule.h"
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT TrackManipulation : public vtkPolyDataAlgorithm
{
public:
	vtkTypeMacro(TrackManipulation, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static TrackManipulation *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	vtkGetMacro(Momentum, double);
	void SetMomentum(double momentum);

	vtkGetMacro(ScatteringAngle, double);
	void SetScatteringAngle(double scatteringAngle);

	vtkGetMacro(DOCA, double);
	void SetDOCA(double doca);

	vtkGetMacro(Ambiguity, int);
	void SetAmbiguity(int ambiguity);

	vtkGetMacro(Drift, double);
	void SetDrift(double drift);

	vtkGetMacro(DriftTime, double);
	void SetDriftTime(double drifttime);

	vtkGetMacro(TrackLength, double);
	void SetTrackLength(double length);

	void SetXBounds(double x0, double x1);
	vtkGetVectorMacro(XBounds, double, 2);

	void SetYBounds(double y0, double y1);
	vtkGetVectorMacro(YBounds, double, 2);

	void SetZBounds(double y0, double y1);
	vtkGetVectorMacro(ZBounds, double, 2);

	void Clear();
	void UpdateMomentumThreshold();
	void UpdateScatteringAngleThreshold();
	bool isPoCAWithinBounds(double x, double y, double z);

	vtkSmartPointer<vtkPolyData> vtk_poly_data;

protected:

	virtual int RequestData(vtkInformation *request,
			vtkInformationVector **inputVector,
			vtkInformationVector *outputVector);

	TrackManipulation();
	virtual ~TrackManipulation();

	double Momentum;
	double ScatteringAngle;
	double DOCA;
	int Ambiguity;
	double Drift;
	double DriftTime;
	double TrackLength;
	double XBounds[2];
	double YBounds[2];
	double ZBounds[2];

private:
	TrackManipulation(const TrackManipulation&);  // Not implemented.
	void operator=(const TrackManipulation&);  // Not implemented.
};

#endif


