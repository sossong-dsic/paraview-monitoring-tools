#ifndef __IncomingTrackManipulation_h
#define __IncomingTrackManipulation_h

#include "vtkPVVTKExtensionsDefaultModule.h"
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class vtkInformationStringKey;

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT IncomingTrackManipulation : public vtkPolyDataAlgorithm
{
public:
	vtkTypeMacro(IncomingTrackManipulation, vtkPolyDataAlgorithm);
	void PrintSelf(ostream& os, vtkIndent indent);

	static IncomingTrackManipulation *New();
	static vtkInformationStringKey *DESCRIPTIVE_NAME();

	vtkGetMacro(Momentum, double);
	void SetMomentum(double momentum);

	vtkGetMacro(ScatteringAngle, double);
	void SetScatteringAngle(double scatteringAngle);

	vtkGetMacro(DOCA, double);
	void SetDOCA(double doca);

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

	IncomingTrackManipulation();
	virtual ~IncomingTrackManipulation();

	double Momentum;
	double ScatteringAngle;
	double DOCA;
	double TrackLength;
	double XBounds[2];
	double YBounds[2];
	double ZBounds[2];

private:
	IncomingTrackManipulation(const IncomingTrackManipulation&);  // Not implemented.
	void operator=(const IncomingTrackManipulation&);  // Not implemented.
};

#endif


