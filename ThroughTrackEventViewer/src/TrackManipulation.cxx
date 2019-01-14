#include "TrackManipulation.h"

#include "vtkObjectFactory.h"
#include "vtkAppendPolyData.h"
#include "vtkPolyData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationStringKey.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkSmartPointer.h"
#include "vtkDoubleArray.h"
#include "vtkCellData.h"
#include "vtkCell.h"
#include "vtkMath.h"
#include "vtkDataArray.h"

#include "vtkSphereSource.h"
#include "vtkLineSource.h"

/*----------------------------------------------------------------------------*/
vtkStandardNewMacro(TrackManipulation);
vtkInformationKeyMacro(TrackManipulation, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
TrackManipulation::TrackManipulation()
{
	this->SetNumberOfInputPorts(1);
	this->SetNumberOfOutputPorts(2);

	this->Momentum = 0.0;
	this->ScatteringAngle = 0.002;
	this->TrackLength = 250.0;
	this->DOCA = 3.0;

	this->XBounds[0] = 0.0;
	this->XBounds[1] = 120.0;

	this->YBounds[0] = 0.0;
	this->YBounds[1] = 120.0;

	this->ZBounds[0] = 90.0;
	this->ZBounds[1] = 180.0;
}

/*----------------------------------------------------------------------------*/
TrackManipulation::~TrackManipulation()
{
}

void TrackManipulation::Clear()
{
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

	vtk_poly_data = polyData;
}

void TrackManipulation::SetMomentum(double momentum)
{
	this->Momentum = momentum;
}

void TrackManipulation::SetTrackLength(double length)
{
	this->TrackLength = length;
}

void TrackManipulation::SetScatteringAngle(double scatteringAngle)
{
	this->ScatteringAngle = scatteringAngle;
}

void TrackManipulation::SetDOCA(double doca)
{
	this->DOCA = doca;
}

void TrackManipulation::SetXBounds(double x0, double x1)
{
	this->XBounds[0] = x0;
	this->XBounds[1] = x1;
}

void TrackManipulation::SetYBounds(double y0, double y1)
{
	this->YBounds[0] = y0;
	this->YBounds[1] = y1;
}

void TrackManipulation::SetZBounds(double z0, double z1)
{
	this->ZBounds[0] = z0;
	this->ZBounds[1] = z1;
}

bool TrackManipulation::isPoCAWithinBounds(double x, double y, double z)
{
#ifndef NDEBUG
	std::cout << "\n" << std::endl;
	std::cout << "x: " << x <<
				 " , y: " << y <<
				 " , z: " << z << std::endl;
#endif

	double xmin = this->XBounds[0];
	double xmax = this->XBounds[1];
	double ymin = this->YBounds[0];
	double ymax = this->YBounds[1];
	double zmin = this->ZBounds[0];
	double zmax = this->ZBounds[1];

#ifndef NDEBUG
	std::cout << "Bounds: " << xmin << ", "
							<< xmax << ", "
							<< ymin << ", "
							<< ymax << ", "
							<< zmin << ", "
							<< zmax << std::endl;
#endif

	bool x_boundary;
	bool y_boundary;
	bool z_boundary;

	if ((x >= xmin) & (x <= xmax))
	{
		x_boundary = true;
	}

	if ((y >= ymin) & (y <= ymax))
	{
		y_boundary = true;
	}

	if ((z >= zmin) & (z <= zmax))
	{
		z_boundary = true;
	}

	if ((x_boundary == true) & (y_boundary == true) & (z_boundary == true))
	{
		return true;
	}

	else
	{
		return false;
	}
}

/*----------------------------------------------------------------------------*/
int TrackManipulation::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
	vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

	vtkInformation *outInfo0 = outputVector->GetInformationObject(0);
	outInfo0->Set(TrackManipulation::DESCRIPTIVE_NAME(), "Filtered Particle Trajectory");
	vtkPolyData *output0 = vtkPolyData::SafeDownCast(outInfo0->Get(vtkDataObject::DATA_OBJECT()));

	vtkInformation *outInfo1 = outputVector->GetInformationObject(1);
	outInfo1->Set(TrackManipulation::DESCRIPTIVE_NAME(), "Filtered PoCA Points");
	vtkPolyData *output1 = vtkPolyData::SafeDownCast(outInfo1->Get(vtkDataObject::DATA_OBJECT()));

	if (input->GetCellData()->GetNumberOfArrays() > 0)
	{
		vtkSmartPointer<vtkAppendPolyData> appendPolyData0 = vtkSmartPointer<vtkAppendPolyData>::New();
		vtkSmartPointer<vtkAppendPolyData> appendPolyData1 = vtkSmartPointer<vtkAppendPolyData>::New();

		vtkDoubleArray* Momentum = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Momentum"));
		vtkDoubleArray* ScatAngle = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Scattering Angle"));
		vtkDoubleArray* InTrLength = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Incoming Track Length"));
		vtkDoubleArray* OutTrLength = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Outgoing Track Length"));
		vtkDoubleArray* Doca = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("DOCA"));
		vtkDoubleArray* XCom = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("PoCA X Coordinate"));
		vtkDoubleArray* YCom = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("PoCA Y Coordinate"));
		vtkDoubleArray* ZCom = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("PoCA Z Coordinate"));

		for (int index = 0; index < input->GetNumberOfCells(); index++)
		{
			if ((Momentum->GetValue(index) > this->Momentum) &
				(ScatAngle->GetValue(index) > this->ScatteringAngle) &
				(Doca->GetValue(index) > this->DOCA) &
				(InTrLength->GetValue(index) < this->TrackLength) &
				(OutTrLength->GetValue(index) < this->TrackLength) &
				(isPoCAWithinBounds(XCom->GetValue(index), YCom->GetValue(index), ZCom->GetValue(index))  == true ))
			{
				vtkSmartPointer<vtkDoubleArray> momentum = vtkSmartPointer<vtkDoubleArray>::New();
				momentum->SetName("Momentum");
				momentum->InsertNextValue(Momentum->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> scatteringAngle = vtkSmartPointer<vtkDoubleArray>::New();
				scatteringAngle->SetName("Scattering Angle");
				scatteringAngle->InsertNextValue(ScatAngle->GetValue(index));

				vtkCell* cell = input->GetCell(index);

				vtkSmartPointer<vtkLineSource> trajectory = vtkSmartPointer<vtkLineSource>::New();
				trajectory->SetPoint1(cell->GetPoints()->GetPoint(0));
				trajectory->SetPoint2(cell->GetPoints()->GetPoint(1));
				trajectory->Update();

				vtkSmartPointer<vtkSphereSource> PoCA = vtkSmartPointer<vtkSphereSource>::New();
				PoCA->SetCenter(XCom->GetValue(index), YCom->GetValue(index), ZCom->GetValue(index));
				PoCA->SetRadius(2);
				PoCA->Update();

				vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
				polyData = trajectory->GetOutput();
				polyData->GetCellData()->AddArray(momentum);
				polyData->GetCellData()->AddArray(scatteringAngle);

				appendPolyData0->AddInputData(polyData);
				appendPolyData1->AddInputData(PoCA->GetOutput());
			}
		}

		appendPolyData0->Update();
		appendPolyData1->Update();

		if (appendPolyData0->GetNumberOfInputConnections(0) > 0)
		{
			output0->ShallowCopy(appendPolyData0->GetOutput());
			output1->ShallowCopy(appendPolyData1->GetOutput());
		}

		return 1;
	}

	else
	{
		return 1;
	}
}

void TrackManipulation::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
