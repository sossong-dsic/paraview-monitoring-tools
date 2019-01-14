#include "LoadGeometryFile.h"

#include "vtkObjectFactory.h"
#include "vtkAppendPolyData.h"
#include "vtkPolyData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkSmartPointer.h"
#include "vtkDoubleArray.h"
#include "vtkCellData.h"
#include "vtkPointData.h"
#include "vtkCell.h"
#include "vtkDataArray.h"
#include "vtkTubeFilter.h"
#include <cstring>

#include "mtlib-types/scanner/GeometryInfoIOBin.h"

#include "vtkLineSource.h"

/*----------------------------------------------------------------------------*/
vtkStandardNewMacro(LoadGeometryFile);

/*----------------------------------------------------------------------------*/
LoadGeometryFile::LoadGeometryFile()
{
	this->SetNumberOfInputPorts(1);
	this->SetNumberOfOutputPorts(1);

	this->FileName = "Please Enter A Binary File Name";

	this->TubeRange[0] = 0.0;
	this->TubeRange[1] = 576.0;
}

/*----------------------------------------------------------------------------*/
LoadGeometryFile::~LoadGeometryFile()
{
}

void LoadGeometryFile::SetFileName(std::string filename)
{
	this->FileName = filename;

	if (this->FileName.length() > 1)
	{
		mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(this->FileName);
		GeometryInfo_ = GeometryInfo;
	}
}

void LoadGeometryFile::SetTubeRange(double first, double last)
{
	this->TubeRange[0] = first;
	this->TubeRange[1] = last;
}

bool LoadGeometryFile::isTubeWithinRange(int tube_index)
{
	if ((double(tube_index) >= this->TubeRange[0]) & (double(tube_index) <= this->TubeRange[1]))
	{
		return true;
	}

	else
	{
		return false;
	}
}

/*----------------------------------------------------------------------------*/
int LoadGeometryFile::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
	vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

	vtkInformation *outInfo = outputVector->GetInformationObject(0);
	vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

	vtkSmartPointer<vtkAppendPolyData> appendPolyData = vtkSmartPointer<vtkAppendPolyData>::New();

	if (input->GetCellData()->GetNumberOfArrays() > 0)
	{
		vtkIntArray* TubeIndex = vtkIntArray::SafeDownCast(input->GetCellData()->GetArray("Tube Index"));

		for (int index = 0; index < input->GetNumberOfCells(); index++)
		{
			if (isTubeWithinRange(TubeIndex->GetValue(index)) == true)
			{
				vtkSmartPointer<vtkLineSource> tube = vtkSmartPointer<vtkLineSource>::New();

				tube->SetPoint1(GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointX(),
								GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointY(),
								GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointZ());

				tube->SetPoint2(GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetX(),
								GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetY(),
								GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetZ());

				tube->Update();

				vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
				tube_filter->SetInputConnection(tube->GetOutputPort());
				tube_filter->SetRadius(2.54);
				tube_filter->SetNumberOfSides(50);
				tube_filter->Update();

				appendPolyData->AddInputData(tube->GetOutput());
				appendPolyData->AddInputData(tube_filter->GetOutput());
				appendPolyData->Update();
			}
		}
		appendPolyData->Update();
	}


	if (appendPolyData->GetNumberOfInputConnections(0) > 0)
	{
		output->ShallowCopy(appendPolyData->GetOutput());
	}

	return 1;
}

void LoadGeometryFile::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
