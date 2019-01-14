#include "FilterDriftTubes.h"
#include "ThroughTrackEventViewer.h"
#include <QFileDialog>

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
#include "vtkTubeFilter.h"

#include "mtlib-types/scanner/GeometryInfoIOBin.h"

/*----------------------------------------------------------------------------*/
vtkStandardNewMacro(FilterDriftTubes);
vtkInformationKeyMacro(FilterDriftTubes, DESCRIPTIVE_NAME, String);

/*----------------------------------------------------------------------------*/
FilterDriftTubes::FilterDriftTubes()
{
	this->SetNumberOfInputPorts(1);
	this->SetNumberOfOutputPorts(2);

	this->Ambiguity = 10.0;
	this->Drift = 10.0;
	this->DriftTime = 500.0;
	this->Outlier = 0;

	this->TubeRange[0] = 0.0;
	this->TubeRange[1] = 576.0;
}

/*----------------------------------------------------------------------------*/
FilterDriftTubes::~FilterDriftTubes()
{
}

void FilterDriftTubes::LoadGeometryFile()
{
	QString fileName = QFileDialog::getOpenFileName(QWidget::find(0), QObject::tr("Open Geometry"), "/export/", QObject::tr("Geometry Files (*.bin)"));

	if (fileName.toStdString().length() > 1)
	{
		mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fileName.toStdString());
		GeometryInfo_ = GeometryInfo;
	}
}

void FilterDriftTubes::SetOutliers(int outlier)
{
	this->Outlier = outlier;
}

void FilterDriftTubes::SetAmbiguity(int ambiguity)
{
	this->Ambiguity = ambiguity;
}

void FilterDriftTubes::SetDrift(double drift)
{
	this->Drift = drift;
}

void FilterDriftTubes::SetDriftTime(double drifttime)
{
	this->DriftTime = drifttime;
}

void FilterDriftTubes::SetTubeRange(double first, double last)
{
	this->TubeRange[0] = first;
	this->TubeRange[1] = last;
}

bool FilterDriftTubes::isTubeWithinRange(int tube_index)
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
int FilterDriftTubes::RequestData(
		vtkInformation *request,
		vtkInformationVector **inputVector,
		vtkInformationVector *outputVector)
{
	vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
	vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

	vtkInformation *outInfoFilteredDriftTubes = outputVector->GetInformationObject(0);
	outInfoFilteredDriftTubes->Set(FilterDriftTubes::DESCRIPTIVE_NAME(), "Filtered Drift Tubes");
	vtkPolyData *outputFilteredDriftTubes = vtkPolyData::SafeDownCast(outInfoFilteredDriftTubes->Get(vtkDataObject::DATA_OBJECT()));

	vtkInformation *outInfoFilteredOutliers = outputVector->GetInformationObject(1);
	outInfoFilteredOutliers->Set(FilterDriftTubes::DESCRIPTIVE_NAME(), "Filtered Outliers");
	vtkPolyData *outputFilteredOutliers = vtkPolyData::SafeDownCast(outInfoFilteredOutliers->Get(vtkDataObject::DATA_OBJECT()));

	if (input->GetCellData()->GetNumberOfArrays() > 0)
	{
		vtkSmartPointer<vtkAppendPolyData> filteredDriftTubesAppend = vtkSmartPointer<vtkAppendPolyData>::New();
		vtkSmartPointer<vtkAppendPolyData> filteredOutliersAppend = vtkSmartPointer<vtkAppendPolyData>::New();
		vtkSmartPointer<vtkAppendPolyData> originalDriftTubesAppend = vtkSmartPointer<vtkAppendPolyData>::New();

		vtkIntArray*    Ambiguity = vtkIntArray::SafeDownCast(input->GetCellData()->GetArray("Ambiguity"));
		vtkDoubleArray* DriftRadius = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Drift Radius"));
		vtkDoubleArray* DriftTime = vtkDoubleArray::SafeDownCast(input->GetCellData()->GetArray("Drift Time"));
		vtkIntArray*    Outlier = vtkIntArray::SafeDownCast(input->GetCellData()->GetArray("Outlier"));
		vtkIntArray*    TubeIndex = vtkIntArray::SafeDownCast(input->GetCellData()->GetArray("Tube Index"));

		for (int index = 0; index < input->GetNumberOfCells(); index++)
		{
			if ((Ambiguity->GetValue(index) < this->Ambiguity) &
				(DriftRadius->GetValue(index) < this->Drift) &
				(DriftTime->GetValue(index) < this->DriftTime) &
				(Outlier->GetValue(index) == this->Outlier) &
				(isTubeWithinRange(TubeIndex->GetValue(index)) == true))
			{
				vtkSmartPointer<vtkDoubleArray> ambiguity = vtkSmartPointer<vtkDoubleArray>::New();
				ambiguity->SetName("ambiguity");
				ambiguity->InsertNextValue(Ambiguity->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> driftRadius = vtkSmartPointer<vtkDoubleArray>::New();
				driftRadius->SetName("Drift Radius");
				driftRadius->InsertNextValue(DriftRadius->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> driftTime = vtkSmartPointer<vtkDoubleArray>::New();
				driftTime->SetName("Drift Time");
				driftTime->InsertNextValue(DriftTime->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> outlier = vtkSmartPointer<vtkDoubleArray>::New();
				outlier->SetName("Outlier");
				outlier->InsertNextValue(Outlier->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> tubeIndex = vtkSmartPointer<vtkDoubleArray>::New();
				tubeIndex->SetName("Tube Index");
				tubeIndex->InsertNextValue(TubeIndex->GetValue(index));

				vtkSmartPointer<vtkLineSource> filteredTube = vtkSmartPointer<vtkLineSource>::New();
				filteredTube->SetPoint1(this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointX(),
										this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointY(),
										this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointZ());

				filteredTube->SetPoint2(this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetX(),
										this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetY(),
										this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetZ());

				filteredTube->Update();

				vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
				tube_filter->SetInputConnection(filteredTube->GetOutputPort());
				tube_filter->SetRadius(2.54);
				tube_filter->SetNumberOfSides(50);
				tube_filter->Update();

				vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
				polyData = filteredTube->GetOutput();
				polyData->GetCellData()->AddArray(ambiguity);
				polyData->GetCellData()->AddArray(driftRadius);
				polyData->GetCellData()->AddArray(driftTime);
				polyData->GetCellData()->AddArray(outlier);
				polyData->GetCellData()->AddArray(tubeIndex);

				filteredDriftTubesAppend->AddInputData(polyData);
				filteredDriftTubesAppend->AddInputData(tube_filter->GetOutput());
			}

			else
			{
				vtkSmartPointer<vtkDoubleArray> ambiguity = vtkSmartPointer<vtkDoubleArray>::New();
				ambiguity->SetName("ambiguity");
				ambiguity->InsertNextValue(Ambiguity->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> driftRadius = vtkSmartPointer<vtkDoubleArray>::New();
				driftRadius->SetName("Drift Radius");
				driftRadius->InsertNextValue(DriftRadius->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> driftTime = vtkSmartPointer<vtkDoubleArray>::New();
				driftTime->SetName("Drift Time");
				driftTime->InsertNextValue(DriftTime->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> outlier = vtkSmartPointer<vtkDoubleArray>::New();
				outlier->SetName("Outlier");
				outlier->InsertNextValue(Outlier->GetValue(index));

				vtkSmartPointer<vtkDoubleArray> tubeIndex = vtkSmartPointer<vtkDoubleArray>::New();
				tubeIndex->SetName("Tube Index");
				tubeIndex->InsertNextValue(TubeIndex->GetValue(index));

				vtkSmartPointer<vtkLineSource> filteredTube = vtkSmartPointer<vtkLineSource>::New();
				filteredTube->SetPoint1(this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointX(),
						this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointY(),
						this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetPointZ());

				filteredTube->SetPoint2(this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetX(),
						this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetY(),
						this->GeometryInfo_->GetDriftTube(int64_t(TubeIndex->GetValue(index)))->GetExtents().GetTip().GetZ());

				filteredTube->Update();

				vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
				tube_filter->SetInputConnection(filteredTube->GetOutputPort());
				tube_filter->SetRadius(2.54);
				tube_filter->SetNumberOfSides(50);
				tube_filter->Update();

				vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
				polyData = filteredTube->GetOutput();
				polyData->GetCellData()->AddArray(ambiguity);
				polyData->GetCellData()->AddArray(driftRadius);
				polyData->GetCellData()->AddArray(driftTime);
				polyData->GetCellData()->AddArray(outlier);
				polyData->GetCellData()->AddArray(tubeIndex);

				filteredOutliersAppend->AddInputData(polyData);
				filteredOutliersAppend->AddInputData(tube_filter->GetOutput());
			}
		}

		filteredDriftTubesAppend->Update();
		filteredOutliersAppend->Update();

		if (filteredDriftTubesAppend->GetNumberOfInputConnections(0) > 0)
		{
			outputFilteredDriftTubes->ShallowCopy(filteredDriftTubesAppend->GetOutput());
			outputFilteredOutliers->ShallowCopy(filteredOutliersAppend->GetOutput());
		}

		return 1;
	}

	else
	{
		return 1;
	}
}

void FilterDriftTubes::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os, indent);
}
