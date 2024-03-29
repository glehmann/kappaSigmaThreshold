#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkSimpleFilterWatcher.h"

#include "itkKappaSigmaThresholdImageFilter.h"


int main(int argc, char * argv[])
{

  if( argc != 5 )
    {
    std::cerr << "usage: " << argv[0]
              << " input output sigmaFactor numberOfIterations" << std::endl;
    exit(1);
    }

  const int dim = 3;
  
  typedef unsigned char PType;
  typedef itk::Image< PType, dim > IType;

  typedef itk::ImageFileReader< IType > ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );

  typedef itk::KappaSigmaThresholdImageFilter< IType > FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetInput( reader->GetOutput() );
  filter->SetKappa( atof(argv[3]) );
  filter->SetNumberOfIterations( atoi(argv[4]) );
  filter->SetNumberOfHistogramBins( 256 );

  itk::SimpleFilterWatcher watcher(filter, "filter");

  typedef itk::ImageFileWriter< IType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( filter->GetOutput() );
  writer->SetFileName( argv[2] );
  writer->Update();

  return 0;
}

