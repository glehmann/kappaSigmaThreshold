/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkKappaSigmaThresholdImageFilter.txx,v $
  Language:  C++
  Date:      $Date: 2006/03/15 01:57:09 $
  Version:   $Revision: 1.8 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef _itkKappaSigmaThresholdImageFilter_txx
#define _itkKappaSigmaThresholdImageFilter_txx

#include "itkKappaSigmaThresholdImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkProgressAccumulator.h"

namespace itk {

template<class TInputImage, class TMaskImage, class TOutputImage>
KappaSigmaThresholdImageFilter<TInputImage, TMaskImage, TOutputImage>
::KappaSigmaThresholdImageFilter()
{
  m_OutsideValue   = NumericTraits<OutputPixelType>::Zero;
  m_InsideValue    = NumericTraits<OutputPixelType>::max();
  m_Threshold      = NumericTraits<InputPixelType>::Zero;
  m_Kappa = 2;
  m_NumberOfIterations = 2;
  m_MaskValue = NumericTraits<MaskPixelType>::max();
  m_NumberOfHistogramBins = 128;
}

template<class TInputImage, class TMaskImage, class TOutputImage>
void
KappaSigmaThresholdImageFilter<TInputImage, TMaskImage, TOutputImage>
::GenerateData()
{
  typename ProgressAccumulator::Pointer progress = ProgressAccumulator::New();
  progress->SetMiniPipelineFilter(this);

  // Create a histogram of the image intensities
  typename HistogramGeneratorType::Pointer histogramGenerator = HistogramGeneratorType::New();
  histogramGenerator->SetInput(  this->GetInput()  );
  histogramGenerator->SetMaskImage( this->GetMaskImage()  );
  histogramGenerator->SetMaskValue( m_MaskValue );
  histogramGenerator->SetNumberOfBins( m_NumberOfHistogramBins );
  // progress->RegisterInternalFilter(histogramGenerator,.5f);
  histogramGenerator->Compute();

  // Compute the Threshold for the input image
  typename CalculatorType::Pointer thresholdCalculator = CalculatorType::New();
  thresholdCalculator->SetInputHistogram( histogramGenerator->GetOutput() );
  thresholdCalculator->SetKappa( m_Kappa );
  thresholdCalculator->SetNumberOfIterations( m_NumberOfIterations );
  thresholdCalculator->Update();

  m_Threshold = static_cast< InputPixelType >( thresholdCalculator->GetOutput() );

  typename BinaryThresholdImageFilter<TInputImage,TOutputImage>::Pointer threshold = 
    BinaryThresholdImageFilter<TInputImage,TOutputImage>::New();;
  
//   progress->RegisterInternalFilter(threshold,.5f);
  // nor histogramGenerator or thresholdCalculator can report their progress so
  // pretend that all the work is done by threshold.
  progress->RegisterInternalFilter(threshold, 1.0f);
  threshold->GraftOutput (this->GetOutput());
  threshold->SetInput (this->GetInput());
  // the value we get is in the background, and so must not be considered
  // as foreground by the threshold filter
  threshold->SetUpperThreshold( m_Threshold );
  threshold->SetOutsideValue (m_InsideValue);
  threshold->SetInsideValue (m_OutsideValue);
  threshold->Update();

  this->GraftOutput(threshold->GetOutput());
}

template<class TInputImage, class TMaskImage, class TOutputImage>
void
KappaSigmaThresholdImageFilter<TInputImage, TMaskImage, TOutputImage>
::GenerateInputRequestedRegion()
{
  const_cast<TInputImage *>(this->GetInput())->SetRequestedRegionToLargestPossibleRegion();
}

template<class TInputImage, class TMaskImage, class TOutputImage>
void 
KappaSigmaThresholdImageFilter<TInputImage, TMaskImage, TOutputImage>
::PrintSelf(std::ostream& os, Indent indent) const
{
  Superclass::PrintSelf(os,indent);

  os << indent << "Threshold: " << static_cast<typename NumericTraits<InputPixelType>::PrintType>(m_Threshold) << std::endl;
  os << indent << "MaskValue: " << static_cast<typename NumericTraits<MaskPixelType>::PrintType>(m_MaskValue) << std::endl;
  os << indent << "Kappa: " << m_Kappa << std::endl;
  os << indent << "NumberOfIterations: " << m_NumberOfIterations << std::endl;
}


}// end namespace itk
#endif
