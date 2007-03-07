/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkKappaSigmaThresholdCalculator.txx,v $
  Language:  C++
  Date:      $Date: 2005/01/13 15:36:46 $
  Version:   $Revision: 1.4 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef _itkKappaSigmaThresholdCalculator_txx
#define _itkKappaSigmaThresholdCalculator_txx

#include "itkKappaSigmaThresholdCalculator.h"

namespace itk
{


template<class TInputHistogram>
KappaSigmaThresholdCalculator<TInputHistogram>
::KappaSigmaThresholdCalculator() 
{
  m_Kappa = 2;
  m_NumberOfIterations = 2;
}


/*
 * Compute Otsu's thresholds
 */                    
template<class TInputHistogram>
void
KappaSigmaThresholdCalculator<TInputHistogram>
::GenerateData()
{

  typename TInputHistogram::ConstPointer histogram = this->GetInputHistogram();

  // TODO: as an improvement, the class could accept multi-dimensional histograms
  // and the user could specify the dimension to apply the algorithm to.
  if (histogram->GetSize().GetSizeDimension() != 1)
    {
    itkExceptionMacro(<<"Histogram must be 1-dimensional.");
    }

  if (histogram->GetTotalFrequency() == 0)
    {
    itkExceptionMacro(<<"Histogram is empty.");
    }

  // init the values
  MeasurementType threshold = histogram->GetMeasurementVector( histogram->Size() - 1 )[0]; // use all the pixels to begin
  // to avoid itertion over all the histogram at each iteration, and avoid testing that the value is
  // smaller than the threshold for all values in the histogram
  unsigned int startingIndex = histogram->Size() - 1;

  for( unsigned int iteration = 0; iteration < m_NumberOfIterations ; iteration++ )
    {
    
    // decrease the starting index to find the right starting index
    while( histogram->GetMeasurementVector( startingIndex )[0] > threshold )
      {
      startingIndex--;
      }
    
    // compute the mean
    double count = 0;
    double mean = 0;
    for( unsigned long i=0; i<=startingIndex; i++)
      {
      const MeasurementType & v = histogram->GetMeasurementVector( i )[0];
      const double & freq = histogram->GetFrequency( i );
      if( v <= threshold )
        {
        mean += v * freq;
        count += freq;
        }
      }
    mean = mean / count;
    
    // compute sigma
    double sigma = 0;
    for( unsigned long i=0; i<=startingIndex; i++)
      {
      const MeasurementType & v = histogram->GetMeasurementVector( i )[0];
      const double & freq = histogram->GetFrequency( i );
      if( v <= threshold )
        {
        sigma += vnl_math_sqr( v - mean ) * freq;
        }
      }
     sigma = vcl_sqrt( sigma / ( count - 1) );
   
    
    // compute the threshold for the next iteration
    MeasurementType newThreshold = mean + m_Kappa * sigma;
    // I don't know why, but without the casts, the following test is always false
    if( (float)newThreshold == (float)threshold )
      {
      // no need to continue - the threshold is the same and will produce the same result
      break;
      }
    threshold = newThreshold;

//     std::cout << "ratio: " << count/histogram->GetTotalFrequency() << "  mean: " << mean << "  sigma: " << sigma << "  threshold: " << threshold+0.0 << std::endl;
    }

  m_Output = threshold;

}

template<class TInputHistogram>
void
KappaSigmaThresholdCalculator<TInputHistogram>
::PrintSelf(std::ostream& os, Indent indent) const
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Output: " << static_cast<typename NumericTraits<MeasurementType>::PrintType>(m_Output) << std::endl;
  os << indent << "Kappa: " << m_Kappa << std::endl;
  os << indent << "NumberOfIterations: " << m_NumberOfIterations << std::endl;

}

} // end namespace itk

#endif
