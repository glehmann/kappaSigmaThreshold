/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkKappaSigmaThresholdImageFilter.h,v $
  Language:  C++
  Date:      $Date: 2006/04/05 13:59:37 $
  Version:   $Revision: 1.3 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkKappaSigmaThresholdImageFilter_h
#define __itkKappaSigmaThresholdImageFilter_h

#include "itkImageToImageFilter.h"
#include "itkKappaSigmaThresholdCalculator.h"
#include "itkScalarImageToHistogramGenerator.h"

namespace itk {

/** \class KappaSigmaThresholdImageFilter 
 * \brief Threshold an image using multiple Otsu Thresholds.
 *
 * This filter creates a labeled image that separates the input
 * image into various classes. The filter
 * computes the thresholds using the MaximumEntropyThresholdCalculator and
 * applies those thesholds to the input image using the
 * ThresholdLabelerImageFilter. The NumberOfHistogramBins and
 * NumberOfThresholds can be set
 * for the Calculator. The LabelOffset can be set
 * for the ThresholdLabelerImageFilter.
 *
 * \sa ScalarImageToHistogramGenerator
 * \sa MaximumEntropyThresholdCalculator
 * \sa ThresholdLabelerImageFilter
 * \ingroup IntensityImageFilters  Multithreaded
 */

template<class TInputImage, class TMaskImage=Image<unsigned char, TInputImage::ImageDimension>, class TOutputImage=TInputImage>
class ITK_EXPORT KappaSigmaThresholdImageFilter : 
    public ImageToImageFilter<TInputImage, TOutputImage>
{
public:
  /** Standard Self typedef */
  typedef KappaSigmaThresholdImageFilter Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>        Pointer;
  typedef SmartPointer<const Self>  ConstPointer;
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);  

  /** Runtime information support. */
  itkTypeMacro(KappaSigmaThresholdImageFilter, ImageToImageFilter);
  
  /** Standard image type within this class. */
  typedef TInputImage InputImageType;
  typedef TMaskImage MaskImageType;

  /** Image pixel value typedef. */
  typedef typename TInputImage::PixelType   InputPixelType;
  typedef typename TOutputImage::PixelType   OutputPixelType;
  typedef typename TMaskImage::PixelType   MaskPixelType;
  
  /** Image related typedefs. */
  typedef typename TInputImage::Pointer InputImagePointer;
  typedef typename TOutputImage::Pointer OutputImagePointer;
  typedef typename TMaskImage::Pointer MaskImagePointer;

  typedef typename TInputImage::SizeType  InputSizeType;
  typedef typename TInputImage::IndexType  InputIndexType;
  typedef typename TInputImage::RegionType InputImageRegionType;
  typedef typename TOutputImage::SizeType  OutputSizeType;
  typedef typename TOutputImage::IndexType  OutputIndexType;
  typedef typename TOutputImage::RegionType OutputImageRegionType;

  typedef itk::Statistics::ScalarImageToHistogramGenerator< 
                                           TInputImage > HistogramGeneratorType;
  typedef typename HistogramGeneratorType::HistogramType HistogramType;
  typedef KappaSigmaThresholdCalculator< HistogramType > CalculatorType;
  
  /** Image related typedefs. */
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension ) ;
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TOutputImage::ImageDimension ) ;

  /** Set the "outside" pixel value. The default value
   * NumericTraits<OutputPixelType>::Zero. */
  itkSetMacro(OutsideValue,OutputPixelType);

  /** Get the "outside" pixel value. */
  itkGetMacro(OutsideValue,OutputPixelType);

  /** Set the "inside" pixel value. The default value
   * NumericTraits<OutputPixelType>::max() */
  itkSetMacro(InsideValue,OutputPixelType);

  /** Get the "inside" pixel value. */
  itkGetMacro(InsideValue,OutputPixelType);

  /** Get the computed threshold. */
  itkGetMacro(Threshold,InputPixelType);

  itkSetMacro(MaskValue, MaskPixelType);
  itkGetMacro(MaskValue, MaskPixelType);

  itkSetMacro(SigmaFactor, double);
  itkGetMacro(SigmaFactor, double);

  itkSetMacro(NumberOfIterations, unsigned int);
  itkGetMacro(NumberOfIterations, unsigned int);

  /** Set/Get the number of histogram bins. Default is 128. */
  itkSetClampMacro( NumberOfHistogramBins, unsigned long, 1, NumericTraits<unsigned long>::max() );
  itkGetConstMacro( NumberOfHistogramBins, unsigned long );

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(OutputComparableCheck,
    (Concept::Comparable<OutputPixelType>));
  itkConceptMacro(OutputOStreamWritableCheck,
    (Concept::OStreamWritable<OutputPixelType>));
  /** End concept checking */
#endif

   /** Set the mask image */
  void SetMaskImage(MaskImageType *input)
     {
     // Process object is not const-correct so the const casting is required.
     this->SetNthInput( 1, const_cast<MaskImageType *>(input) );
     }

  /** Get the mask image */
  MaskImageType * GetMaskImage()
    {
    return static_cast<MaskImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(1)));
    }


   /** Set the input image */
  void SetInput1(TInputImage *input)
     {
     this->SetInput( input );
     }

   /** Set the marker image */
  void SetInput2(MaskImageType *input)
     {
     this->SetMaskImage( input );
     }

protected:
  KappaSigmaThresholdImageFilter();
  ~KappaSigmaThresholdImageFilter(){};
  void PrintSelf(std::ostream& os, Indent indent) const;

  void GenerateInputRequestedRegion();
  void GenerateData ();

private:
  KappaSigmaThresholdImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  MaskPixelType m_MaskValue;
  double m_SigmaFactor;
  unsigned int m_NumberOfIterations;
  InputPixelType      m_Threshold;
  OutputPixelType     m_InsideValue;
  OutputPixelType     m_OutsideValue;
  unsigned long       m_NumberOfHistogramBins;


} ; // end of class

} // end namespace itk
  
#ifndef ITK_MANUAL_INSTANTIATION
#include "itkKappaSigmaThresholdImageFilter.txx"
#endif

#endif
