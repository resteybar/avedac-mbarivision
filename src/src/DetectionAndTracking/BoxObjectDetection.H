/*
 * BoxObjectDetection.h
 *
 *  Created on: Jul 2, 2018
 *      Author: mochoa
 */

/*!@file BOXOBJECTDETECTION.H used for object detection*/
#ifndef SRC_DETECTIONANDTRACKING_BOXOBJECTDETECTION_H_
#define SRC_DETECTIONANDTRACKING_BOXOBJECTDETECTION_H_

#include "Component/ModelManager.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Data/Winner.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/ImageCache.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Media/FrameSeries.H"
#include "Media/MbariResultViewer.H"
#include "Image/Rectangle.H" //New rectangle

#include <map>
#include <vector>
#include <list>

class BoxObjectDetection : public ModelComponent //Ask if we need to inheritance ModelComponent
{
public:
	BoxObjectDetection(OptionManager& mgr, const std::string& descrName = "mbarivision box object detector",
            const std::string& tagName = "box object detector");
	//! destructor
	virtual ~BoxObjectDetection();

	 //! init
	void init(nub::soft_ref<InputFrameSeries> ifs);

	  //! Overload so that we can reconfigure when our params get changed
	virtual void paramChanged(ModelParamBase* const param,
	                            const bool valueChanged,
	                            ParamClient::ChangeStatus* status);

	  //! BoxObjectDetection
	std::list<BitObject> run(nub::soft_ref<MbariResultViewer> rv,
					const std::list<Rectangle> &rec,
					const Image< PixRGB<byte> > &segmentInImg);

	protected:

	 //! overload start1()
	 virtual void start1();

	private:
};

#endif /* SRC_DETECTIONANDTRACKING_BOXOBJECTDETECTION_H_ */

