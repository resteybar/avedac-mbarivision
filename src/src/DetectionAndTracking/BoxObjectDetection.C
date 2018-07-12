/*
 * BoxObjectDetection.C
 *
 *  Created on: Jul 2, 2018
 *      Author: mochoa
 */

#include "Component/OptionManager.H"
#include "Component/ParamClient.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/FilterOps.H"
#include "Image/PixelsTypes.H"
#include "Util/StringConversions.H"

class ModelParamBase;
class DetectionParameters;

#include "BoxObjectDetection.h"



BoxObjectDetection::BoxObjectDetection(OptionManager& mgr,
      const std::string& descrName,
      const std::string& tagName)
      : ModelComponent(mgr, descrName, tagName)
{

}
void BoxObjectDetection::start1(){

}
BoxObjectDetection::~BoxObjectDetection()
{}

void BoxObjectDetection::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status){

}
std::list<BitObject> BoxObjectDetection::run(
    nub::soft_ref<MbariResultViewer> rv,
    const std::list<Rectangle> &rec,
    const Image< PixRGB<byte> > &segmentInImg)
{
    DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;
    std::list<BitObject> bosFiltered;
    std::list<BitObject> bosUnfiltered;
    std::list<Rectangle>::const_iterator iter = rec.begin();
    int minArea = p.itsMinEventArea;
    int maxArea = p.itsMaxEventArea;

    //go through each winner and extract salient objects
    while (iter != rec.end()) {

		//list of boxes
		//replace with our box
		Rectangle region = (*iter); /*= iter.getBoundingBox();*/

		// Classifier helpful, soo don't use this for now
		// if a very interesting object, override the min event area
		// may be important in class probability
//            if (winner.sv > .004F) //Change this for rec
//                minArea = 1;

		// get the region used for searching for a match based on the foa region
//		LINFO("Extracting bit objects from frame %d winning point %d %d/region %s minSize %d maxSize %d segment dims %dx%d", \
//			   (*iter).getFrameNum(), rec.p.i, rec.p.j, minArea,
//				maxArea);

		//check if this list is empty
		Point2D<int> unusedSeed;
		std::list<BitObject> sobjsKeep = extractBitObjects(segmentInImg, unusedSeed, region, region, minArea, maxArea);
        // add to the list
        bosUnfiltered.splice(bosUnfiltered.begin(), sobjsKeep);

        //if list its empty we gonna take the rectangle or box
        if(rec.empty()){
			Image<byte> foamask;
			BitObject bo;
			bo.reset(makeBinary(foamask,byte(1),byte(1),byte(1)));

			iter++;
        }// end while iter != winners.end()

    LINFO("Found %lu bitobject(s)", bosUnfiltered.size());

    bool found = false;
    int minSize = p.itsMinEventArea;
    if (p.itsRemoveOverlappingDetections) {
        LINFO("Removing overlapping detections");
        // loop until we find all non-overlapping objects starting with the smallest
        while (!bosUnfiltered.empty()) {

            std::list<BitObject>::iterator biter, siter, smallest;
            // find the smallest object
            smallest = bosUnfiltered.begin();
            for (siter = bosUnfiltered.begin(); siter != bosUnfiltered.end(); ++siter)
                if (siter->getArea() < minSize) {
                    minSize = siter->getArea();
                    smallest = siter;
                }

            // does the smallest object intersect with any of the already stored ones
            found = true;
            for (biter = bosFiltered.begin(); biter != bosFiltered.end(); ++biter) {
                if (smallest->isValid() && biter->isValid() && biter->doesIntersect(*smallest)) {
                    // no need to store intersecting objects -> get rid of smallest
                    // and look for the next smallest
                    bosUnfiltered.erase(smallest);
                    found = false;
                    break;
                }
            }

            if (found && smallest->isValid())
                bosFiltered.push_back(*smallest);
        }
    }
    else {
        std::list<BitObject>::iterator biter;
        for (biter = bosUnfiltered.begin(); biter != bosUnfiltered.end(); ++biter) {
            if (biter->isValid())
                bosFiltered.push_back(*biter);
        }
    }

    LINFO("Found total %lu objects", bosFiltered.size());
    return bosFiltered;
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
