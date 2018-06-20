/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance 
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleft/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This is a program to automate detection and tracking of events in underwater 
 * video. This is based on modified version from Dirk Walther's 
 * work that originated at the 2002 Workshop  Neuromorphic Engineering 
 * in Telluride, CO, USA. 
 * 
 * This code requires the The iLab Neuromorphic Vision C++ Toolkit developed
 * by the University of Southern California (USC) and the iLab at USC. 
 * See http://iLab.usc.edu for information about this project. 
 *  
 * This work would not be possible without the generous support of the 
 * David and Lucile Packard Foundation
 */

#include "Image/OpenCVUtil.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/Rectangle.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/colorDefs.H"
#include "Image/Geometry2D.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "DetectionAndTracking/MbariFunctions.H"

#include <algorithm>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
// ###### VisualEventSet
// ######################################################################
VisualEventSet::VisualEventSet(const DetectionParameters &parameters,
                               const string& fileName)
  : startframe(-1),
    endframe(-1),
    itsFileName(fileName),
    itsDetectionParms(parameters)
{

}

// ######################################################################
VisualEventSet::~VisualEventSet()
{
  reset();
}

// ######################################################################
VisualEventSet::VisualEventSet(istream& is)
{
  readFromStream(is);
}

void VisualEventSet::readHeaderFromStream(istream& is)
{
  is >> itsFileName;
  is >> itsDetectionParms.itsMaxDist;
  is >> itsDetectionParms.itsMaxCost;
  is >> itsDetectionParms.itsMinEventFrames;
  is >> itsDetectionParms.itsMinEventArea;
  is >> startframe;
  is >> endframe;
}

void VisualEventSet::writeHeaderToStream(ostream& os)
{
  os << itsFileName << "\n";
  os << itsDetectionParms.itsMaxDist << " "
     << itsDetectionParms.itsMaxCost << " "
     << itsDetectionParms.itsMinEventFrames << " "
     << itsDetectionParms.itsMinEventArea << "\n";
  os << startframe << ' ' << endframe << '\n';

  os << "\n";
}

// ######################################################################
void VisualEventSet::writeToStream(ostream& os)
{
  list<VisualEvent *>::iterator currEvent;

  os << itsFileName << "\n";
  os << itsDetectionParms.itsMaxDist << " "
     << itsDetectionParms.itsMaxCost << " "
     << itsDetectionParms.itsMinEventFrames << " "
     << itsDetectionParms.itsMinEventArea << "\n";
  os << startframe << ' ' << endframe << '\n';

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    (*currEvent)->writeToStream(os);

  os << "\n";
}


// ######################################################################
void VisualEventSet::readFromStream(istream& is)
{
  is >> itsFileName; LINFO("filename: %s",itsFileName.data());
  is >> itsDetectionParms.itsMaxDist;
  is >> itsDetectionParms.itsMaxCost;
  is >> itsDetectionParms.itsMinEventFrames;
  is >> itsDetectionParms.itsMinEventArea;
  is >> startframe;
  is >> endframe;

  itsEvents.clear();

  while (is.eof() != false)
    itsEvents.push_back(new VisualEvent(is));
}

// ######################################################################
void VisualEventSet::writePositions(ostream& os) const
{
  list<VisualEvent *>::const_iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    (*currEvent)->writePositions(os);
}
// ######################################################################
void VisualEventSet::insert(VisualEvent *event)
{
  itsEvents.push_back(event);
}
// ######################################################################
void VisualEventSet::runKalmanHoughTracker(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *currEvent,
                                           const BayesClassifier &bayesClassifier,
                                           FeatureCollection& features,
                                           ImageData& imgData)
{
 bool found = false;
 Token evtToken;

  // prefer the Kalman tracker, and fall back to the Hough tracker
  if (!runKalmanTracker(currEvent, bayesClassifier, features, imgData, true)){
    evtToken = currEvent->getToken(currEvent->getEndFrame());

    // only use the Hough tracker if object found to be interesting or has high enough voltage
    if (!currEvent->isClosed() && (evtToken.bitObject.getSMV() > .002F ||
                                  currEvent->getCategory() == VisualEvent::INTERESTING)){
      currEvent->setTrackerType(VisualEvent::HOUGH);

      // reset Hough tracker if only now switching to this tracker to save computation
      if (currEvent->trackerChanged()) {
        evtToken = currEvent->getToken(currEvent->getEndFrame());
        LINFO("Resetting Hough Tracker frame: %d event: %d with bounding box %s",
               imgData.frameNum,currEvent->getEventNum(),toStr(evtToken.bitObject.getBoundingBox()).data());
         Image<byte> mask = evtToken.bitObject.getObjectMask(byte(1));
         BitObject obj(rescale(mask, Dims(960, 540)));
         obj.setSMV(evtToken.bitObject.getSMV());
         Image< PixRGB<byte> > prevImgRescaled = rescale(imgData.prevImg, Dims(960, 540));
         if (obj.isValid())
          currEvent->resetHoughTracker(prevImgRescaled, obj);
      }

      // try to run the Hough tracker; if fails, close event
      if (runHoughTracker(rv, currEvent, bayesClassifier, features, imgData, true))
        found = true;
      else {
        LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
        currEvent->close();
      }
    }
  }
  else
  {
    found = true;
    currEvent->setTrackerType(VisualEvent::KALMAN);
  }

  int frameInc = int(imgData.frameNum - currEvent->getValidEndFrame());
  if ( frameInc > itsDetectionParms.itsEventExpirationFrames ) {
    LINFO("Event %i - KalmanHough Tracker failed, closing event",currEvent->getEventNum());
    currEvent->close();
  }

  if (!currEvent->isClosed() && !found) {
    // assign an empty token in case keeping the event open
    evtToken = currEvent->getToken(currEvent->getEndFrame());
    evtToken.frame_nr = imgData.frameNum;
    currEvent->assign_noprediction(evtToken, imgData.foe, currEvent->getValidEndFrame(),\
      itsDetectionParms.itsEventExpirationFrames);
  }
}

// ######################################################################
void VisualEventSet::checkFailureConditions(VisualEvent *currEvent, Dims d)
{
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  // if small object, turn down forget constant to avoid drift
  if (currEvent->getNumberOfFrames() > 1) {
    Token evtToken2 = currEvent->getToken(currEvent->getEndFrame()-1);
    uint maxArea = currEvent->getMaxSize();

    if( evtToken.bitObject.getArea() < (int)((float)maxArea*.25F) && currEvent->getTrackerType() == VisualEvent::HOUGH) {
          float c = 0.5F;
          LINFO("Event %i growing small in Hough tracking mode. Changing forget ratio from %3.2f to %3.2f to avoid drift", \
          currEvent->getEventNum(), currEvent->getForgetConstant(), c);
          currEvent->setForgetConstant(c);
    }
  }

  Rectangle r1 = evtToken.bitObject.getBoundingBox();
  //check distance to edge and size; if close to edge, turn down forget ratio
  if ( (r1.bottomI() >= d.h()-20 || r1.rightI() >= d.w()-20 || r1.top() <=20 || r1.left() <=20) \
    && currEvent->getTrackerType() == VisualEvent::HOUGH )
  {
    float c = 0.80F*currEvent->getForgetConstant();
    if (c < 0.10F) c = 0.F; //clamp to 0 when too small to avoid drifting
    LINFO("Event %i near edge. Changing forget ratio from %3.2f to %3.2f  to avoid drift", \
    currEvent->getEventNum(), currEvent->getForgetConstant(), c);
    currEvent->setForgetConstant(c);
  }
}

// ######################################################################
void VisualEventSet::runNearestNeighborHoughTracker(nub::soft_ref<MbariResultViewer>&rv,
                                                    VisualEvent *currEvent,
                                                    const BayesClassifier &bayesClassifier,
                                                    FeatureCollection& features,
                                                    ImageData& imgData)
{

  bool found = false;
  Token evtToken;

  // prefer the NN tracker, and fall back to the Hough tracker
  if (!runNearestNeighborTracker(currEvent, bayesClassifier, features, imgData, true)){
    evtToken = currEvent->getToken(currEvent->getEndFrame());

    // only use the Hough tracker if object found to be interesting or has high enough voltage
    if (!currEvent->isClosed() && (evtToken.bitObject.getSMV() > .002F ||
                                   currEvent->getCategory() == VisualEvent::INTERESTING)){
      currEvent->setTrackerType(VisualEvent::HOUGH);

      // reset Hough tracker if only now switching to this tracker to save computation
      if (currEvent->trackerChanged()) {
        evtToken = currEvent->getToken(currEvent->getEndFrame());
        LINFO("Resetting Hough Tracker frame: %d event: %d with bounding box %s",
              imgData.frameNum,currEvent->getEventNum(),toStr(evtToken.bitObject.getBoundingBox()).data());
        Image<byte> mask = evtToken.bitObject.getObjectMask(byte(1));
        BitObject obj(rescale(mask, Dims(960, 540)));
        obj.setSMV(evtToken.bitObject.getSMV());
        Image< PixRGB<byte> > prevImgRescaled = rescale(imgData.prevImg, Dims(960, 540));
        if (obj.isValid())
          currEvent->resetHoughTracker(prevImgRescaled, obj);
      }

      // try to run the Hough tracker; if fails, close event
      if (runNearestNeighborTracker(currEvent, bayesClassifier, features, imgData, true))
        found = true;
      else {
        LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
        currEvent->close();
      }
    }
  }
  else
  {
    found = true;
    currEvent->setTrackerType(VisualEvent::NN);
  }

  int frameInc = int(imgData.frameNum - currEvent->getValidEndFrame());
  if ( frameInc > itsDetectionParms.itsEventExpirationFrames ) {
    LINFO("Event %i - Nearest Neighbor Hough Tracker failed, closing event",currEvent->getEventNum());
    currEvent->close();
  }

  if (!currEvent->isClosed() && !found) {
    // assign an empty token in case keeping the event open
    evtToken = currEvent->getToken(currEvent->getEndFrame());
    evtToken.frame_nr = imgData.frameNum;
    currEvent->assign_noprediction(evtToken, imgData.foe,  currEvent->getValidEndFrame(),\
      itsDetectionParms.itsEventExpirationFrames);
  }

}
// ######################################################################
bool VisualEventSet::runHoughTracker(nub::soft_ref<MbariResultViewer>&rv,
                                     VisualEvent *currEvent,
                                     const BayesClassifier &bayesClassifier,
                                     FeatureCollection& features,
                                     ImageData& imgData,
                                     bool skip)
{
  Dims houghDims(960, 540);
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
  Image< byte > binaryImg(houghDims, ZEROS);
  Image< byte > occlusionImg(imgData.img.getDims(), ZEROS);
  occlusionImg = highThresh(occlusionImg, byte(0), byte(255)); //invert image
  Rectangle region;
  uint intersectEventNum;
  bool found = false;
  bool occlusion = false;
  BitObject obj;
  const byte black = byte(0);
  float opacity = 1.0F;

  // does this guy participate in frameNum? already have a token for this frame
  if (currEvent->frameInRange(imgData.frameNum))
    return true;

  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  const Point2D<int> pred = currEvent->predictedLocation();

  // is the prediction too far outside the image?
  int gone = itsDetectionParms.itsMaxDist;
  if ((pred.i < -gone) || (pred.i >= (imgData.img.getWidth() + gone)) ||
      (pred.j < -gone) || (pred.j >= (imgData.img.getHeight() + gone)))
    {
      currEvent->close();
      LINFO("Event %i out of bounds - closed",currEvent->getEventNum());
      return false;
    }

  // if an object intersects, create a mask for it
  if (doesIntersect(evtToken.bitObject, &intersectEventNum, imgData.frameNum)) {
      LINFO("Event %i - Hough Tracker intersection with event %i",currEvent->getEventNum(),\
                                                                  intersectEventNum);
      VisualEvent* vevt = getEventByNumber(intersectEventNum);
      BitObject intersectObj = vevt->getToken(imgData.frameNum).bitObject;
      intersectObj.drawShape(occlusionImg, black, opacity);
      occlusion = true;
  }

  // then apply the mask
  occlusionImg = maskArea(occlusionImg, imgData.mask);

  // calculate the scaling factors for adjusting input to the Hough tracker
  Dims actualDims = imgData.img.getDims();
  float scaleW = (float) houghDims.w() / (float) actualDims.w();
  float scaleH = (float) houghDims.h() / (float) actualDims.h();

  // get the region used for searching for a match based on the dimension of the last token
  // centered on the Kalman predicted location
  //ken evtTokenMax = currEvent->getMaxSizeToken();
  Rectangle rect = evtToken.bitObject.getBoundingBox();
  Point2D<int> predHough((float)pred.i * scaleW, (float)pred.j * scaleH);
  Dims searchDimsHough = Dims((float)rect.width() * scaleW,(float)rect.height() * scaleH);
  Rectangle searchRegion = Rectangle::centerDims(predHough, searchDimsHough);
  searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), houghDims - 1));
  LINFO("Search region %i %s ", currEvent->getEventNum(),toStr(searchRegion).data());

  if (!searchRegion.isValid()) {
    LINFO("Event %i - Hough Tracker invalid search region ",currEvent->getEventNum());
    if (!skip)
      currEvent->close();

    return false;
  }

  Image< PixRGB<byte> > imgRescaled = rescale(imgData.img, houghDims);
  Image< byte > occlusionImgRescaled = rescale(occlusionImg, houghDims);

  LINFO("Running Hough Tracker for event %d", currEvent->getEventNum());
  if (!currEvent->updateHoughTracker(rv, imgData.frameNum, imgRescaled,
                                                   occlusionImgRescaled,
                                                   binaryImg, searchRegion)) {
      if (!skip) {
        LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
        currEvent->close();
      }
      return false;
  }

  // rescale back to original dimensions
  binaryImg = rescale(binaryImg, actualDims);

  // create new token from returned binary image
  obj.reset(binaryImg);

  if (obj.isValid())
    obj.setMaxMinAvgIntensity(luminance(imgData.img));

  float minArea, maxArea;

  if (currEvent->getNumberOfFrames() > 1 || dp.itsUseFoaMaskRegion) {
    // search for up to 4 and shrink down to 0.25 but only if initialized beyond the first frame since
    // the FOA initialized object is generally large
    minArea = 0.25F * (float) evtToken.bitObject.getArea();
    maxArea = 4.0F * (float) evtToken.bitObject.getArea();
  }
  else {
    minArea = dp.itsMinEventArea;
    maxArea = dp.itsMaxEventArea;
  }

  int area = obj.getArea();

  if (obj.isValid() && (area >= 0 || occlusion) ) {
    // apply same cost function as Kalman to make sure Hough is not drifting
    float cost = currEvent->getCost(Token(obj,imgData.frameNum));

    // skip cost function with occlusion since this shifts the centroid
    if (occlusion)
      found = true;
    else if(cost < itsDetectionParms.itsMaxCost) {
      found = true;
      LINFO("Event %i - found token ", currEvent->getEventNum());
      checkFailureConditions(currEvent, imgData.img.getDims());
    }
    else
      LINFO("Event %i - cost too high %g max cost %g", currEvent->getEventNum(), cost, itsDetectionParms.itsMaxCost);
  }
  else
    LINFO("Event %i - invalid token or token found with area %d outside area bounds min area %d max area: %d ", \
        currEvent->getEventNum(), area, minArea, maxArea);

  if (!found && !skip) {
    if ( int(imgData.frameNum - currEvent->getValidEndFrame()) >= itsDetectionParms.itsEventExpirationFrames ) {
      currEvent->close();
      LINFO("Event %i - no token found, closing event", currEvent->getEventNum());
    }
    else {
      // skip over putting in a placeholder for prediction when running multiple trackers and
      // let the multiple tracker algorithm decide
       LINFO("##########Event %i - no token found, keeping event open for expiration frames: %d ##########",
                        currEvent->getEventNum(), itsDetectionParms.itsEventExpirationFrames);
      // get a copy of the last token in this event as placeholder
      Token evtToken = currEvent->getToken(currEvent->getEndFrame());
      evtToken.frame_nr = imgData.frameNum;
      currEvent->assign_noprediction(evtToken, imgData.foe,  currEvent->getValidEndFrame(), \
                                  itsDetectionParms.itsEventExpirationFrames);
     }
 }

  if (found && !currEvent->isClosed()) {
   // associate the best fitting guy
   Token tl = currEvent->getToken(currEvent->getEndFrame());
   FeatureCollection::Data feature = features.extract(tl.bitObject.getBoundingBox(), imgData);
   Token tk(obj, imgData.frameNum, imgData.metadata, feature.featureJETred, feature.featureJETgreen,
            feature.featureJETblue, feature.featureHOG3, feature.featureHOG8);
   tk.bitObject.computeSecondMoments();
   currEvent->assign(tk, imgData.foe, imgData.frameNum);
   LINFO("Event %i - token found at %g, %g area: %d",currEvent->getEventNum(),
         tl.location.x(),
         tl.location.y(),
         tk.bitObject.getArea());
 }

 return found;
}

// ######################################################################
bool VisualEventSet::runKalmanTracker(VisualEvent *currEvent,
                                      const BayesClassifier &bayesClassifier,
                                      FeatureCollection& features,
                                      ImageData& imgData,
                                      bool skip)
{

  bool found = false;

  // does this guy participate in frameNum? already have a token for this frame
  if (currEvent->frameInRange(imgData.frameNum))
    return true;

  // get the predicted location
  const Point2D<int> pred = currEvent->predictedLocation();

  // get a copy of the last token in this event for prediction
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  LINFO("Event %i prediction: %d,%d", currEvent->getEventNum(), pred.i, pred.j);

  // is the prediction too far outside the image?
  int gone = itsDetectionParms.itsMaxDist;
  if ((pred.i < -gone) || (pred.i >= (imgData.segmentImg.getWidth() + gone)) ||
      (pred.j < -gone) || (pred.j >= (imgData.segmentImg.getHeight() + gone)))
    {
      currEvent->close();
      LINFO("Event %i out of bounds - closed",currEvent->getEventNum());
      return false;
    }

  const byte black(0);
  float opacity = 1.0F;
  uint intersectEventNum;
  bool occlusion = false;
  Image< byte > occlusionImg(imgData.segmentImg.getDims(), ZEROS);
  occlusionImg = highThresh(occlusionImg, byte(0), byte(255)); //invert image

  // if an object intersects, create a mask for it
  if (doesIntersect(evtToken.bitObject, &intersectEventNum, imgData.frameNum)) {
      LINFO("Event %i - Kalman Tracker intersection with event %i",currEvent->getEventNum(),\
                                                                  intersectEventNum);
      VisualEvent* vevt = getEventByNumber(intersectEventNum);
      BitObject intersectObj = vevt->getToken(imgData.frameNum).bitObject;
      intersectObj.drawShape(occlusionImg, black, opacity);
      occlusion = true;
  }

  Image< PixRGB<byte> > img = maskArea(imgData.segmentImg, occlusionImg);

  // adjust prediction if negative
  const Point2D<int> center =  Point2D<int>(max(pred.i,0), max(pred.j,0));

  // get the region used for searching for a match based on the dimension of the last token
  Rectangle r1 = evtToken.bitObject.getBoundingBox();
  Dims segmentDims = Dims((float)r1.width()*5,(float)r1.height()*5);
  Dims searchDims = Dims(r1.width(),r1.height());
  Rectangle segmentRegion = Rectangle::centerDims(center, segmentDims);
  Rectangle searchRegion = Rectangle::centerDims(center, searchDims);
  segmentRegion = segmentRegion.getOverlap(Rectangle(Point2D<int>(0, 0), imgData.segmentImg.getDims() - 1));
  searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), imgData.segmentImg.getDims() - 1));
  LINFO("Search region %i %s ", currEvent->getEventNum(),toStr(searchRegion).data());
  LINFO("Segment region %i %s ", currEvent->getEventNum(),toStr(segmentRegion).data());

  if (!searchRegion.isValid() || !segmentRegion.isValid() ) {
    LINFO("Invalid region. Closing event %i", currEvent->getEventNum());
    currEvent->close();
    return false;
  }

  float maxIntensity, minIntensity, avgIntensity, minArea, maxArea;
  evtToken.bitObject.getMaxMinAvgIntensity(maxIntensity, minIntensity, avgIntensity);

  int area = segmentRegion.dims().w() * segmentRegion.dims().h();
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;

  // search for up to 2x the size of 1/4 the size
  minArea = 0.25F * (float) evtToken.bitObject.getArea();
  maxArea = 2.0 * (float) evtToken.bitObject.getArea();

  if (currEvent->getNumberOfFrames() == 1) {
    // bound maximum to last area if using FOA mask and this is the second frame because FOA masks are generally over sized
    maxArea = (float) evtToken.bitObject.getArea();
    // and allow for smaller object in the second frame if the mask is way over sized
    minArea = 1;
  }

  // extract bit objects removing those that fall outside area and intensity minimum set by previous bitobject
  list<BitObject> objs = extractBitObjects(img, center, searchRegion, segmentRegion, minArea, maxArea, 0, 3);//0.5*avgIntensity);

  LINFO("pred. location: %s; region: %s; Number of extracted objects: %ld",
         toStr(pred).data(),toStr(searchRegion).data(),objs.size());

  // now look which one fits best
  float lCost = -1.0F;
  int size = objs.size();
  float maxCost = itsDetectionParms.itsMaxCost;

  list<BitObject>::iterator cObj, lObj = objs.begin();

  // need at least two objects to find a match, otherwise just background
  for (cObj = objs.begin(); cObj != objs.end(); ++cObj) {
    list<BitObject>::iterator next = cObj;
    ++next;

    if (size > 1 && doesIntersect(*cObj, imgData.frameNum)) {
      objs.erase(cObj);
      cObj = next;
      continue;
    }

    //  calculate change in area
    float area = (float) (evtToken.bitObject.getArea());
    float newarea = (float) (cObj->getArea());
    float areaDiff = abs((float) (area - newarea) / (float) area);

    float cost = currEvent->getCost(Token(*cObj, imgData.frameNum));
    Rectangle r2 = cObj->getBoundingBox();
    Rectangle r1 = evtToken.bitObject.getBoundingBox();

    //FeatureCollection::Data f2 = features.extract(r2, imgData);

    // calculate change in bounding box
    float wDiff = abs((float) (r2.width() - r1.width()) / (float) r1.width());
    float hDiff = abs((float) (r2.height() - r1.height()) / (float) r1.height());

    if (cost < 0.0F) {
      objs.erase(cObj);
      cObj = next;
      continue;
    }

    // if cost valid and the bounding box hasn't shifted more than 50 percent (this doesn't work well for midwater)
    if (((lCost == -1.0F) || (cost < lCost)))// && wDiff< 0.50 && hDiff< 0.50)
    {
      lCost = cost;
      lObj = cObj;
      found = true;
    }

  }

  // cost too high but skip if occluded since this shifts the centroid
  if ( (lCost > maxCost || lCost == -1.0)  && !occlusion ) {
    LINFO("Event %i - no token found, event cost: %g maxCost: %g ",
              currEvent->getEventNum(), lCost, maxCost);
    found = false;
  }

  // skip over this when running multiple trackers and let the multiple tracker algorithm decide
  if (!skip && !found) {
      if ( int(imgData.frameNum - currEvent->getValidEndFrame()) > itsDetectionParms.itsEventExpirationFrames )
          currEvent->close();
      else {
          LINFO("########## Event %i - no token found, keeping event open for expiration frames: %d ##########",
            currEvent->getEventNum(), itsDetectionParms.itsEventExpirationFrames);
            evtToken.frame_nr = imgData.frameNum;
            currEvent->assign_noprediction(evtToken, imgData.foe, currEvent->getValidEndFrame(), itsDetectionParms.itsEventExpirationFrames);
      }
  }

  if (found) {
    // associate the best fitting one
    Token tl = currEvent->getToken(currEvent->getEndFrame());
    FeatureCollection::Data feature = features.extract(tl.bitObject.getBoundingBox(), imgData);
    Token tk(*lObj, imgData.frameNum, imgData.metadata, feature.featureJETred,
             feature.featureJETgreen, feature.featureJETblue,
             feature.featureHOG3,  feature.featureHOG8);
    tk.bitObject.computeSecondMoments();
    currEvent->assign(tk, imgData.foe, imgData.frameNum);
    LINFO("Event %i - token found at %g, %g area: %d",currEvent->getEventNum(),
          tl.location.x(),
          tl.location.y(),
          tk.bitObject.getArea());
  }

  objs.clear();
  return found;

}
// ######################################################################
bool VisualEventSet::runNearestNeighborTracker(VisualEvent *currEvent,
                                               const BayesClassifier &bayesClassifier,
                                               FeatureCollection& features,
                                               ImageData& imgData,
                                               bool skip)
{
  Dims d;
  Point2D<int> center;

  // get a copy of the last token in this event for prediction
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  const byte black(0);
  float opacity = 1.0F;
  uint intersectEventNum;
  bool occlusion = false;
  Image< byte > occlusionImg(imgData.segmentImg.getDims(), ZEROS);
  occlusionImg = highThresh(occlusionImg, byte(0), byte(255)); //invert image

  // if an object intersects, create a mask for it
  if (doesIntersect(evtToken.bitObject, &intersectEventNum, imgData.frameNum)) {
    LINFO("Event %i - Nearest Neighbor Tracker intersection with event %i",currEvent->getEventNum(),\
                                                                  intersectEventNum);
    VisualEvent* vevt = getEventByNumber(intersectEventNum);
    BitObject intersectObj = vevt->getToken(imgData.frameNum).bitObject;
    intersectObj.drawShape(occlusionImg, black, opacity);
    occlusion = true;
  }

  Image< PixRGB<byte> > img = maskArea(imgData.segmentImg, occlusionImg);

  // get the object dimensions and centroid for token
  d = evtToken.bitObject.getObjectDims();
  center = evtToken.bitObject.getCentroid();

   // get the region used for searching for a match based on the dimension of the last token
  Rectangle r1 = evtToken.bitObject.getBoundingBox();
  Dims segmentDims = Dims((float)r1.width()*3,(float)r1.height()*3);
  Dims searchDims = Dims(r1.width(),r1.height());
  Rectangle segmentRegion = Rectangle::centerDims(center, segmentDims);
  Rectangle searchRegion = Rectangle::centerDims(center, searchDims);
  segmentRegion = segmentRegion.getOverlap(Rectangle(Point2D<int>(0, 0), imgData.segmentImg.getDims() - 1));
  searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), imgData.segmentImg.getDims() - 1));
  LINFO("Search region %i %s ", currEvent->getEventNum(),toStr(searchRegion).data());
  LINFO("Segment region %i %s ", currEvent->getEventNum(),toStr(segmentRegion).data());

  int minArea, maxArea;
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
  int area = segmentRegion.dims().w() * segmentRegion.dims().h();

  if (currEvent->getNumberOfFrames() > 1 || dp.itsUseFoaMaskRegion) {
    // search for up to 4 and shrink down to 0.25 but only if initialized beyond the first frame since
    // the FOA initialized object is generally large
    minArea = 0.25F * (float) evtToken.bitObject.getArea();
    maxArea = min(4.0F * (float) evtToken.bitObject.getArea(), 0.90F * (float) area);
  }
  else {
    minArea = dp.itsMinEventArea;
    maxArea = dp.itsMaxEventArea;
  }

  if (!searchRegion.isValid() || !segmentRegion.isValid() ) {
    LINFO("Invalid region. Closing event %i", currEvent->getEventNum());
    currEvent->close();
    return false;
  }

  float maxIntensity, minIntensity, avgIntensity;
  evtToken.bitObject.getMaxMinAvgIntensity(maxIntensity, minIntensity, avgIntensity);

  list<BitObject> objs = extractBitObjects(img, center, searchRegion, segmentRegion, minArea, maxArea, 0.5*avgIntensity, 3);

  LINFO("region: %s; Number of extracted objects: %ld", toStr(searchRegion).data(),objs.size());

  // now look which one fits best
  float lCost = -1.0F;
  int size = objs.size();
  bool found = false;
  float maxCost = itsDetectionParms.itsMaxCost;
  Point2D<int> p1 = evtToken.bitObject.getCentroid();

  list<BitObject>::iterator cObj, lObj = objs.begin();
  for (cObj = objs.begin(); cObj != objs.end(); ++cObj)
  {
    list<BitObject>::iterator next = cObj;
    ++next;

    if (size > 1 && doesIntersect(*cObj, imgData.frameNum) ) {
      objs.erase(cObj);
      cObj = next;
      continue;
    }

    Point2D<int> p2 = cObj->getCentroid();
    Rectangle r2 = cObj->getBoundingBox();
    Rectangle r1 = evtToken.bitObject.getBoundingBox();

    // calculate cost function as distance between centroids
    float cost1 = sqrt(pow((double)(p1.i - p2.i),2.0) + pow((double)(p1.j - p2.j),2.0));
    // calculate other cost function as distance between bounding box corners
    float cost2 = sqrt(pow((double)(r1.top() - r2.top()),2.0) +  pow((double)(r1.left() - r2.left()),2.0));
    float cost3 = sqrt(pow((double)(r1.bottomI() - r2.bottomI()),2.0) +
                       pow((double)(r1.rightI() - r2.rightI()),2.0));
    float cost = cost1 + cost2 + cost3;

    //  calculate change in area
    float area = (float)(evtToken.bitObject.getArea());
    float newarea = (float)(cObj->getArea());
    float areaDiff= abs( (float)(area - newarea)/(float)area );

    // calculate change in bounding box
    float wDiff= abs( (float)(r2.width() - r1.width())/(float)r1.width() );
    float hDiff= abs( (float)(r2.height() - r1.height())/(float)r1.height() );

    if (cost < 0.0F) {
      objs.erase(cObj);
      cObj = next;
      continue;
    }

    // if cost valid and the bounding box hasn't shifted more than 50 percent
    if ( ((lCost == -1.0F) || (cost < lCost)) && wDiff< 0.50 && hDiff< 0.50)
    {
      lCost = cost;
      lObj = cObj;
      found = true;
    }

  }

  // cost too high but skip if occluded since this shifts the centroid
  if ( (lCost > maxCost || lCost == -1.0)  && !occlusion ) {
    LINFO("Event %i - no token found, event cost: %g maxCost: %g ",
          currEvent->getEventNum(), lCost, maxCost);
    found = false;
  }

  // skip over this when running multiple trackers and let the multiple tracker algorithm decide
  if (!skip && !found) {
    if ( int(imgData.frameNum - currEvent->getValidEndFrame()) > itsDetectionParms.itsEventExpirationFrames )
      currEvent->close();
    else {
      LINFO("########## Event %i - no token found, keeping event open for expiration frames: %d ##########",
            currEvent->getEventNum(), itsDetectionParms.itsEventExpirationFrames);
      evtToken.frame_nr = imgData.frameNum;
      currEvent->assign_noprediction(evtToken, imgData.foe, currEvent->getValidEndFrame(), itsDetectionParms.itsEventExpirationFrames);
    }
  }

  if (found) {
    // associate the best fitting one
    Token tl = currEvent->getToken(currEvent->getEndFrame());
    FeatureCollection::Data feature = features.extract(tl.bitObject.getBoundingBox(), imgData);
    Token tk(*lObj, imgData.frameNum, imgData.metadata, feature.featureJETred,
             feature.featureJETgreen, feature.featureJETblue,
             feature.featureHOG3, feature.featureHOG8);
    tk.bitObject.computeSecondMoments();
    currEvent->assign(tk, imgData.foe, imgData.frameNum);
    LINFO("Event %i - token found at %g, %g area: %d",currEvent->getEventNum(),
          tl.location.x(),
          tl.location.y(),
          tk.bitObject.getArea());
  }

  objs.clear();
  return found;
}

// ######################################################################
void VisualEventSet::getAreaRange(int &minArea, int &maxArea)
{
  list<VisualEvent *>::iterator currEvent;
  int currMinArea = itsDetectionParms.itsMinEventArea;
  int currMaxArea = itsDetectionParms.itsMaxEventArea;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent) {
    if((*currEvent)->getMaxSize() > currMaxArea)
     currMaxArea = (*currEvent)->getMaxSize();

    if((*currEvent)->getMinSize() < currMinArea)
     currMinArea = (*currEvent)->getMinSize();
  }
}

// ######################################################################
float VisualEventSet::getAcceleration(uint skipEventNum)
{
  list<VisualEvent *>::iterator currEvent;
  float sumAccel = 0.F;
  int i = 0;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent) {
    if((*currEvent)->getEventNum() != skipEventNum) {
     sumAccel += (*currEvent)->getAcceleration();
     i++;
     }
  }
  if (i > 0)
    return sumAccel/(float)i;
  return 0.F;
}


// ######################################################################
void VisualEventSet::updateEvents(nub::soft_ref<MbariResultViewer>&rv,
                                  const BayesClassifier &bayesClassifier,
                                  FeatureCollection& features,
                                  ImageData& imgData)
{
  if (startframe == -1) {startframe = (int) imgData.frameNum; endframe = (int) imgData.frameNum;}
  if ((int) imgData.frameNum > endframe) endframe = (int) imgData.frameNum;

  list<VisualEvent *>::iterator currEvent;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    if ((*currEvent)->isOpen()) {
      switch(itsDetectionParms.itsTrackingMode) {
      case(TMKalmanFilter):
        (*currEvent)->setTrackerType(VisualEvent::KALMAN);
        runKalmanTracker(*currEvent, bayesClassifier, features, imgData);
        break;
      case(TMNearestNeighbor):
        (*currEvent)->setTrackerType(VisualEvent::NN);
        runNearestNeighborTracker(*currEvent, bayesClassifier, features,imgData);
        break;
      case(TMHough):
        runHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMNearestNeighborHough):
        runNearestNeighborHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMKalmanHough):
        runKalmanHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMNone):
        break;
      default:
        (*currEvent)->setTrackerType(VisualEvent::KALMAN);
        runKalmanTracker(*currEvent, bayesClassifier, features, imgData);
        break;
      }
    }
}

// ######################################################################
void VisualEventSet::initiateEvents(list<BitObject>& bos,
                                    FeatureCollection& features,
                                    ImageData &imgData)
{
  if (startframe == -1) {startframe = imgData.frameNum; endframe = imgData.frameNum;}
  if (imgData.frameNum > endframe) endframe = imgData.frameNum;

  list<BitObject>::iterator currObj;

  // loop over the BitObjects
  currObj = bos.begin();
  while(currObj != bos.end())    {
    list<BitObject>::iterator next = currObj;
    ++next;

    // is there an intersection with an event? then reset the object
    if (resetIntersect(imgData.img, *currObj, imgData.foe, imgData.frameNum))
      bos.erase(currObj);

    currObj = next;
  }

  // now go through all the remaining BitObjects and create new events for them
  // if they are not already out of bounds and using a tracker
  for (currObj = bos.begin(); currObj != bos.end(); ++currObj)
    {
      FeatureCollection::Data feature = features.extract(currObj->getBoundingBox(), imgData);
      Token token = Token(*currObj, imgData.frameNum, imgData.metadata, feature.featureJETred,
                          feature.featureJETgreen, feature.featureJETblue,
                          feature.featureHOG3, feature.featureHOG8);
      itsEvents.push_back(new VisualEvent(token, itsDetectionParms, imgData.img));
      LINFO("assigning object of area: %i to new event %i frame %d",currObj->getArea(),
            itsEvents.back()->getEventNum(), imgData.frameNum);
    }
}

// ######################################################################
bool VisualEventSet::resetIntersect(Image< PixRGB<byte> >& img, BitObject& obj, const Vector2D& curFOE, int frameNum)
{
  // ######## Initialization of variables, reading of parameters etc.
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
  list<VisualEvent *>::iterator cEv;
  int area;
  float areadiff, distul, distbr;
  Token evtToken;
  Rectangle r1, r2;
  Image<byte> mask, mask1, mask2;
  BitObject obj1, obj2;
  Image< PixRGB<byte> > imgRescaled = rescale(img, Dims(960, 540));

  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv) {
    if ((*cEv)->doesIntersect(obj, frameNum)) {
      switch (dp.itsTrackingMode) {
            case(TMHough):
            case(TMNearestNeighborHough):
            case(TMKalmanHough):
              areadiff = 0.F;

              if ((*cEv)->getNumberOfFrames() > 1 && (*cEv)->getTrackerType() == VisualEvent::HOUGH ) {

                evtToken = (*cEv)->getToken((*cEv)->getEndFrame());
                area = evtToken.bitObject.getArea();
                areadiff = (area - evtToken.bitObject.intersect(obj))/ area;

                // if intersecting area at least 50%
                if (areadiff > .5F){
                  mask1 = obj.getObjectMask(byte(1));
                  mask2 = evtToken.bitObject.getObjectMask(byte(1));
                  mask = mask1 + mask2;

                  // reset the object and propagate the saliency map voltage
                  obj1.reset(mask);
                  obj1.setSMV(obj.getSMV());

                  // create second object rescaled to reduce memory used by the Hough tracker
                  mask = rescale(mask, Dims(960,540));
                  obj2.reset(mask);

                  if (obj2.isValid()){
                      (*cEv)->resetHoughTracker(imgRescaled, obj2);
                      (*cEv)->resetBitObject(frameNum, obj1);
                      LINFO("Resetting Hough Tracker frame: %d event: %d with bit object in bounding box %s",
                       frameNum,(*cEv)->getEventNum(),toStr(obj.getBoundingBox()).data());
                  }
                }
              }
            break;
            case(TMKalmanFilter):
            case(TMNearestNeighbor):
            case(TMNone):
            break;
          }
    return true;
    }
  }
  return false;
}

// ######################################################################
bool VisualEventSet::doesIntersect(BitObject& obj, int frameNum)
{
  list<VisualEvent *>::iterator cEv;
  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv)
      if ((*cEv)->doesIntersect(obj,frameNum)) {
      // reset the SMV for this bitObject
      Token  evtToken = (*cEv)->getToken(frameNum);
      evtToken.bitObject.setSMV(obj.getSMV());
      return true;
    }
  return false;
}

// ######################################################################
bool VisualEventSet::doesIntersect(BitObject& obj, uint* eventNum, int frameNum)
{
  list<VisualEvent *>::iterator cEv;
  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv)
    // return the first object that intersects
    if ((*cEv)->doesIntersect(obj,frameNum)) {
      *eventNum = (*cEv)->getEventNum();
      return true;
    }
  return false;
}

// ######################################################################
uint VisualEventSet::numEvents() const
{
  return itsEvents.size();
}

// ######################################################################
void VisualEventSet::reset()
{
  itsEvents.clear();
}

// ######################################################################
void VisualEventSet::replaceEvent(uint eventnum, VisualEvent *event)
{
  list<VisualEvent *>::iterator currEvent = itsEvents.begin();
  while (currEvent != itsEvents.end())  {
    if((*currEvent)->getEventNum() == eventnum) {
      itsEvents.insert(currEvent, event);
      delete *currEvent;
      itsEvents.erase(currEvent);
      return;
    }
    ++currEvent;
  }
  LFATAL("Event %d does not exist in event list cannot replace", eventnum);
}
// ######################################################################
void VisualEventSet::cleanUp(uint currFrame, uint lastFrame)
{
  list<VisualEvent *>::iterator currEvent = itsEvents.begin();

  while(currEvent != itsEvents.end()) {
    list<VisualEvent *>::iterator next = currEvent;
    ++next;

    switch((*currEvent)->getState())
      {
      case(VisualEvent::DELETE):
        LINFO("Erasing event %i", (*currEvent)->getEventNum());
        delete *currEvent;
        itsEvents.erase(currEvent);
        break;
      case(VisualEvent::WRITE_FINI):
        LINFO("Event %i flagged as written", (*currEvent)->getEventNum());
        break;
      case(VisualEvent::CLOSED):
        LINFO("Event %i flagged as closed", (*currEvent)->getEventNum());
        break;
      case(VisualEvent::OPEN):
        if (itsDetectionParms.itsMaxEventFrames > 0 && currFrame > ((*currEvent)->getStartFrame() + itsDetectionParms.itsMaxEventFrames)){
          //limit event to itsMaxFrames
          LINFO("Event %i reached max frame count:%d - flagging as closed", (*currEvent)->getEventNum(),\
                itsDetectionParms.itsMaxEventFrames);
          (*currEvent)->close();
        }
        break;
      default:
        //this event is still within the window of itsMaxFrames
        LDEBUG("Event %d still open", (*currEvent)->getEventNum());
        break;
      }

    currEvent = next;
  } // end for loop over events
}

// ######################################################################
void VisualEventSet::closeAll()
{
  list<VisualEvent *>::iterator cEvent;
  for (cEvent = itsEvents.begin(); cEvent != itsEvents.end(); ++cEvent)
    (*cEvent)->close();
}
// ######################################################################
void VisualEventSet::printAll()
{
  list<VisualEvent *>::iterator cEvent;
  for (cEvent = itsEvents.begin(); cEvent != itsEvents.end(); ++cEvent)
    {
      LINFO("EVENT %d sframe %d eframe %d numtokens %d",
            (*cEvent)->getEventNum(),
            (*cEvent)->getStartFrame(),
            (*cEvent)->getEndFrame(),
            (*cEvent)->getNumberOfTokens());
    }
}
// ######################################################################
vector<Token> VisualEventSet::getTokens(uint frameNum)
{
  vector<Token> tokens;
  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    {
      // does this guy participate in frameNum?
      if (!(*currEvent)->frameInRange(frameNum)) continue;

      tokens.push_back((*currEvent)->getToken(frameNum));
    } // end loop over events

  return tokens;
}

// ######################################################################
void VisualEventSet::drawTokens(Image< PixRGB<byte> >& img,
                                uint frameNum,
                                int circleRadius,
                                BitObjectDrawMode mode,
                                float opacity,
                                PixRGB<byte> colorInteresting,
                                PixRGB<byte> colorCandidate,
                                PixRGB<byte> colorPred,
                                PixRGB<byte> colorFOE,
                                bool showEventLabels,
                                bool showCandidate,
                                bool saveNonInterestingEvents,
                                float scaleW,
                                float scaleH)
{
  // dimensions of the number text and location to put it at
  const int numW = 10;
  const int numH = 21;
  Token tk;

  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
  {
      // does this guy  participate in frameNum ? and
      // if also saving non-interesting events and this is BORING event, be sure to save this
      // otherwise, save all INTERESTING events
      if( (*currEvent)->frameInRange(frameNum) &&
          ( (saveNonInterestingEvents && (*currEvent)->getCategory() == VisualEvent::BORING ) ||
          (*currEvent)->getCategory() == VisualEvent::INTERESTING  ||
          showCandidate ) )
        {
          PixRGB<byte> circleColor;
          tk = (*currEvent)->getToken(frameNum);

          if(!tk.location.isValid())
            continue;

          Point2D<int> center = tk.location.getPoint2D();
          center.i *= scaleW;
          center.j *= scaleH;

          if ((*currEvent)->getCategory() == VisualEvent::INTERESTING)
            circleColor = colorInteresting;
          else
            circleColor = colorCandidate;

          // if requested, prepare the event labels
          Image< PixRGB<byte> > textImg;
          if (showEventLabels)
            {
              // write the text and create the overlay image
              string numText = toStr((*currEvent)->getEventNum());
              ostringstream ss;
              ss.precision(2);
              ss << numText;
              if (tk.class_probability >= 0.F && tk.class_probability <= 1.0F) {
                ss << "," << tk.class_name;
                ss << "," << tk.class_probability;
              }
              //ss << numText << "," << (*currEvent)->getForgetConstant();

              textImg.resize(numW * ss.str().length(), numH, NO_INIT);
              //textImg.resize(numW * numText.length(), numH, NO_INIT);
              textImg.clear(COL_WHITE);
              writeText(textImg, Point2D<int>(0,0), ss.str().data());
              //writeText(textImg, Point2D<int>(0,0), numText.data());
            }

          // draw the event object itself if requested
          if (circleColor != COL_TRANSPARENT)
            {
              // the box so that the text knows where to go
              Rectangle bbox;

              // draw rectangle or circle and determine the pos of the number label
              if (tk.bitObject.isValid())
                {
                  tk.bitObject.draw(mode, img, circleColor, opacity);
                  bbox = tk.bitObject.getBoundingBox(BitObject::IMAGE);
                  Point2D<int> topleft((float)(bbox.left())*scaleW, (float)(bbox.top())*scaleH);
                  Dims dims((float)(bbox.width())*scaleW, (float)(bbox.height())*scaleH);
                  bbox = Rectangle(topleft, dims);
                  bbox = bbox.getOverlap(img.getBounds());
                }
              else
                {
                  LINFO("BitObject is invalid: area: %i;",tk.bitObject.getArea());
                  LFATAL("bounding box: %s",toStr(tk.bitObject.getBoundingBox()).data());
                  drawCircle(img, center, circleRadius, circleColor);
                  bbox = Rectangle::tlbrI(center.j - circleRadius, center.i - circleRadius,
                                          center.j + circleRadius, center.i + circleRadius);
                  bbox = bbox.getOverlap(img.getBounds());
                }

              // if requested, write the event labels into the image
              if (showEventLabels)
                {
                  Point2D<int> numLoc = getLabelPosition(img.getDims(),bbox,textImg.getDims());
                  Image<PixRGB <byte> > textImg2 = replaceVals(textImg,COL_BLACK,circleColor);
                  textImg2 = replaceVals(textImg2,COL_WHITE,COL_TRANSPARENT);
                  pasteImage(img,textImg2,COL_TRANSPARENT, numLoc, opacity);
                } // end if (showEventLabels)

            } // end if we're not transparent

          // now do the same for the predicted value
          if ((colorPred != COL_TRANSPARENT) && tk.prediction.isValid())
            {
              Point2D<int> ctr = tk.prediction.getPoint2D();
              ctr.i *= scaleW;
              ctr.j *= scaleH;
              Rectangle ebox =
                Rectangle::tlbrI(ctr.j - circleRadius, ctr.i - circleRadius, ctr.j + circleRadius, ctr.i + circleRadius);
                  ebox = ebox.getOverlap(img.getBounds());

                  // round down the radius in case near the edges
                  if (ebox.width() > 0 && ebox.height() > 0) {
                      int radius = (int) sqrt(pow(ebox.width(), 2.0) + pow(ebox.height(), 2.0))/2;
                      drawCircle(img, ctr, radius, colorPred);
                      if (showEventLabels) {
                          Point2D<int> numLoc = getLabelPosition(img.getDims(), ebox, textImg.getDims());
                          Image< PixRGB<byte> > textImg2 = replaceVals(textImg, COL_BLACK, colorPred);
                          textImg2 = replaceVals(textImg2, COL_WHITE, COL_TRANSPARENT);
                          pasteImage(img, textImg2, COL_TRANSPARENT, numLoc, opacity);
                      }
                  }
            }

        }
    } // end loop over events

  if ((colorFOE != COL_TRANSPARENT) && tk.foe.isValid())
    {
      Point2D<int> ctr = tk.foe.getPoint2D();
      ctr.i *= scaleW;
      ctr.j *= scaleH;
      drawDisk(img, ctr,2,colorFOE);
    }
}


// ######################################################################
Point2D<int> VisualEventSet::getLabelPosition(Dims imgDims,
                                         Rectangle bbox,
                                         Dims textDims) const
{
  // distance of the text label from the bbox
  const int dist = 2;

  Point2D<int> loc(bbox.left(),(bbox.top() - dist - textDims.h()));

  // not enough space to the right? -> shift as appropriate
  if ((loc.i + textDims.w()) > imgDims.w())
    loc.i = imgDims.w() - textDims.w() - 1;

  // not enough space on the top? -> move to the bottom
  if (loc.j < 0)
    loc.j = bbox.bottomI() + dist;

  return loc;
}

// ######################################################################
PropertyVectorSet VisualEventSet::getPropertyVectorSet()
{
  PropertyVectorSet pvs;

  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
       ++currEvent)
    pvs.itsVectors.push_back((*currEvent)->getPropertyVector());

  return pvs;
}

// ######################################################################
PropertyVectorSet VisualEventSet::getPropertyVectorSetToSave()
{
  PropertyVectorSet pvs;
  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
       ++currEvent) {
    if((*currEvent)->isClosed()) {
      pvs.itsVectors.push_back((*currEvent)->getPropertyVector());
    }
  }

  return pvs;
}

// ######################################################################
int VisualEventSet::getAllClosedFrameNum(uint currFrame)
{
  list<VisualEvent *>::iterator currEvent;
  for (int frame = (int)currFrame; frame >= -1; --frame)
    {
      bool done = true;

      for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
           ++currEvent)
        {
          done &= ((frame < (int)(*currEvent)->getStartFrame())
                   || (*currEvent)->isClosed());
          if (!done) break;
        }

      if (done) return frame;
    }
  return -1;
}

// ######################################################################
bool VisualEventSet::doesEventExist(uint eventNum) const
{
  list<VisualEvent *>::const_iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->getEventNum() == eventNum) return true;

  return false;
}
// ######################################################################
VisualEvent *VisualEventSet::getEventByNumber(uint eventNum) const
{
  list<VisualEvent *>::const_iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->getEventNum() == eventNum) return *evt;

  LFATAL("Event with number %i does not exist.",eventNum);

  return *evt;
}
// ######################################################################
list<VisualEvent *>
VisualEventSet::getEventsReadyToSave(uint framenum)
{
  list<VisualEvent *> result;
  list<VisualEvent *>::iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->isClosed()) result.push_back(*evt);

  return result;
}

// ######################################################################
list<VisualEvent *>
VisualEventSet::getEventsForFrame(uint framenum)
{
  list<VisualEvent *> result;
  list<VisualEvent *>::iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->frameInRange(framenum)) result.push_back(*evt);

  return result;
}


// ######################################################################
list<BitObject>
VisualEventSet::getBitObjectsForFrame(uint framenum)
{
  list<BitObject> result;
  list<VisualEvent *>::iterator evt;

  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->frameInRange(framenum))
      if((*evt)->getToken(framenum).bitObject.isValid())
        result.push_back((*evt)->getToken(framenum).bitObject);

  return result;
}

// ######################################################################
const int VisualEventSet::minSize()
{
  return itsDetectionParms.itsMinEventArea;
}
