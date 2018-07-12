//============================================================================
// Name        : locateCreatures.C
// Author      : Mayra Ochoa & Raymond Esteybar
// Version     :
// Copyright   : Your copyright notice
// Description : Parses through .xml to gather the values in <object>
//		 for the <name> & <bndbox> dimensions
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <list>  // Change into a diff DS or make our own class
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include "Media/FrameState.H"
#include "Image/OpenCVUtil.H"
#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "Component/JobServerConfigurator.H"
#include "Features/HistogramOfGradients.H"
#include "Image/FilterOps.H"    // for lowPass3y()
#include "Image/Kernels.H"      // for twofiftyfives()
#include "Image/ColorOps.H"
#include "Image/fancynorm.H"
#include "Image/MorphOps.H"
#include "Image/ShapeOps.H"   // for rescale()
#include "Raster/GenericFrame.H"
#include "Raster/PngWriter.H"
#include "Media/FrameRange.H"
#include "Media/FrameSeries.H"
#include "Media/SimFrameSeries.H"
#include "Media/MediaOpts.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/Retina.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/SimulationViewerStd.H"
#include "SIFT/Histogram.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Util/sformat.H"
#include "Util/StringConversions.H"
#include "Util/Pause.H"
#include "Data/Logger.H"
#include "Data/MbariMetaData.H"
#include "Data/MbariOpts.H"
#include "DetectionAndTracking/FOEestimator.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "DetectionAndTracking/Segmentation.H"
#include "DetectionAndTracking/ColorSpaceTypes.H"
#include "DetectionAndTracking/ObjectDetection.H"
#include "DetectionAndTracking/Preprocess.H"
#include "Image/MbariImage.H"
#include "Image/MbariImageCache.H"
#include "Image/BitObject.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/IO.H"
#include "Learn/BayesClassifier.H"
#include "Media/MbariResultViewer.H"
#include "Motion/MotionEnergy.H"
#include "Motion/MotionOps.H"
#include "Motion/OpticalFlow.H"
#include "Util/StringConversions.H"
#include "Utils/Version.H"
#include "Component/ModelManager.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "DetectionAndTracking/BoxObjectDetection.H"

using namespace std;
using namespace xercesc;

#define DEBUG
#define MAX_INT32 2147483647

struct BoundingBox {
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

// Functions
void storeValue(BoundingBox& temp, const string& tagName, const string& tagValue);    		// Stores tag value in temporary 'Creature' Object
string getmyXML(const string& descrip, const uint& fnum);								// Get .xml in directory
void getObjectValues(XercesDOMParser *itsParser, list<Rectangle>& creatureDims); 	// Reads .xml for values in <object> ... </object>

int main(const int argc, const char** argv) {

	DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
	Segmentation segmentation;

	#ifdef DEBUG
    PauseWaiter pause;
    setPause(true);
    #endif
    const int foaSizeRatio = 19;

    try {
    	XMLPlatformUtils::Initialize();
    } catch(...) {
    	LINFO("CAUGHT EXCEPTION");
    	return -1;
    }

	// Variables Utilized
	XercesDOMParser *itsParser = new XercesDOMParser;	// Ensures File is readable

	bool singleFrame = false;
	int numSpots = 0;
	uint frameNum = 0;
	Image< PixRGB<byte> > inputRaw, inputScaled;

	ModelManager manager("Read .xml Files");

	nub::soft_ref<SimEventQueueConfigurator>seqc(new SimEventQueueConfigurator(manager));
    manager.addSubComponent(seqc);
	nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
	nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));

	nub::soft_ref<BoxObjectDetection> objdet(new BoxObjectDetection(manager));
    manager.addSubComponent(objdet);

    nub::soft_ref<Preprocess> preprocess(new Preprocess(manager));
    manager.addSubComponent(preprocess);

	manager.addSubComponent(ofs);
	manager.addSubComponent(ifs);

	// Get the directory of this executable
    string exe(argv[0]);
    size_t found = exe.find_last_of("/\\");
    nub::soft_ref<Logger> logger(new Logger(manager, ifs, ofs, exe.substr(0, found)));
    manager.addSubComponent(logger);

    nub::soft_ref<MbariResultViewer> rv(new MbariResultViewer(manager));
    manager.addSubComponent(rv);

    nub::ref<DetectionParametersModelComponent> parms(new DetectionParametersModelComponent(manager));
    manager.addSubComponent(parms);

    // Request MBARI specific option aliases
    REQUEST_OPTIONALIAS_MBARI(manager);

    // Request a bunch of toolkit option aliases
    REQUEST_OPTIONALIAS_NEURO(manager);

	if (manager.parseCommandLine(argc, argv, "", 0, -1) == NULL)
		LFATAL("Invalid command line argument. Aborting program now !");
	// fix empty frame range bug and set the range to be the same as the input frame range
    FrameRange fr = ifs->getModelParamVal< FrameRange > ("InputFrameRange");
    if (fr.getLast() == MAX_INT32) {
        FrameRange range(0, 0, 0);
        ifs->setModelParamVal(string("InputFrameRange"), range);
        ofs->setModelParamVal(string("OutputFrameRange"), range);
        singleFrame = true;
    }
    else
        ofs->setModelParamVal(string("OutputFrameRange"), fr);
	    // get image dimensions and set a few parameters that depend on it
    parms->reset(&dp);

    // is this a a gray scale sequence ? if so disable computing the color channels
    // to save computation time. This assumes the color channel has no weight !
    if (dp.itsColorSpaceType == SAColorGray) {
        string search = "C";
        string source = manager.getOptionValString(&OPT_VisualCortexType);
        size_t pos = source.find(search);
        if (pos != string::npos) {
            string replace = source.erase(pos, 1);
            manager.setOptionValString(&OPT_VisualCortexType, replace);
        }
    }

	Dims scaledDims = ifs->peekDims();
	// if the user has selected to retain the original dimensions in the events disable scaling in the frame series
    // and use the scaling factors directly
    if (dp.itsSaveOriginalFrameSpec) {
        ofs->setModelParamVal(string("OutputFrameDims"), Dims(0, 0));
        ifs->setModelParamVal(string("InputFrameDims"), Dims(0, 0));
    }

    int foaRadius;
    const string foar = manager.getOptionValString(&OPT_FOAradius);
    convertFromString(foar, foaRadius);

    // calculate the foa size based on the image size if set to defaults
    // A zero foa radius indicates to set defaults from input image dims
    if (foaRadius == 0) {
        foaRadius = scaledDims.w() / foaSizeRatio;
        char str[256];
        sprintf(str, "%d", foaRadius);
        manager.setOptionValString(&OPT_FOAradius, str);
    }

    // get reference to the SimEventQueue
    nub::soft_ref<SimEventQueue> seq = seqc->getQ();
	manager.start();
	// set defaults for detection model parameters
    DetectionParametersSingleton::initialize(dp, scaledDims, foaRadius);

    // initialize the visual event set
    VisualEventSet eventSet(dp, manager.getExtraArg(0));

    // initialize masks
    Image<byte> mask(scaledDims, ZEROS);
    mask = highThresh(mask, byte(0), byte(255));

    Image<byte> staticClipMask(scaledDims, ZEROS);
    mask = highThresh(mask, byte(0), byte(255));
    staticClipMask = maskArea(mask, &dp);

    // initialize the preprocess
    preprocess->init(ifs, scaledDims);
    ifs->reset1(); //reset to state after construction since the preprocessing caches input frames

    // main loop:
    LINFO("MAIN_LOOP");

    MbariImage< PixRGB<byte> > input(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB<byte> > prevInput(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB <byte> > output(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    Image<byte>  foaIn;
    ImageData imgData;
    Image< PixRGB<byte> > segmentIn(input.getDims(), ZEROS);

    Image< PixRGB<byte> > clampedInput(input.getDims(), ZEROS);

    // initialize property list and FOE estimator
//    PropertylistSet pvs;
    FOEestimator foeEst(20, 0);
    Vector2D curFOE;

    // Initialize bayesian network
    BayesClassifier bayesClassifier(dp.itsBayesPath, dp.itsFeatureType, scaledDims);
    FeatureCollection features(scaledDims);

	// Read all .xml
	while(1) {

		// Read new image in
		FrameState is = FRAME_NEXT;

		if (!singleFrame)
			is = ifs->updateNext();
		else
			is = FRAME_FINAL;

		if (is == FRAME_COMPLETE )	break; // done
		if (is == FRAME_NEXT || is == FRAME_FINAL) { // new frame

			LINFO("Reading new frame");

			numSpots = 0;

			// initialize the default mask
			mask = staticClipMask;

			// Get Frame
			inputRaw = ifs->readRGB();
			inputScaled = rescale(inputRaw, scaledDims);

			frameNum = ifs->frame();

			// get updated input image erasing previous bit objects
			const list<BitObject> bitObjectFrameList = eventSet.getBitObjectsForFrame(frameNum - 1);

			// update the background cache
			input = preprocess->update(inputScaled, prevInput, frameNum, bitObjectFrameList);

			// (image, , window)
			rv->display(input, frameNum, "Input");

			// choose image to segment; these produce different results and vary depending on midwater/benthic/etc.
			if (dp.itsSegmentAlgorithmInputType == SAILuminance) {
				segmentIn = input;
			} else if (dp.itsSegmentAlgorithmInputType == SAIDiffMean) {
				segmentIn = preprocess->clampedDiffMean(inputScaled);
			} else {
				segmentIn = preprocess->clampedDiffMean(inputScaled);
			}

			// update the focus of expansion - is this still needed ?
			foaIn = luminance(input);
			double threshold = mean(foaIn);
			curFOE = foeEst.updateFOE(makeBinary(foaIn, (const byte)threshold));

			Dims d = input.getDims();
			if (prevInput.initialized())
				 clampedInput = preprocess->clampedDiffMean(prevInput);

			imgData.foe = curFOE;
			imgData.frameNum = frameNum;
			imgData.clampedImg = clampedInput;
			imgData.img = input;
			imgData.metadata = input.getMetaData();
			imgData.prevImg = prevInput;
			imgData.segmentImg = segmentIn;
			imgData.mask = mask;

			// update the open events
			eventSet.updateEvents(rv, bayesClassifier, features, imgData);
		}
		std::list<Rectangle> reclist;
		std::list<BitObject> objs;

		// Read File - get list<Rectangle>
		string description(manager.getOptionValString(&OPT_InputFrameSource).c_str());
		description = "/" + getmyXML(description, frameNum);

		itsParser->resetDocumentPool();
		itsParser->parse(description.c_str()); // Ensures the file is readable

		// 	- Extract Values
		if (itsParser->getErrorCount() == 0) {
			getObjectValues(itsParser, reclist);
		} else {
		  cout << "Error when attempting to parse the XML file : " << description.c_str() << endl;
		  return -1;
		}

		#ifdef DEBUG
		Dims d = segmentIn.getDims();
		rv->display(segmentIn, frameNum, "SegmentIn");
		Rectangle r = Rectangle::tlbrI(0,0,d.h()-1,d.w()-1);
		Image< PixRGB<byte> > t = segmentation.runGraph(segmentIn, r, 1.0);
		rv->display(t, frameNum, "Segment1.0");
		t = segmentation.runGraph(segmentIn, r, 0.5);
		rv->display(t, frameNum, "Segment.5");
		#endif

		// View creatures found in .xml
		cout << "Size: " << reclist.size() << endl;

		LINFO("START > objdet->run() <");
		objs = objdet->run(rv, reclist, segmentIn);

		// create new events with this
		eventSet.initiateEvents(objs, features, imgData);

		// Image< PixRGB<byte > >
//			rv->output(ofs, , frameNum, "Rectangles");
		reclist.clear();
		objs.clear();

		FrameState os = FRAME_NEXT;
		if (!singleFrame)
			os = ofs->updateNext();
		else
			os = FRAME_FINAL;

		if (os == FRAME_NEXT || os == FRAME_FINAL) {

			// save features for each event
			logger->saveFeatures(frameNum, eventSet);

			// classify
			/*bayesClassifier.runEvents(frameNum, eventSet, featureSet);
			float w = (float)d.w()/(float)scaledDims.w();
			float h = (float)d.h()/(float)scaledDims.h();
			Image< PixRGB<byte> > o = rv->createOutput(input,
													   eventSet,
													   40,
													   w, h);

			// display output ?
			rv->display(o, frameNum, "ResultsClassified");*/

			// create MBARI image with metadata from input and original input frame
			if (rv->contrastEnhance())
				output.updateData(preprocess->contrastEnhance(inputRaw), input.getMetaData(), ofs->frame());
			else
				output.updateData(inputRaw, input.getMetaData(), ofs->frame());

			// write out/display anything that's ready
			logger->run(rv, output, eventSet, scaledDims);

			// prune invalid events
			eventSet.cleanUp(ofs->frame());

			// save the input image
			prevInput = input;
		}
		#ifdef DEBUG
		if ( pause.checkPause()) Raster::waitForKey();// || ifs->shouldWait() || ofs->shouldWait()) Raster::waitForKey();
		#endif

		if (os == FRAME_FINAL) {
			 // last frame? -> close everyone
			eventSet.closeAll();
			eventSet.cleanUp(ofs->frame());
			break;
		}
	}
	manager.stop();

	cout << "Reached END" << endl;

	delete itsParser;

	return EXIT_SUCCESS;
}

// Main Functions
// 	- Stores value in temporary 'Creature' Object
void storeValue(BoundingBox& temp, const string& tagName, const string& tagValue) {    // Stores value in temporary 'Creature' Object
	// Convert Bounding Box Value from 'string to int'
	istringstream is(tagValue);
	int dim = 0;
	is >> dim;

	if(tagName == "xmin") {
		temp.xmin = dim;
	} else if(tagName == "ymin") {
		temp.ymin = dim;
	} else if(tagName == "xmax") {
		temp.xmax = dim;
	} else if(tagName == "ymax") {
		temp.ymax = dim;
	}
}

// 	- Get .xml in directory
string getmyXML(const string& descrip, const uint& fnum){
	vector<string>image_path;

	boost::split(image_path, descrip, boost::is_any_of("/"));

	vector<string>filename;
	boost::split(filename, image_path[image_path.size()-1], boost::is_any_of("#"));

	string fs = sformat("%s%06d.xml", filename[0].c_str() , fnum);

	string xml_path="";
	for(int i=1; i< image_path.size()-1; i++){
		xml_path.append(image_path[i]);
		xml_path.append("/");
	}
	xml_path.append(fs);
	return xml_path;
}

// 	- Reads .xml for values in <object> ... </object>
void getObjectValues(XercesDOMParser *itsParser, list<Rectangle>& creatureDims) {
	DOMNodeList *list = NULL;
	DOMDocument *domDocParser = itsParser->getDocument();

	// How many instances of the '<tag>' found
	XMLCh *source = XMLString::transcode("object"); 		// Tag wanted
	list = domDocParser->getElementsByTagName(source);		// Returns list of '<tag>' found

	// Parse through each object to grab values
	for(int i = 0; i < list->getLength(); ++i) {

		DOMNode *node = list->item(i); 						// Gets the ith <object> in the list
		DOMNodeList *length = node->getChildNodes(); 		// Lines counted, including: "<object> ... </object>" = 13 lines total

		BoundingBox temp;

		// Iterate through ea. <tag> in <object> ... </object> to retrieve values
		for(int k = 0; k < length->getLength(); ++k) {
			DOMNode *childNode = length->item(k);

			if(childNode->getNodeType() == DOMNode::ELEMENT_NODE) {						// Ensures we found a <tag>
				string tagNameObj = XMLString::transcode(childNode->getNodeName());		// <Gets Tag Name>
				string tagValueObj = XMLString::transcode(childNode->getTextContent());	// <tag> Gets Value </tag>

				// Grab Bounding Box Dimensions
				//	- Otherwise, get the <name>
				if(tagNameObj == "bndbox") {
					DOMNodeList *dimensions = childNode->getChildNodes();						// Gets all the <tags> in <bndbox>

					for(int j = 0; j < dimensions->getLength(); ++j) {							// Iterate each for dim. value
						DOMNode *dim = dimensions->item(j);

						if(dim->getNodeType() == DOMNode::ELEMENT_NODE) {
							string tagNameBB = XMLString::transcode(dim->getNodeName());		// <Gets Tag Name>
							string tagValueBB = XMLString::transcode(dim->getTextContent());	// <tag> Gets Value </tag>

							storeValue(temp, tagNameBB, tagValueBB);							// Store the dim values 1 by 1
						}
					}

					creatureDims.push_back(Rectangle::tlbrI(temp.xmin, temp.ymin, temp.xmax, temp.ymax));	// Store creature found b/c dim are the last values to collect

					break;
				}
			}
		}
	}
}
