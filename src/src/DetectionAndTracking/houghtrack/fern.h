/******************************************************************************
 * Author: Martin Godec
 *         godec@icg.tugraz.at
 ******************************************************************************/

#ifndef FERN_H_
#define FERN_H_

#include <vector>
#include <deque>
#include <math.h>
#include <iostream>
#include <exception>

#include "features.h"
#include "utilities.h"

#define MAP_SIZE 100.0f
#define MAP_STEP 2.0f

bool sortVotesDesc (const std::pair<CvPoint, float>& A, const std::pair<CvPoint, float>& B);

class RandomTest
{
public:
	RandomTest(const cv::Size& baseSize, unsigned int numChannels)
	{
		channel = static_cast<unsigned int>(randIntFromRange(0,numChannels));
		A = cv::Point(randIntFromRange(0,baseSize.width), randIntFromRange(0,baseSize.height));
		B = cv::Point(randIntFromRange(0,baseSize.width), randIntFromRange(0,baseSize.height));
	}

	inline bool eval( const Features& ft, const cv::Point& base) const
	{
		cv::Mat img = ft.getChannel(channel);
		IplImage ch = img;

		int valA = static_cast<int>(CV_IMAGE_ELEM(&ch, unsigned char, base.y + A.y, base.x + A.x));
		int valB = static_cast<int>(CV_IMAGE_ELEM(&ch, unsigned char, base.y + B.y, base.x + B.x));
		return (valA > valB);
	};

private:
	unsigned int channel;
	cv::Point A, B;
};
 
class VoteTooLargeException: public std::exception {};

class Node
{
public:
	Node(int size, int step) : numPos(1.0f), numNeg(1.0f), probPos(0.5f)
	{
		MapSize = size;
		MapStep = step;
		voteMap = cv::Mat( MapSize, MapSize, CV_32FC1, cv::Scalar(0.0f) );
	};

	~Node()
	{
		voteMap.release();
		buffered.clear();
	};

	Node(const Node& n) : numPos(n.numPos), numNeg(n.numNeg), probPos(n.probPos)
	{
		voteMap = n.voteMap;
		MapSize = n.MapSize;
		MapStep = n.MapStep;
	};

 
	float numPos, numNeg;
	float probPos;

	int MapStep;
	int MapSize;
	cv::Mat voteMap;
	mutable std::vector< std::pair<cv::Point, float> > buffered;

	inline void forget( const double& factor )
	{
		voteMap *= factor;
		numPos *= factor;
		numNeg *= factor;
	}

	inline void clear()
	{
		voteMap = 0.0f;
		numPos = 1.0f;
		numNeg = 1.0f;
		probPos = 0.5f;
		buffered.clear();
	}

	inline void updateMap( const cv::Point& vote )
	{
		buffered.clear();
		int idx = round(static_cast<float>(vote.x) / (float)MapStep) + (float)MapSize/2.0f;
		int idy = round(static_cast<float>(vote.y) / (float)MapStep) + (float)MapSize/2.0f;

		if(idx < 0 || idy < 0 || idx >= MapSize || idy >= MapSize) {
			std::cout << "Vote is out of bounds for Map Size " << MapSize << "( " << idx << "," << idy << " "<< vote.x << "/" << vote.y << ")" << std::endl;
			throw VoteTooLargeException();
		}
		else {
			voteMap.at<float>(idy, idx) += 1.0f;
		}
	}

	inline std::vector< std::pair<cv::Point, float> > getVotes() const
	{
		//if(buffered.size() > 0)
		//	return buffered;

		std::vector< std::pair<cv::Point, float> > ret;
		float avg = static_cast<float>(numPos)/(MapSize*MapSize);

		for(int x = 0; x < MapSize; x++)
		{
			for(int y = 0; y < MapSize; y++)
			{
				float val = voteMap.at<float>(y, x);
				if(val > avg)
				{
					int voteX = static_cast<int>(round((x - MapSize/2.0f) * MapStep));
					int voteY = static_cast<int>(round((y - MapSize/2.0f) * MapStep));

					ret.push_back( std::make_pair( cv::Point(voteX, voteY), probPos * val / numPos ));
				}
			}
		}

		std::sort(ret.begin(), ret.end(), sortVotesDesc);
		ret.resize( std::min(10, (int)ret.size()) );
		//buffered = ret;

		return ret;
	}
};

class Fern
{
public:
	Fern( const cv::Size& baseSize, unsigned int numTests, unsigned int numChannels );
	~Fern();

	void evaluate(Features& ft, const cv::Rect& ROI, cv::Mat& result, int stepSize = 1, float threshold = 0.5f) const;
	void update(Features& ft, const cv::Point& pos, int label, const cv::Point& center);
	void forget(const double& factor);
	void clear();
	int backProject(Features& ft, cv::Mat& projected, const cv::Rect& ROI, cv::Point& center, float radius, int stepSize = 1, float threshold = 0.5f) const;

	cv::Size getBaseSize() const
	{
	    return m_baseSize;
	};

	int getTableSize() const
	{
		return m_nodeTable.size();
	}

	void printStatistics() const
	{
		std::cout << "{ " << m_nodeTable.size() << " / " << std::pow(2, m_numTests) << " } " << std::endl;;

		std::map< unsigned int, Node >::const_iterator it = m_nodeTable.begin();
		while(it != m_nodeTable.end())
		{
			std::cout << "  " << (*it++).second.probPos;
		}
		std::cout << std::endl;
    };


private:
	std::vector< RandomTest > m_tests;
	std::map< unsigned int, Node > m_nodeTable;
	cv::Size m_baseSize;
	unsigned int m_numTests;
	float numPos, numNeg;

	inline unsigned int calcIndex(Features& ft, const cv::Point& point) const
	{
		unsigned int idx = 0x00000000;
		for(unsigned int t = 0; t < m_numTests; t++)
		{
			idx = idx << 1;
			idx &= 0xFFFFFFFE;
			if( m_tests.at(t).eval( ft, cv::Point( point.x - m_baseSize.width/2, point.y - m_baseSize.height/2)) )
				idx |= 0x00000001;
		}
		return idx;
	};
};

bool sortFernsDesc (const Fern& A, const Fern& B);

class Ferns
{
public:
	Ferns()
	{
	};

	~Ferns()
	{
		clear();
		m_ferns.clear();
	};

	void initialize(unsigned int numFerns, const cv::Size& baseSize, unsigned int numTests, unsigned int numChannels)
	{
		clear();
		m_ferns.clear();
		std::cout << " INIT FERNS (" << numFerns << ", " << baseSize.width << "/" << baseSize.height << ", " << numTests << ", " << numChannels << ")" << std::endl;
    		for(unsigned int f = 0; f < numFerns; f++)
    		{
    			m_ferns.push_back( Fern(baseSize, numTests, numChannels) );
    		}
  		isSorted = false;
	}

	void evaluate(Features& ft, const cv::Rect& ROI, cv::Mat& result, int stepSize = 1, float threshold = 0.5f)
	{
		if(!isSorted)
		{
			std::sort(m_ferns.begin(), m_ferns.end(), sortFernsDesc);
			isSorted = true;
		}

		// attention shared memory!
		for(unsigned int f = 0; f < m_ferns.size()/2; f++)
		{
			std::cout.flush();
	        m_ferns.at(f).evaluate(ft, ROI, result, stepSize);
	    }

		GaussianBlur(result, result, cv::Size(5,5), 0);
	};

	int backProject(Features& ft, cv::Mat& projected, const cv::Rect& ROI, cv::Point& center, float radius, int stepSize = 1, float threshold = 0.5f)
	{
		if(!isSorted)
		{
			std::sort(m_ferns.begin(), m_ferns.end(), sortFernsDesc);
			isSorted = true;
		}

		int cnt = 0;

		// attention shared memory!
		for(unsigned int f = 0; f < m_ferns.size()/2; f++)
		{
			std::cout.flush();
	        cnt += m_ferns.at(f).backProject(ft, projected, ROI, center, radius, stepSize);
	    }

		return cnt;
	}

	void update(Features& ft, const cv::Point& pos, int label, const cv::Point& center)
	{
		for(unsigned int f = 0; f < m_ferns.size(); f++)
		{
			m_ferns.at(f).update(ft, pos, label, center);
    	}
		isSorted = false;
    };

	void forget(const double& factor)
	{
		for(unsigned int f = 0; f < m_ferns.size(); f++)
		{
			m_ferns.at(f).forget(factor);
    	}
    };

	void clear()
	{
		for(unsigned int f = 0; f < m_ferns.size(); f++)
		{
			m_ferns.at(f).clear();
    	}
    };

    cv::Size getBaseSize() const
    {
        return m_ferns.at(0).getBaseSize();
    };

	void printStatistics()
	{
		if(!isSorted)
		{
			std::sort(m_ferns.begin(), m_ferns.end(), sortFernsDesc);
			isSorted = true;
		}

		for(unsigned int f = 0; f < m_ferns.size()/2; f++)
		{
		    std::cout << "[" << f << "] ";
			m_ferns.at(f).printStatistics();
			std::cout << std::endl;
    	}
    };


private:
	std::vector< Fern > m_ferns;
	bool isSorted;
};

#endif //FERN_H_
