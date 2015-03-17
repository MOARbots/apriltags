/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <ctime>
#include <iomanip>
#include <time.h>
#include <errno.h>
#include "/usr/include/fcntl.h"
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>

#include "AllHelpers.h"
#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

#define DEVICE0 "/dev/ttyACM0"
#define DEVICE1 "/dev/ttyACM1"
//Device 1 is for debug only
#define SPEED B9600
#define MILLION 1000000L

using namespace std;
using namespace cv;
using april::tag::UINT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;
using helper::GConfig;

std::vector< cv::Ptr<TagFamily> > gTagFamilies;
cv::Ptr<TagDetector> gDetector;

//clock constants
struct timespec start, check;
uint32_t diff, timeval;

//serial stuff
struct termios tio;
struct termios stdio;
struct termios old_stdio;
int tty_fd0, tty_fd1;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0;            // VMIN=0, VTIME=0 'nonblocking' case, returns # byte available

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}




template <typename T>
std::string to_string(T value)
{
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}

//A set of markers to be used for camera pose optimization
struct MarkerSet {
	enum OPTMETHOD{
		OPT_LM = 0, OPT_EPNP, OPT_CAM, OPT_RAW, OPT_NONE
	} optimizeMethod;
	std::string name;
	std::vector<std::string> markerNames;
	std::vector<cv::Point3f> markerCorners;

	//aux data for easy search
	typedef std::map<std::string, int > Name2Rid;
	Name2Rid order; //TagDetection.name() -> row index in the markerCorners of the marker

	MarkerSet() : optimizeMethod(OPT_NONE), name("") {}

	void loadFromConfig(ConfigHelper::ConfigNode& cfn) {
		this->markerCorners.clear();
		this->markerNames.clear();

		int method=(int)cfn["optimizeMethod"];
		this->optimizeMethod = (OPTMETHOD)method;
		this->name = cfn["name"].str();
		cfn["markerNames"] >> markerNames;
		std::vector<double> pts;
		cfn["markerCorners"] >> pts;
		assert(pts.size()==markerNames.size()*4*3);

		cv::Mat(markerNames.size()*4, 3, CV_64FC1, &pts[0])
			.reshape(3).copyTo(markerCorners);
		for(int i=0; i<(int)markerNames.size(); ++i) {
			order[markerNames[i]]=i*4;
		}
	}

	static double reProjError(const std::vector<cv::Point3f>& X,
			const std::vector<cv::Point2f>& U, const cv::Mat& K,
			const cv::Mat& distCoeffs, cv::Mat& r, cv::Mat& t) {
		assert(X.size()>0);
		std::vector<cv::Point2f> V;
		cv::projectPoints(X, r, t, K, distCoeffs, V);
		assert(V.size()==U.size());
		double ret=0;
		for(int i=0; i<(int)V.size(); ++i) {
			const cv::Point2f& u=U[i];
			const cv::Point2f& v=V[i];
			ret+=(u.x-v.x)*(u.x-v.x)+(u.y-v.y)*(u.y-v.y);
		}
		return cv::sqrt(ret/(double)V.size());
	}

	//X: marker corner's 3D coords
	//U: corresponding raw image points (i.e. without undistort yet)
	//K: camera calibration matrix
	//distCoeffs: distortion coefficients
	//R: Rwc, from world (or model) frame to camera frame
	//t: twc, similar to R
	//return re-projection error, i.e. rms
	double optimize(const std::vector<cv::Point3f>& X,
			const std::vector<cv::Point2f>& U, const cv::Mat& K,
			const cv::Mat& distCoeffs, cv::Mat& R, cv::Mat& t) const {
		if(optimizeMethod==OPT_NONE) return -1; //shouldn't reach here, just for safe
		if(optimizeMethod==OPT_EPNP || optimizeMethod==OPT_LM) {
			cv::Mat r;
			cv::solvePnP(X,U,K,distCoeffs,r,t,false,optimizeMethod==OPT_EPNP?CV_EPNP:CV_ITERATIVE);
			cv::Rodrigues(r,R);
			return reProjError(X,U,K,distCoeffs,r,t);
		}
		if (optimizeMethod == OPT_RAW) {
			//TODO: temporarily assumes X are xy-planar and use homography and RTfromKH for OPT_RAW
			std::vector<cv::Point2f> u; //undistorted image points
			cv::undistortPoints(U, u, K, distCoeffs,cv::noArray(),K); //be careful! don't forget the last K!
			std::vector<cv::Point2f> x(X.size());
			for (int i = 0; i < (int) x.size(); ++i) {
				if (X[i].z != 0) {
					logle("[MarkerSet::optimize error] all z coordinate"
							" of MarkerSet::X should be zero if OPT_RAW!");
					return -1;
				}
				x[i] = cv::Point2f(X[i].x, X[i].y);
			}
			cv::Mat Hmi = cv::findHomography(x, u); //u=Hmi*x, Hmi maps marker coords => image coords
			R.create(3, 3, CV_64FC1);
			t.create(3, 1, CV_64FC1);
			helper::RTfromKH(K.ptr<double>(0), Hmi.ptr<double>(0),
					R.ptr<double>(0), t.ptr<double>(0), true);
			cv::Mat r;
			cv::Rodrigues(R,r);
			return reProjError(X,U,K,distCoeffs,r,t);
		}
		if (optimizeMethod == OPT_CAM) { //TODO: add support to optimize both intrinsic and extrinsics together
			logle("[MarkerSet::optimize error] OPT_CAM not supported yet...");
			return -1;
		}
		return -1;
	}
};


const char * finalmsg;
std::string message;
string pwmcommand;
unsigned char c;

struct AprilTagprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog,doRecord;
	bool isPhoto; //whether image source is photo/list or others
	bool useEachValidPhoto; //whether do log for each frame
	bool logVisFrame;
	std::string outputDir;

	bool undistortImage;
	int hammingThresh;

	cv::Mat K, distCoeffs;
	bool no_distortion;

	std::vector<MarkerSet> markerSets;

	virtual ~AprilTagprocessor() {}
	AprilTagprocessor() : doLog(false), doRecord(false), isPhoto(false), no_distortion(true) {
		ConfigHelper::Config& cfg = GConfig::Instance();
		tagTextScale = cfg.get<double>("AprilTagprocessor:tagTextScale",1.0f);
		tagTextThickness = cfg.get<int>("AprilTagprocessor:tagTextThickness",1);
		useEachValidPhoto = cfg.get<bool>("AprilTagprocessor:useEachValidPhoto",false);
		hammingThresh = cfg.get<int>("AprilTagprocessor:hammingThresh",0);
		logVisFrame = cfg.get<bool>("AprilTagprocessor:logVisFrame",false);
		undistortImage = cfg.get<int>("AprilTagprocessor:undistortImage",false);
		gDetector->segDecimate = cfg.get<bool>("AprilTagprocessor:segDecimate",false);

		loadIntrinsics();
		loadMarkerSets();
	}

	void loadMarkerSets() {
		markerSets.clear();
		ConfigHelper::ConfigNode::Ptr cfg_ptr=GConfig::Instance()->getChild("markerSets");
		if(cfg_ptr==0) return;
		ConfigHelper::ConfigNode& cfg=*cfg_ptr;
		markerSets.resize(cfg.size());
		for(int i=0; i<(int)markerSets.size(); ++i) {
			MarkerSet& ms = markerSets[i];
			ms.loadFromConfig(cfg[i]);
		}
	}

	void loadIntrinsics() {
		ConfigHelper::Config& cfg = GConfig::Instance();
		{
			std::vector<double> K_;
			if(!cfg->exist("K") || 9!=(cfg.getRoot()["K"]>>K_)) {
				logli("[loadIntrinsics warn] calibration matrix K"
					" not correctly specified in config!");
				this->undistortImage=false;
				this->no_distortion=true;
				return;
			}
			cv::Mat(3,3,CV_64FC1,&K_[0]).copyTo(K);
			K/=K_[8];
		}

		{
			std::vector<double> distCoeffs_(5,0);
			if(!cfg->exist("distCoeffs") || 5!=(cfg.getRoot()["distCoeffs"]>>distCoeffs_)) {
				logli("[loadIntrinsics warn] distortion coefficients "
					"distCoeffs not correctly specified in config! Assume all zero!");
				for(int i=0; i<5; ++i) distCoeffs_[i]=0;
			}
			double sum=distCoeffs_[0]+distCoeffs_[1]
			+distCoeffs_[2]+distCoeffs_[3]+distCoeffs_[4];
			this->no_distortion=(sum==0);
			if(this->no_distortion) this->undistortImage=false;
			cv::Mat(5,1,CV_64FC1,&distCoeffs_[0]).copyTo(distCoeffs);
		}

		logli("[loadIntrinsics] K="<<K);
		logli("[loadIntrinsics] distCoeffs="<<distCoeffs);
	}

	/**
	 * optimize the markerset based on the detections, output Rwc and twc
	 * return rms error if done, or return -1 if optimization failed
	 */
	double optimizeOn(const std::vector<TagDetection>& detections,
			const MarkerSet& ms, cv::Mat& Rwc, cv::Mat& twc, std::ostream& os) const {
		if (ms.optimizeMethod == MarkerSet::OPT_NONE)
			return -1;

		std::vector<cv::Point2f> U;
		std::vector<cv::Point3f> X;
		U.reserve(detections.size() * 4);
		X.reserve(detections.size() * 4);
		for (int i = 0; i < (int) detections.size(); ++i) {
			const TagDetection &dd = detections[i];
			if (dd.hammingDistance > this->hammingThresh)
				continue;

			MarkerSet::Name2Rid::const_iterator itr = ms.order.find(
					dd.name());
			if (itr == ms.order.end())
				continue;

			for (int k = 0; k < 4; ++k) { //each marker <=> 4 corners <=> 4 2D-3D correspondences
				U.push_back(cv::Point2f(dd.p[k][0], dd.p[k][1]));
				X.push_back(ms.markerCorners[itr->second + k]);
			}
		}
		if ((int) U.size() < 8) {
			logld("[AprilTagFinder::optimizeOn] no need to optimize since less than 8 corners found!");
			return -1;
		}
		double err = ms.optimize(X, U, this->K, this->distCoeffs, Rwc, twc);

		os << ms.name << ".U=" << cv::Mat(U).reshape(1) <<"';" << std::endl;
		os << ms.name << ".X=" << cv::Mat(X).reshape(1) <<"';" << std::endl;
		os << ms.name << ".Rwc=" << Rwc << ";" << std::endl;
		os << ms.name << ".twc=" << twc << ";" << std::endl;
		os << ms.name << ".err=" << err << ";" << std::endl;
		os << ms.name << ".optimizeMethod=" << (int)ms.optimizeMethod << ";" << std::endl;
		os << std::endl;

		return err;
	}
	
	/**
	write the detection in matlab script format:
	tag.id <1x1>, tag.H <3x3>, tag.p <2x4>, tag.c <2x1> [tag.up <2x4>]
	tag.up probably says which way up is on the tag?
	*/
	void writeData(TagDetection &dd,
		std::string varname="tag", bool ud=false)
	{
		const cv::Mat H(3,3,CV_64FC1,(void*)dd.homography[0]);
		const cv::Mat p(4,2,CV_64FC1,(void*)dd.p[0]);
		const cv::Mat c(2,1,CV_64FC1,(void*)dd.cxy);

		//cout<<varname<<".id="<<dd.id<<";\n";
			//<<varname<<".hammingDistance="<<dd.hammingDistance<<";\n"
			//<<varname<<".familyName='"<<dd.familyName<<"';\n"
			//<<varname<<".H="<<H<<";\n"
			//<<varname<<".p="<<p<<"';\n"
			//<<varname<<".c="<<c<<";"<<std::endl;

		if(!ud) return;

		cv::Mat uH(3,3,CV_64FC1);
		std::vector<cv::Point2d> up;
		cv::Point2d uc;
		dd.undistort<double>(this->K, this->distCoeffs, up, uc, uH);
		//cout<<varname<<".uH="<<uH<<"';"<<std::endl;
		//cout<<varname<<".up="<<cv::Mat(up).reshape(1)<<"';"<<std::endl;
		//cout<<varname<<".uc="<<uc<<";"<<std::endl;

		//Using uc.x, uc.y, the central coordinates, and up[0].x, up[0].y, one of the corner coordinates, we now compute the rotation 
		float myX = up[0].x - uc.x;
		float myY = up[0].y - uc.y;
		float myR = 0;
		if ((myX >= 0 && myY >= 0) || (myX >=0 && myY <0)) {
			if (myY == 0) { myY = 0.00001; }
			if (myX == 0) { myX = 0.00001; }//just in case we ever get (un)lucky, don't divide by zero
			myR = atan((myY/myX));
		}
		else if ((myX <0 && myY <0) || (myX <0 && myY >=0) ) {
			if (myY == 0) { myY = 0.00001; }
			if (myX == 0) { myX = 0.00001; }//just in case we ever get (un)lucky, don't divide by zero
			myR = atan((myY/myX)) + M_PI;	
		}
		myR = myR +M_PI/2 + M_PI/4;
		if (myR >2*M_PI){
			myR = myR-2*M_PI;		
		}

		myR = myR * (180.00/M_PI);
		myR = 360-myR;

		/*	We construct the message according to the format
		*	111111		6 bit header, pads out total message size to a round number of bytes
		*	-----		5 bit ID (max 32 decimal)
		*	----------	10 bit y position (max 1023 rounded to the nearest pixel in decimal)
		*	----------	10 bit x position (max 1023 rounded to the nearest pixel in decimal)
		*	---------	9 bit rotation (max 512 rounded to the nearest degree in decimal)
		*/

		//Write the packet format. Not very reconfigurable yet. TODO: make this work with some defines.
		uint8_t bytearray[5];
		bytearray[0] = 63;
		bytearray[0] = bytearray[0] << 2;
		uint8_t IDint = dd.id;
		bytearray[0] = bytearray[0] | IDint >> 3;
		bytearray[1] = IDint << 5;
		uint16_t Yint = uc.y;
		bytearray[1] = bytearray[1] | ( (Yint >> 5) & (0x1F) ); //take myY bits 10 thru 6
		bytearray[2] = Yint << 3; //take myY bits 5 thru 1
		uint16_t Xint = uc.x;
		bytearray[2] = bytearray[2] | ( (Xint >> 7) & (0x07) ); //take myX bits 10 thru 8
		uint16_t myRint = myR;
		bytearray[3] =  Xint << 1; //take myX bits 7 thru 1
		bytearray[3] = bytearray[3] | ( (myRint >> 8) & 0x01 ); //take myRint bit 9
		bytearray[4] = myRint; //take myRint bits 8 thru 1

		//This is where we concatenate and prepare message
		finalmsg = reinterpret_cast <const char *> (bytearray); //does bytearray already point at first element in C?
		
		write(tty_fd0, finalmsg, 5);//This command pushes data to the wixel, if available
		cout << "ID: " << dd.id << ", X: " << uc.x << ", Y: " << uc.y << ", R: " << myR << endl;

	}

/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		PM.tic();
		if(this->undistortImage) cv::undistort(orgFrame,frame,K,distCoeffs);
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		char buf [1024]; //way more than enough, but we're on a laptop, where 1kb of memory is not significant.
		int n = read (tty_fd0, buf, sizeof buf);  // read up to 100 characters if ready to read
		if (n>0) {
		    for (int count = 0; count <=n ; count++ ){
			cout << buf[count];
			cout.flush();
		    }
		}

		//visualization
		int nValidDetections=0;
		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;
			++nValidDetections;

			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		clock_gettime(CLOCK_REALTIME,&check); //check the time

		//logging results
		if(nValidDetections>0 /*&& (doLog || (isPhoto && useEachValidPhoto) || (!isPhoto && doRecord))*/) {
			doLog=false;
			static int cnt=0;
			std::string fileid = helper::num2str(cnt, 5);


			//throttle the frequency with which we send updates to the serial (via method writeData)
			//by changing timeval to be the wait period desired between writes. Full speed isn't always a problem (depends on various factors)
			diff = (check.tv_sec - start.tv_sec)*1000 + double((check.tv_nsec - start.tv_nsec))/MILLION; //convert to seconds!!!
			if( diff <= timeval) { //check to see if diff has elapsed. You can vary diff to suppress the output speed
			    clock_gettime(CLOCK_REALTIME,&start); //reset timer	
			    for(int i=0,j=0; i<(int)detections.size(); ++i) {
				TagDetection &dd = detections[i];
				if(dd.hammingDistance>this->hammingThresh) continue;
				    ++j;//note matlab uses 1-based index
				    writeData(dd, "tag", !this->undistortImage && !this->no_distortion);
			    }
			}		
			++cnt;
		}
	}

	void handle(char key) {
		switch (key) {
		case 'g': //Send a g command, for starting robots
			pwmcommand = "\x67"; //send 'g'	
			write(tty_fd0,pwmcommand.c_str(),strlen(pwmcommand.c_str()));
			cout << "Sent the start command, 'g', to the robot." << endl;
			break;
		case 'c': //Send a c command, for continuing robots
			pwmcommand = "\x63"; //send 'c'	
			write(tty_fd0,pwmcommand.c_str(),strlen(pwmcommand.c_str()));
			cout << "Sent the continue command, 'c', to the robot." << endl;
			break;
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] gDetector.segDecimate="<<gDetector->segDecimate); break;
		case 'l':
			doLog=true; break;
		case 'r':
			doRecord=true; break;
		case '1':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_DEBUG; break;
		case '2':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO; break;
		case 'h':
			cout<<"d: segDecimate\n"
				"l: do log\n"
				"r: do record\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilTagprocessor

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" [url] [tagfamiliesID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Combination of TagFamily ID: 014 (use tagFamily 0, 1 and 4)"<<endl;
	cout<<"default tagfamiliesID=0"<<endl;
	cout<<"default url=camera://0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"url=photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"url=camera://0?w=640?h=480?f=60\n";
	cout<<"url=video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
#ifdef USE_FLYCAP
	cout<<"url=pgr://0?v=5?f=4"<<endl;
#endif
}

int main(const int argc, const char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;


	if (argc > 1) {
		std::string arg1(argv[1]);
		if (arg1 == "-h" || arg1 == "/?" || arg1 == "--help") {
			usage(argc, argv);
			return -1;
		}
	}

	tty_fd0 = open (DEVICE0, O_RDWR | O_NOCTTY | O_SYNC);
	fcntl(tty_fd0, F_SETFL, FNDELAY); //necessary for immediate return on read functions
	if (tty_fd0 < 0) {
            printf ("error %d opening %s: %s", errno, DEVICE0, strerror (errno));
	}
	set_interface_attribs (tty_fd0, SPEED, 0);  // set speed, 8n1 (no parity)
	set_blocking (tty_fd0, 0);                // set no blocking

	clock_gettime(CLOCK_REALTIME,&start); //initialize clock
	timeval=0;

	ConfigHelper::Config& cfg = GConfig::Instance();
	if(!cfg.autoLoad("AprilTagFinder.cfg",DirHelper::getFileDir(argv[0]))) {
		logli("[main] no AprilTagFinder.cfg file loaded");
	}
	if(argc>1) {
		cfg.reset(argc-1, argv+1);
	}
	logli("[main] final Config:");
	cfg->print(std::cout);
	logli("");

	cv::Ptr<ImageSource> is = helper::createImageSource(cfg.get("url","camera://0"));
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid = cfg.get("tagfamiliesID","0"); //defaul Tag16h5
	TagFamilyFactory::create(tagid, gTagFamilies);
	if(gTagFamilies.size()<=0) {
		tagle("create TagFamily failed all! exit...");
		return -1;
	}

	gDetector = new TagDetector(gTagFamilies);
	if(gDetector.empty()) {
		tagle("create TagDetector fail!");
		return -1;
	}

	AprilTagprocessor processor;
	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	processor.outputDir = cfg.get("outputDir", is->getSourceDir());
	logli("[main] detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
		cfg.get<bool>("ImageSource:pause", is->getPause()),
		cfg.get<bool>("ImageSource:loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
