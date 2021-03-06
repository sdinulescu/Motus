/*
 * Created by Courtney Brown
 * Gets data from different sensors
 * Creates Signal Processing Tree for phone and wii sensors
 */


//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

//includes for background subtraction
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp" //new include for this project


#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h" //needed for capture
#include "cinder/Log.h" //needed to log errors

#include <sstream>

#include "CinderOpenCV.h"

#include "Osc.h"

#include "MotionCaptureData.h"
#include "Sensor.h"
#include "UGENs.h"
#include "MeasuredEntities.h"
#include "SquareGenerator.hpp"
#include "MovieSaver.h"


//orbbec stuff
#include <cstdio>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "Astra.h"

#include "Osc.h" //add to send OSC

#include "CinderOpenCV.h"

#include "Blob.h"


#define LOCALPORT 8886
#define LOCALPORT2 8887
#define DESTHOST "127.0.0.1"
#define DESTPORT 8888
#define MAX_CORNERS 300 //sets up a constant
#define QUALITY_LEVEL 0.005 //whatever corner you find is good
#define MIN_DISTANCE 3 //how far away the corners have to be from each other
#define ELAPSED_FRAMES 300 //number of elapsed frames to check features

#define NUMBER_OF_SQUARES 20

//osc messages
#define ACCEL_ADDR "/wii/accel"
#define SYNTIEN_MESSAGE "/syntien/motion/1/scope1"
#define WIIMOTE_ACCEL_MESSAGE_PART1 "/wii/"
#define WIIMOTE_ACCEL_MESSAGE_PART2 "/accel/pry"
#define WIIMOTE_BUTTON_1 "/wii/1/button/1"

#define MAX_NUM_OF_WIIMOTES 6 //limitation of bluetooth class 2
#define PHONE_ID "7" //this assumes only one phone using Syntien or some such -- can modify if you have more...

#define SAMPLE_WINDOW_MOD 300
#define MAX_FEATURES 300

using namespace ci;
using namespace ci::app;
using namespace std;

using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;

//This class demonstrates a 'hello, world' for the signal processing tree paradigm for motion capture
//Receives wiimote data, puts an averaging filter on it, then draws the data
//Also, w/o signal processing paradigm, this program has the functions to draw a phone but not implemented currently
class MotusApp : public App {
  public:
    MotusApp();
    ~MotusApp();
    
	void setup() override;
//    void mouseDown( MouseEvent event ) override;
    void keyDown( KeyEvent event ) override;

	void update() override;
	void draw() override;
    
  protected:
    CaptureRef                 mCapture;
    gl::TextureRef             mTexture;
    SurfaceRef                 mSurface;
    
    cv::Mat mPrevFrame, mCurrFrame, mBGFrame, mFrameDiff;
    vector<cv::Point2f> mPrevFeatures, mFeatures;
    vector<uint8_t> mFeatureStatuses;
    vector<float> errors; //unsigned integers
    
    osc::SenderUdp             mSender;
    
     SquareFrameDiff squareDiff;
    
    void sendOSC(std::string addr,  float posX, float posY, float vel, float acc);

    Receiver mReceiver; //new
    std::map<uint64_t, protocol::endpoint> mConnections; //new
    
    //below for printing out iphone info
    std::vector<ci::vec2> points;
    std::vector<float>  alpha;
    
    void updatePhoneValues(const osc::Message &message);
    void updateWiiValues(const osc::Message &message);
    void addPhoneAndWiiData(const osc::Message &message, std::string _id);

    CRCPMotionAnalysis::SensorData *getSensor( std::string _id, int which ); // find sensor or wiimote in list via id
    std::vector<CRCPMotionAnalysis::SensorData *> mSensors; //all the sensors which have sent us OSC -- well only wiimotes so far
    std::vector<CRCPMotionAnalysis::Entity *> mEntities;  //who are we measuring? change name when specifics are known.
    
    float seconds;
    bool newFrame;
    
    cv::Mat frameDifferencing(cv::Mat frame);
    void frameDifference();
    void updateFrameDiff();
    void sendSquareOSC(string address, float maxSquareMotion, float maxSquareX, float maxSquareY );
    
    //Stuff to read the astra stream
    SampleFrameListener listener;
    astra::StreamSet streamSet;
    astra::StreamReader reader;
    
    //to save our capture
    MovieSaver *saver = NULL;
};

MotusApp::MotusApp() : mSender(LOCALPORT, DESTHOST, DESTPORT), mReceiver( LOCALPORT2 )
{
    
}
MotusApp::~MotusApp() {
    astra::terminate();
}

//outdated vestige
void MotusApp::sendOSC(std::string addr, float posX, float posY, float vel, float acc)
{
    osc::Message msg;
    msg.setAddress(addr);
    msg.append(posX); //adds a parameter
    msg.append(posY);
    msg.append(vel);
    msg.append(acc);
    
    mSender.send(msg);
    
}

//this has not been implemented into signal tree paradigm yet. ah well. TODO: implement as such
void MotusApp::updatePhoneValues(const osc::Message &message)
{
    addPhoneAndWiiData(message, PHONE_ID);
}

//gets data from osc message then adds wiimote data to sensors
void MotusApp::addPhoneAndWiiData(const osc::Message &message, std::string _id)
{
    CRCPMotionAnalysis::MocapDeviceData *sensorData = new CRCPMotionAnalysis::MocapDeviceData;
    std::string dID = _id ;
    int which = std::atoi(_id.c_str()); //convert to int
    
    CRCPMotionAnalysis::SensorData *sensor = getSensor( _id, which );
    
    //set time stamp
    sensorData->setData( CRCPMotionAnalysis::MocapDeviceData::DataIndices::TIME_STAMP, seconds ); //set timestamp from program -- synch with call to update()
    
    //add accel data
    for(int i= 0; i<3; i++)
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ACCELX+i, message.getArgFloat(i));
    
    sensor->addSensorData(sensorData); //hands it to the sensors
}

//return sensor with id & or create one w/detected id then return that one
CRCPMotionAnalysis::SensorData *MotusApp::getSensor( std::string _id, int which )
{

        bool found = false;
        int index = 0;
        
        while( !found && index < mSensors.size() )
        {
            found = mSensors[index]->same( _id, which );
            index++;
        }
    
        if(found)
        {
            return mSensors[index-1];
        }
        else
        {
            CRCPMotionAnalysis::SensorData *sensor = new CRCPMotionAnalysis::SensorData( _id, which ); //create a sensor
            mSensors.push_back(sensor);

            //add to 'entity' the data structure which can combine sensors. It currently only has one body part so it is simple.
            int entityID  = mSensors.size()-1;
            CRCPMotionAnalysis::Entity *entity = new CRCPMotionAnalysis::Entity();
            entity->addSensorBodyPart(entityID, sensor, CRCPMotionAnalysis::Entity::BodyPart::HAND );
            mEntities.push_back(entity);
            return sensor; 
        }
}


//finds the id of the wiimote then adds the wiidata to ugens
void MotusApp::updateWiiValues(const osc::Message &message)
{
    //get which wii
    std::string addr = message.getAddress();
    std::string pt1 = WIIMOTE_ACCEL_MESSAGE_PART1;
    int index = addr.find_first_of(pt1);
    std::string whichWii = addr.substr(index+pt1.length(), 1);
    addPhoneAndWiiData(message, whichWii);
}

//set up osc
void MotusApp::setup()
{
    //initialize astra
    astra::initialize();
    reader = streamSet.create_reader();
    reader.stream<astra::PointStream>().start();
    reader.stream<astra::DepthStream>().start();
    reader.add_listener(listener);
    
    std::cout << "depthStream -- hFov: "
    << reader.stream<astra::DepthStream>().hFov()
    << " vFov: "
    << reader.stream<astra::DepthStream>().vFov()
    << std::endl;
    
    //makes sure this is a valid stream
    std::cout << "valid:" << streamSet.is_valid() <<std:: endl;
    
    //initialize saver
        fs::path saveFilePath = getSaveFilePath();
        saver = new MovieSaver(saveFilePath);
    
   //square code
    squareDiff.divideScreen(NUMBER_OF_SQUARES);
    
    //webcam code
//    try
//    {
//        mCapture = Capture::create(640, 480); //creates the CamCapture using default camera (webcam)
//        mCapture->start(); //start the CamCapture (starts the webcam)
//    } catch (ci::Exception &e)
//    {
//        CI_LOG_EXCEPTION("Failed to init capture", e); //if it fails, it will log on the console
//    }
    
    try{
        mSender.bind();
    }
    catch( osc::Exception &e)
    {
        CI_LOG_E( "Error binding" << e.what() << " val: " << e.value() );
        quit();
    }
    
    //ListenerFn = std::function<void( const Message &message )>
    mReceiver.setListener( SYNTIEN_MESSAGE, [&]( const osc::Message &msg ){
        updatePhoneValues(msg); //listening for phone
    });
    
    for (int i=0; i<MAX_NUM_OF_WIIMOTES; i++) //receiving for all wiimotes that we are getting osc from
    {
        std::stringstream addr;
        addr << WIIMOTE_ACCEL_MESSAGE_PART1 << i << WIIMOTE_ACCEL_MESSAGE_PART2;
        mReceiver.setListener( addr.str(), [&]( const osc::Message &msg ){
            updateWiiValues(msg); //listening for wiimote
        });
    }

    try {
        // Bind the receiver to the endpoint. This function may throw.
        mReceiver.bind();
    }
    catch( const osc::Exception &ex ) {
        CI_LOG_E( "Error binding: " << ex.what() << " val: " << ex.value() );
        quit();
    }
    
    // UDP opens the socket and "listens" accepting any message from any endpoint. The listen
    // function takes an error handler for the underlying socket. Any errors that would
    // call this function are because of problems with the socket or with the remote message.
    mReceiver.listen(
                     []( asio::error_code error, protocol::endpoint endpoint ) -> bool {
                         if( error ) {
                             CI_LOG_E( "Error Listening: " << error.message() << " val: " << error.value() << " endpoint: " << endpoint );
                             return false;
                         }
                         else
                             return true;
                     });
    
}



void MotusApp::keyDown( KeyEvent event )
{

}

cv::Mat MotusApp::frameDifferencing(cv::Mat frame) //frame differencing with currFrame
{
    cv::Mat input, outputImg;
    cv::GaussianBlur(mCurrFrame, input, cv::Size(5, 5), 0);
    cv::absdiff(input, frame, outputImg);
    cv::threshold(outputImg, outputImg, 50, 255, cv::THRESH_BINARY);
    return outputImg;
}

void MotusApp::frameDifference() //for differencing with prev frame
{
    if(!mSurface || !mCurrFrame.data) return ;
    if (mPrevFrame.data) { mFrameDiff = frameDifferencing(mPrevFrame); }
    mPrevFrame = mCurrFrame;
}

void MotusApp::updateFrameDiff()
{
    if (mCapture && mCapture->checkNewFrame()) //is there a new image?
    {
        mSurface = mCapture->getSurface(); //will get its most recent surface/whatever it is capturing
        mCurrFrame = toOcv( Channel( *mSurface ) );
        if ( !mTexture ) //if texture doesn't exist
        {
            mTexture = gl::Texture::create( *mSurface ); //create a texture from the surface that we got from the camera
        } else {
            mTexture->update( *mSurface ); //if it does exist, update the surface
        }
    }
    
    frameDifference();
    
    if (mFrameDiff.data) { squareDiff.countPixels(mFrameDiff); } //count the pixels for frame differencing
    
    sendSquareOSC( "/mocap/square", squareDiff.getMotionValue(), squareDiff.getMaxXValue(), squareDiff.getMaxYValue() );
    
}

void MotusApp::sendSquareOSC( string address, float maxSquareMotion, float maxSquareX, float maxSquareY ) //sends Osc messages containing square values
{
    osc::Message msg;
    msg.setAddress(address); //sets address of the message
    msg.append(maxSquareMotion); //adds a parameter to the message
    msg.append(maxSquareX);
    msg.append(maxSquareY);
    
    //cout << "maxMotion: " << maxSquareMotion << " maxX: " << maxSquareY << " maxY: " << maxSquareY << endl;
    
    mSender.send(msg);
}

//update entities and ugens and send OSC, if relevant
void MotusApp::update()
{
    
    astra_status_t status = astra_temp_update();
    
    //checks if there is a new frame, if so, updates the surface
    newFrame = listener.newFrame();
    if(newFrame)
    {
        mSurface = listener.getNewFrame();
                if(saver)
                    saver->update(mSurface);
    } else return;
    
    seconds = getElapsedSeconds(); //clock the time update is called to sync incoming messages
    
    mCurrFrame.copyTo(mPrevFrame);
    
    //computer vision mocap code
    mCurrFrame = toOcv(Channel(*mSurface));
    
    //resize curFrame to windowHeight & width
    cv::resize(mCurrFrame, mCurrFrame, cv::Size( getWindowWidth(), getWindowHeight()  ));
    cv::blur(mCurrFrame, mCurrFrame, cv::Size(9,9));
    
//    if(mPrevFrame.data){
//        mDiffFrame = frameDifference();
//    }

    //update sensors
    for(int i=0; i<mSensors.size(); i++)
    {
        mSensors[i]->update(seconds);
    }
    //update all entities
    for(int i=0; i<mEntities.size(); i++)
    {
        mEntities[i]->update(seconds);
    }

    //send OSC from the entities -- after all are updated..
    for(int i=0; i<mEntities.size(); i++)
    {
        std::vector<osc::Message> msgs = mEntities[i]->getOSC();
        for(int i=0; i<msgs.size(); i++)
        {
            mSender.send(msgs[i]);
        }
    }
    
    //framedifferencing
    updateFrameDiff();
    

}

//draw the entities
void MotusApp::draw()
{
    gl::clear( Color( 1, 1, 1 ) );

    //draw frame differencing
    //gl::draw(mTexture);
    squareDiff.displaySquares();
//
//    //draw wiimote stuff
//    for(int i=0; i<mEntities.size(); i++)
//    {
//        mEntities[i]->draw();
//    }
    
    //draws the camera surface
    if( mSurface )
    {
        //    note: the size of the surface/frame is about 25% of the window frame, so Rectf tells it to draw so that it fills the screen
        gl::draw( gl::Texture::create( *mSurface ), ci::Rectf(0, 0, getWindowWidth(), getWindowHeight()) );
    }
 
}

CINDER_APP( MotusApp, RendererGl )
