//
//  UGENs.h
//
//  Created by Dr. Courtney Brown, 2014-2018
//  A set of unit generators/filters and a framework for use in motion capture class
//  These ugens are for retrieving motion data features and signal conditioning.
//  Modified 10-22-2018 for utility of mocap class.
//

#ifndef UGENs_h
#define UGENs_h

using namespace ci;
using namespace gl;

#include <iostream>
using namespace std;

namespace CRCPMotionAnalysis {
    
    static const double SR = 51.2; // Sample rate -- this may vary for sensors...
    
    
    //abstract class of all ugens
    class UGEN
    {
    public:
        UGEN(){};
        virtual std::vector<ci::osc::Message> getOSC()=0;//<-- create/collect OSC messages that you may want to send to another program or computer
        virtual void update(float seconds=0)= 0; //<-- do the meat of the signal processing / feature extraction here
    };
    
    class SignalAnalysis : public UGEN //any data that will analyze a signal
    {
    protected:
        
        std::vector<MocapDeviceData *> data1; //stores the ugens
        std::vector<MocapDeviceData *> data2; //stores the ugens
        
        SignalAnalysis *ugen, *ugen2  ; // 2 ugens in this data, could just create a vector of ugens (more buffers of data to modify in update)
        std::vector<SignalAnalysis *> ugens;
        
        int buffersize;
        
        bool useAccel;
        bool useGry;
        bool useQuart;
        
        //find the average over a window given an input & start & index of a buffer
        virtual double findAvg(std::vector<double> input, int start, int end)
        {
            double N = end - start;
            double sum = 0;
            for(int i=start; i<end; i++)
            {
                sum += input[i];
            }
            return (sum/N);
        };
        
    public:
        //create analysis with pointers to prev. signal analyses which provide input data + how much data is in the buffer
        //note:  this takes  data from maximum 2 other signal analyses -- if you anticipated more, would want to redesign std::vector<SignalAnalysis *> for max flexibility
        SignalAnalysis(SignalAnalysis *s1 = NULL, int bufsize=48, SignalAnalysis *s2 = NULL)
        {
            ugens.push_back(s1);
            ugens.push_back(s2);
            
            ugen = s1;
            ugen2 = s2;
            
            setBufferSize(bufsize);
            
            useAccel=true;
            useGry=false;
            useQuart=false;
            
        };
        
        //how many new samples do we need to process
        inline virtual int getNewSampleCount()
        {
            for (int i = 0; i < ugens.size(); i++)
            {
                int max = 0;
                if (ugens[i] != NULL && ugen[i].getNewSampleCount() > max)
                {
                    max = ugen[i].getNewSampleCount();
                    return max;
                } else { return 0; }
            }
            
            if(ugen != NULL && ugen2==NULL)
                return ugen->getNewSampleCount();
            else if(ugen != NULL && ugen2!=NULL)
                return std::max(ugen->getNewSampleCount(), ugen2->getNewSampleCount());
            else return 0;
        }
        
        //which signals are we dealing with or using?
        inline void processAccel(bool a)
        {
            useAccel = a;
        };
        
        inline void processGry(bool g)
        {
            useGry = g;
        };
        
        inline void processQuart(bool q)
        {
            useQuart=q;
        };
        
        inline virtual void setBufferSize(int sz)
        {
            buffersize = sz;
        };
        
        virtual inline std::vector<MocapDeviceData *> getBuffer(){
            return data1;
        };
        
        virtual inline std::vector<MocapDeviceData *> getBuffer2(){
            return data2;
        };
        
        int getBufferSize(){return buffersize;};
        
        virtual void update(float seconds=0)
        {
            //get the buffers from the ugen inputs
            for(int i = 0; i < ugens.size(); i++)
            {
                //if (ugen[i] != NULL) data[i] = ugens[i]->getBuffer();
            }
            if( ugen != NULL ) data1 = ugen->getBuffer();
            if( ugen2 != NULL ) data2 = ugen2->getBuffer();
        };
    
//command to the compiler
#ifdef USING_ITPP //if we are using the it++ library which allows some signal processing + matlab functionalty then compile this wrapper/conversion function --> note: we're not.. as of yet.
        std::vector<itpp::vec> toITPPVector(std::vector<MocapDeviceData *> sdata) //to the it++ vector data type
        {
            std::vector<itpp::vec> newVec;
            for(int i=0; i<20; i++)
            {
                itpp::vec v(sdata.size());
                for(int j=0; j<sdata.size(); j++)
                {
                    v.set( j, sdata[j]->getData(i));
                }
                newVec.push_back(v);
            }
            return newVec;
        };
        std::vector<float> toFloatVector(itpp::vec input) //from the it++ vector data type
        {
            std::vector<float> output;
            for(int i=0; i<input.length(); i++)
            {
                output.push_back(input[i]);
            }
            return output;
        };
#endif

        //finds average in buffer using the MotionCaptureData instead of float vector as before
        virtual double findAvg(std::vector<MocapDeviceData *> input, int start, int end, int index)
        {
            double N = end - start;
            double sum = 0;
            for(int i=start; i<end; i++)
            {
                sum += input[i]->getData(index);
            }
            return (sum/N);
        };
    };
    



//This ugen only gets/receives inputs from sensors so sends nothing -- not an output bc it doesn't modify its inputs
//TODO: this filters for acc data ONLY, so turn on options, etc. if needed
class InputSignal : public SignalAnalysis
{
protected:
    int ID1;
    bool isPhone;
    bool isWiiMote;
    SensorData *sensor  ;

public:
    InputSignal(int idz, bool phone=false, SignalAnalysis *s1 = NULL, int bufsize=48, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
    {
        ID1= idz;
        isPhone = phone;
    };
    
    //not sending any OSC currently
    virtual std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        
        return msgs;
    };
    
    void setInput( SensorData *s1 )
    {
        sensor = s1;
    };
    
    virtual int getNewSampleCount()
    {
        return sensor->getNewSampleCount();
    };
    
    //puts valid mocap data in buffers for other ugens.
    //work with one buffer at a time in each frame
    virtual void update(float seconds=0)
    {
        data1.clear(); //clear what is in data (previous stuff)
        if( sensor != NULL )
        {
            std::vector<MocapDeviceData *> data = sensor->getBuffer(buffersize);
            for(int i =0; i<data.size(); i++) //filters out dummy data
            {
                if( data[i]->getData(MocapDeviceData::DataIndices::ACCELX) != NO_DATA )
                {
                    data1.push_back(data[i]); //get the new data if it is what we need
                 }
            }
        }
    };
    
    virtual std::vector<MocapDeviceData *> getBuffer(){
        return data1;
    };
    
};

    //has output signals that it modifies  & forwards on to others
    //only handles one stream of data...
    class OutputSignalAnalysis : public SignalAnalysis
    {
    protected:
        std::vector<MocapDeviceData *> outdata1;
        
        void eraseData()
        {
            for( int i=0; i<outdata1.size(); i++ )
                delete outdata1[i];
            
            outdata1.clear();
        };
    public:
        
        OutputSignalAnalysis(SignalAnalysis *s1, int bufsize, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
        {
            
        }
        
        //puts in accel data slots -- all other data left alone -- ALSO only
        void toOutputVector( std::vector<float> inputX, std::vector<float> inputY, std::vector<float> inputZ )
        {
            for(int i=0; i<inputX.size(); i++)
            {
                MocapDeviceData *data = new MocapDeviceData();
                data->setData(MocapDeviceData::DataIndices::INDEX, data1[i]->getData(MocapDeviceData::DataIndices::INDEX));
                data->setData(MocapDeviceData::DataIndices::TIME_STAMP, data1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP));
                data->setData(MocapDeviceData::DataIndices::ACCELX, inputX[i]);
                data->setData(MocapDeviceData::DataIndices::ACCELY, inputY[i]);
                data->setData(MocapDeviceData::DataIndices::ACCELZ, inputZ[i]);
                outdata1.push_back(data);
            }
        };
        
        virtual void update(float seconds = 0){
            eraseData();
            SignalAnalysis::update(seconds);
        };
        
        virtual std::vector<MocapDeviceData *> getBuffer(){
            return outdata1;
        };
    
    };
    
    //this class averages data over a window - it is -- ***this only operates on accel. data***
    //--> but it can v. easily be expanded to include other data -- see commented out functionality
    class AveragingFilter : public OutputSignalAnalysis
    {
    protected:
        int windowSize; //amount of data we are averaging
    public:
        
        AveragingFilter(SignalAnalysis *s1, int w=10, int bufsize=48 ) : OutputSignalAnalysis(s1, bufsize)
        {
            windowSize = w;
        };
        
        //I'm gonna be shot for yet another avg function
        float mocapDeviceAvg(std::vector<MocapDeviceData *> data, int start, int end, int index )
        {
            double sum = 0;
            int valCount = 0;
            for( int j=start; j<=end; j++ )
            {
                if(data1[j]->getData(index) != NO_DATA)
                {
                    sum += data1[j]->getData(index);
                    valCount++;
                }
            }
            if(valCount==0) return NO_DATA;
            else return sum / double( valCount );
        };
        
        //perform the averaging here...
        virtual void update(float seconds=0)
        {
            OutputSignalAnalysis::update(seconds);
            if( data1.size() < buffersize) return ;
            
            for( int i=0; i<data1.size(); i++ )
            {
                int start = std::max(0, i-windowSize);
                int end = i;
                MocapDeviceData *mdd= new MocapDeviceData();
                mdd->setData(MocapDeviceData::DataIndices::INDEX, data1[i]->getData(MocapDeviceData::DataIndices::INDEX));
                mdd->setData(MocapDeviceData::DataIndices::TIME_STAMP, data1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP));
                
                if( useAccel )
                {
                    mdd->setData(MocapDeviceData::DataIndices::ACCELX, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELX));
                    mdd->setData(MocapDeviceData::DataIndices::ACCELY, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELY));
                    mdd->setData(MocapDeviceData::DataIndices::ACCELZ, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELZ));
                }
      
//    untested functionality in this domain...
//                if( useGry )
//                {
//                    mdd->setData(MotionDeviceData::DataIndices::GYROX, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROX));
//                    mdd->setData(MotionDeviceData::DataIndices::GYROY, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROY));
//                    mdd->setData(MotionDeviceData::DataIndices::GYROZ, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROZ));
//                }
//
//                if( useQuart )
//                {
//                    mdd->setData(MotionDeviceData::DataIndices::QX, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QX));
//                    mdd->setData(MotionDeviceData::DataIndices::QY, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QY));
//                    mdd->setData(MotionDeviceData::DataIndices::QZ, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QZ));
//                    mdd->setData(MotionDeviceData::DataIndices::QA, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QA));
//                }
                outdata1.push_back(mdd);
            }
        }
        
        //if you wanted to send the signal somewhere
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            for (int i = 0; i < outdata1.size(); i++)
            {
                ci::osc::Message m;
                m.setAddress(  "/mocap/points"  ); //"/mydata/shit/x"
                m.append( (float)outdata1[i]->getData(2) ); //x pos
                m.append( (float)outdata1[i]->getData(3) ); //y pos
                msgs.push_back( m  );
            }
            return msgs;
        };
    };
    
    class Derivative : public OutputSignalAnalysis
    {
    protected:
        std::vector<float> derivative;

    public:
        Derivative(SignalAnalysis *s1, int bufsize=48 ) : OutputSignalAnalysis(s1, bufsize)
        {
            
        }

        virtual void update(float seconds = 0)
        {

            OutputSignalAnalysis::update(seconds);
            if( data1.size() < buffersize) return ;
            
            derivative.clear();
            outdata1.clear();
            
            for (int i = 1; i < data1.size(); i++)
            {
                float distX = distance(  data1[i]->getData(MocapDeviceData::DataIndices::ACCELX),
                                        data1[i - 1]->getData(MocapDeviceData::DataIndices::ACCELX)  );
                float distY = distance(  data1[i]->getData(MocapDeviceData::DataIndices::ACCELY),
                                      data1[i - 1]->getData(MocapDeviceData::DataIndices::ACCELY)  );
                
                vec2 p(distX, distY);
                
//                float distZ = distance(  data1[i]->getData(MocapDeviceData::DataIndices::ACCELZ),
//                                      data1[i - 1]->getData(MocapDeviceData::DataIndices::ACCELZ)  );
                
                derivative.push_back( distX );
                derivative.push_back( distY );
                
            }
            
            for( int i=2; i < derivative.size(); i++ )
            {
                MocapDeviceData *mdd= new MocapDeviceData();
                mdd->setData(MocapDeviceData::DataIndices::INDEX, derivative[i - 1]);
                mdd->setData(MocapDeviceData::DataIndices::TIME_STAMP, derivative[i - 1]);

                if( useAccel )
                {
                    mdd->setData(  MocapDeviceData::DataIndices::ACCELX, derivative[i - 1]  * 100 );
                    mdd->setData(  MocapDeviceData::DataIndices::ACCELY, derivative[i] * 100  );
                    //mdd->setData(MocapDeviceData::DataIndices::ACCELZ, derivative[i]);
                }
                outdata1.push_back(mdd);
            }
            
//            cout << "outdata1: ";
//            for (int i = 0; i < outdata1.size(); i++)
//            {
//                cout << outdata1[i]->getData(i) << " ";
//            }
//            cout << endl;
//
//
//            cout << "der: ";
//            for (int i = 0; i < derivative.size(); i++)
//            {
//
//                cout << derivative[i] << " ";
//            }
//            cout << endl;
        }
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            for (int i = 0; i < outdata1.size(); i++)
            {
                ci::osc::Message m;
                m.setAddress(  "/mocap/derivative/"  ); //"/mydata/shit/x"
                m.append( (float)outdata1[i]->getData(2) ); //x pos
                m.append( (float)outdata1[i]->getData(3) ); //y pos
                msgs.push_back( m  );
            }
            return msgs;
        };
    
    };
    
    //this class visualizes incoming motion data
    class MocapDataVisualizer : public SignalAnalysis
    {
    protected :
       int  maxDraw; //how far back in history to draw
        
        std::vector<ci::vec2> points;
        std::vector<float>  alpha;
        
    public:
        MocapDataVisualizer(OutputSignalAnalysis *s1 = NULL, int _maxDraw=25,int bufsize=48, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
        {
            maxDraw = _maxDraw;
        };
        
        //add data as screen positions and color alphas.
        void update(float seconds = 0)
        {
            std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
            if(buffer.size()<maxDraw) return; //ah well I don't want to handle smaller buffer sizes for this function. feel free to implement that.
            
            points.clear();
            alpha.clear();
            
            //one class that did one thing -> two instances of one class with different inputs
            //signal analysis class, inherit from it
            
            for(int i=buffer.size()-maxDraw; i<buffer.size(); i++)
            {
                points.push_back(ci::vec2(buffer[i]->getData(MocapDeviceData::DataIndices::ACCELX)*ci::app::getWindowWidth(), buffer[i]->getData(MocapDeviceData::DataIndices::ACCELY)*ci::app::getWindowHeight()));
                //cout << buffer[i]->getData(MocapDeviceData::DataIndices::ACCELZ ) << endl;
                alpha.push_back(buffer[i]->getData(MocapDeviceData::DataIndices::ACCELZ));
            }
            
            //printVectors();
            
        };
        
        
        //visualize the data
        void draw()
        {
            //float circleSize = 2;
                for(int i=1; i<points.size(); i++)
                {
                    drawLine(points[i-1], points[i]);
                    //drawSolidCircle(points[i], circleSize);
                }
            
            points.clear();
        };
        
        //if you wanted to send something somewhere... prob. not
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
//            for (int i = 0; i < points.size(); i++)
//            {
//                ci::osc::Message m;
//                m.setAddress(  "/mocap/"  ); //"/mydata/shit/x"
//                m.append( points[i].x );
//                m.append( points[i].y );
//                //m.append( data1[i]->getData(MocapDeviceData::DataIndices::INDEX) );
//                msgs.push_back( m  );
//            }
            return msgs;
        };


};

};

#endif /* UGENs_h */
