//
//  MeasuredEntities.h
//  Where we are going to put all of our UGENs together
//
//  Created by courtney on 10/22/18.
//

#ifndef MeasuredEntities_h
#define MeasuredEntities_h

namespace CRCPMotionAnalysis {

//who is being measured? or a collection or who or what's? --
//this sets up all the ugens and filters and organizes them according to what they represent in real life. For this example, I am only
//defining a hand. In order to use for yourselves, inherit from this class & add to BodyPart -- or rename it if you are tracking a non-human or non-animal.
class Entity : public UGEN
{
protected:
    std::vector<UGEN * > hand; //assuming sensor is being held by the hand
    std::vector<UGEN * > foot; //assuming sensor is being held by the hand
     MocapDataVisualizer *handVisualizer; //assuming sensor is being held by the hand -- this will have to be a vector for mo' body parts, etc.
    MocapDataVisualizer *der1Visualizer; //assuming sensor is being held by the hand -- this will have to be a vector for mo' body parts, etc.
    MocapDataVisualizer *der2Visualizer; //assuming sensor is being held by the hand -- this will have to be a vector for mo' body parts, etc.
    
    Derivative *der1;
    Derivative *der2;

    bool handInit = false; //whether we have set up the hand ugens
    int handID; //this will correspond to wii mote # or some OSC id'ing the sensor

public:
    enum BodyPart{ HAND=0, FOOT=1 }; //add body parts here. Doesn't have to be a body I suppose but you get what I mean..

    Entity(){
        handInit = false;
    };
    
    bool isInit()
    {
        return handInit;
    };
    

    void addSensorBodyPart(int idz, SensorData *sensor, BodyPart whichBody )
    {
        //which body part is the sensor of?
        std::vector<UGEN * > *bUgens; //the ugen vector to add ugens to
        switch (whichBody)
        {
            case HAND: //well it has to be currently, but ya know, in case this gets expanded...
                bUgens = &hand;
                handInit = true;
                handID = idz;
                break;
            default:
                break;
        };
        
        //add all the basic signal analysis that happens for ea. sensor
        InputSignal *signal_input = new InputSignal(idz); //1. create an input
        signal_input->setInput(sensor);
        bUgens->push_back( signal_input );
        
        //add an averaging filter
        AveragingFilter *avgfilter = new AveragingFilter(signal_input);
        bUgens->push_back( avgfilter );
        
        //add a first derivative
        der1 = new Derivative( avgfilter );
        bUgens->push_back( der1 );
        
        //add a second derivative
        der2 = new Derivative( der1 );
        bUgens->push_back( der2 );
        
        //add the visualizers
        handVisualizer = new MocapDataVisualizer( avgfilter );
        bUgens->push_back( handVisualizer );
        
        der1Visualizer = new MocapDataVisualizer( der1 );
        bUgens->push_back( der1Visualizer );
        
        der2Visualizer = new MocapDataVisualizer( der2 );
        bUgens->push_back( der2Visualizer );
        


    }
    
    virtual void draw()
    {
        color(0, 0, 1, 1.0f);
        handVisualizer->draw();
        color(1, 0, 0, 1.0f);
        der1Visualizer->draw();
        color(1, 0, 1, 1.0f);
        der2Visualizer->draw();
    }
    
    
    //if we wanted to gather & send OSC from our ugens or send our own osc.. do it here.
    virtual std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        for (int i = 0 ; i < hand.size(); i++)
        {
            for (int j = 0; j < hand[i]->getOSC().size(); j++) {
                msgs.push_back( hand[i]->getOSC()[i] );
            }
        }
        return msgs;
    };
    
    
    //update all the ugens we own. all of them that need updating..
    virtual void update(float seconds = 0)
    {
        for(int i=0; i<hand.size(); i++)
            hand[i]->update(seconds);
        
        der1->update(seconds);
        der2->update(seconds);
    };

};
    
};

#endif /* MeasuredEntities_h */
