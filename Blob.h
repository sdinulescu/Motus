//
//  Blob.h
//  BlobTrackingLab
//
//  Created by courtney on 10/8/18.
//

#ifndef Blob_h
#define Blob_h

#include <sstream>

class Blob
{
protected:
    cv::KeyPoint keyPoint;
    int b_id;
public:
    Blob(cv::KeyPoint pt, int _id)
    {
        b_id = _id;
        update(pt);
    }
    
    void update(cv::KeyPoint pt)
    {
        keyPoint = pt;
    }
    
    void draw()
    {
        ci::gl::color(0.5,0.5, 0.65, 0.5);
        ci::gl::drawSolidCircle(ci::fromOcv(keyPoint.pt),keyPoint.size);
        ci::gl::color(1, 1, 1, 1);
        ci::gl::draw(drawText(), ci::fromOcv(keyPoint.pt));
    }
    
    ci::gl::Texture2dRef drawText()
    {
        //now we will draw the label
        std::stringstream sstr;
        sstr << b_id ;
        
        ci::TextLayout theID;
        theID.setColor( ci::Color( 1, 0, 0 ) ); //NOTE! put this in
        theID.addLine(sstr.str());
        return ci::gl::Texture2d::create(theID.render(true, false));
    }
};


#endif /* Blob_h */
