//
//  Astra.h
//  BlobTrackingOrrbec
//
//  Created by courtney on 11/16/18.
//  This code is for use with Cinder, libcinder.org
//  Gets a depth image from the astra then returns as ci::Surface
//  Uses code from the orbbec astra samples, mainly the Simple Depth Viewer, then adds the conversion to the cinder types
//

#ifndef AstraClass_h
#define AstraClass_h

// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.

#include "astra/astra.hpp"
#include "LitDepthVisualizer.hpp"

//this was modified from from Astra API  samples by Courtney Brown
class SampleFrameListener : public astra::FrameListener
{
private:
    using buffer_ptr = std::unique_ptr<int16_t []>;
    buffer_ptr buffer_;
    unsigned int lastWidth_;
    unsigned int lastHeight_;
    
    //making it work with cinder - CDB
    ci::SurfaceRef mSurface;
    bool new_frame_ready;
    
    int depthWidth_{0};
    int depthHeight_{0};
    using DepthPtr = std::unique_ptr<int16_t[]>;
    DepthPtr depthData_{nullptr};
//    const astra::CoordinateMapper& coordinateMapper_; //not used
    
    int displayWidth_{0};
    int displayHeight_{0};
    
    using BufferPtr = std::unique_ptr<uint8_t[]>;
    BufferPtr displayBuffer_{nullptr};
    
    LitDepthVisualizer visualizer_;
    
public:
    //CDB -  init some values
        SampleFrameListener() : astra::FrameListener()

    {
        mSurface = NULL;
        new_frame_ready = false;
    };
    
    //gets a frame from the astra then converts into ci::Surface
    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
//        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();  //not used
        
        const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

            const int width = pointFrame.width();
            const int height = pointFrame.height();
//
            visualizer_.update(pointFrame);
            
            if(mSurface == NULL)
                mSurface = ci::Surface::create(width, height, true);
//
            ci::Surface::Iter iter = mSurface->getIter( ci::Area(0,0,width,height) );
            const astra::RgbPixel* vizBuffer = visualizer_.get_output();
        
        // this converts the data from the visualizer into a ci::Surface
//          //TODO: optimize, if needed
            int i = 0;
            while( iter.line() ){
                while( iter.pixel() )
                {
                    iter.r() = vizBuffer[i].r;
                    iter.b() = vizBuffer[i].b;
                    iter.g() = vizBuffer[i].g;
                    iter.a() = 255;
                    i++;
                }
                
            }
        
            new_frame_ready = true;
    }
    

    //this prints out info in ea. depth frame
    void print_depth(const astra::DepthFrame& depthFrame,
                     const astra::CoordinateMapper& mapper)
    {
        if (depthFrame.is_valid())
        {
            int width = depthFrame.width();
            int height = depthFrame.height();
            int frameIndex = depthFrame.frame_index();
            
            //determine if buffer needs to be reallocated
            if (width != lastWidth_ || height != lastHeight_)
            {
                buffer_ = buffer_ptr(new int16_t[depthFrame.length()]);
                lastWidth_ = width;
                lastHeight_ = height;
            }
            
            size_t index = ((width * (height / 2.0f)) + (width / 2.0f));
            short middle = buffer_[index];

            float worldX, worldY, worldZ;
            float depthX, depthY, depthZ;
            mapper.convert_depth_to_world(width / 2.0f, height / 2.0f, middle, &worldX, &worldY, &worldZ);
            mapper.convert_world_to_depth(worldX, worldY, worldZ, &depthX, &depthY, &depthZ);

            std::cout << "depth frameIndex: " << frameIndex
            << " value: " << middle
            << " wX: " << worldX
            << " wY: " << worldY
            << " wZ: " << worldZ
            << " dX: " << depthX
            << " dY: " << depthY
            << " dZ: " << depthZ
            << std::endl;
        }
    }
    
    //currently unused
    void copy_depth_data(const astra::DepthFrame depthFrame)
    {
        
        if (depthFrame.is_valid())
        {
            const int width = depthFrame.width();
            const int height = depthFrame.height();
            if (!depthData_ || width != depthWidth_ || height != depthHeight_)
            {
                depthWidth_ = width;
                depthHeight_ = height;
                
                // texture is RGBA
                const int byteLength = depthWidth_ * depthHeight_ * sizeof(uint16_t);
                
                depthData_ = DepthPtr(new int16_t[byteLength]);
            }
            
            depthFrame.copy_to(&depthData_[0]);
        }
    }

    
    //get that new frame. Note you can only get it once.
    ci::SurfaceRef getNewFrame()
    {
        new_frame_ready = false; //now we have the most recent frame - cdb
        return mSurface;
    }
    
    //did we get a new frame from the astra?
    bool newFrame()
    {
        return new_frame_ready;
    }
    
    //can be added back into on_frame_ready to check/see the frame rate.
    void check_fps()
    {
        const double frameWeight = 0.2;
        
        auto newTimepoint = clock_type::now();
        auto frameDuration = std::chrono::duration_cast<duration_type>(newTimepoint - lastTimepoint_);
        
        frameDuration_ = frameDuration * frameWeight + frameDuration_ * (1 - frameWeight);
        lastTimepoint_ = newTimepoint;
        
        double fps = 1.0 / frameDuration_.count();
        
        auto precision = std::cout.precision();
        std::cout << std::fixed
        << std::setprecision(1)
        << fps << " fps ("
        << std::setprecision(2)
        << frameDuration.count() * 1000 << " ms)"
        << std::setprecision(precision)
        << std::endl;
    }
    
private:
    using duration_type = std::chrono::duration < double > ;
    duration_type frameDuration_{ 0.0 };
    
    using clock_type = std::chrono::system_clock;
    std::chrono::time_point<clock_type> lastTimepoint_;
    
};
#endif /* AstraClass_h */
