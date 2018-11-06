//
//  MotionCaptureData.h
//  VideoAndOSCLab
//
//  Created by courtney on 10/22/18.
//

#ifndef MotionCaptureData_h
#define MotionCaptureData_h

#define WIIMOTE_ACCELMAX 1.0
#define WIIMOTE_ACCELMIN 0.0
#define DEVICE_ARG_COUNT_MAX 24

namespace CRCPMotionAnalysis {


class MocapDeviceData
{
    
    private:
        double data[DEVICE_ARG_COUNT_MAX+1];
        float quaternion[4]; //rotations
        float orientationMatrix[3];
    
        //the default is wiimote
        double getAccelMax(){ return WIIMOTE_ACCELMAX; };
        double getAccelMin(){ return WIIMOTE_ACCELMIN; };
    public:
        enum DataIndices { INDEX=0, TIME_STAMP=1, ACCELX=2, ACCELY=3, ACCELZ=4, GYROX=11, GYROY=12, GYROZ=13, POSX= 14, POSY=15, QX=20, QY=21, QZ=22, QA=23 };
    
        //scale accel to 0 - 1 -- obv. not needed if wiimote
        void scaleAccel()
        {
            for (int i = ACCELX; i <= ACCELZ; i++)
            {
                data[i] =  std::abs(data[i] )  / ( getAccelMax() );
            }
        };
    
        //return sensor data as a string
        std::string str()
        {
            std::stringstream ss;
            
            for (int i = 0; i < DEVICE_ARG_COUNT_MAX; i++)
            {
                ss << data[i];
                if (i < DEVICE_ARG_COUNT_MAX - 1) ss << ",";
            }
            ss << std::endl;
            
            return ss.str();
        };
        
        inline double getTimeStamp(){ return data[1]; };
        inline ci::vec3 getAccelData()
        {
            ci::vec3 accelData(data[ACCELX], data[ACCELY], data[ACCELZ]);
            return accelData;
        };
        inline void setAccelData(ci::vec3 d)
        {
            data[ACCELX] = d.x;
            data[ACCELY] = d.y;
            data[ACCELZ] = d.z;
        };
        inline ci::vec3 getGyro()
        {
            ci::vec3 gyro(data[GYROX], data[GYROY], data[GYROZ]);
            return gyro;
        };
        inline void setGyro(ci::vec3 d)
        {
            data[GYROX] = d.x;
            data[GYROY] = d.y;
            data[GYROZ] = d.z;
        };

        inline void setData(int index, double d)
        {
            data[index] = d;
        }
        inline double getData(int index)
        {
            if(index < 20)
            {
                return data[index];
            }
            else if (index < 24)
            {
                int i = index - 20;
                return getQuarternion(i);
            }
            else
            {
                std::cout << "Warning! Motion Sensor Data: Out of Range! Index: " << index << "\n";
                return NO_DATA;
            }
            
        }
        inline float getQuarternion(int index)
        {
            assert( index < 4 && index >= 0 );
            return quaternion[index];
        };
        
        inline ci::vec4 getQuarternionVec4d()
        {
            return ci::vec4(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        };
        
        inline void setQuarternion(ci::vec4 quat)
        {
            quaternion[0]=quat[0];
            quaternion[1]=quat[1];
            quaternion[2]=quat[2];
            quaternion[3]=quat[3];
        };
        
        inline void setQuarternion(float x, float y, float z, float angle)
        {
            quaternion[0] = x;
            quaternion[1] = y;
            quaternion[2] = z;
            quaternion[3] = angle;
            
        };
        
        MocapDeviceData()
        {
            //init memory
            for(int i=0; i<DEVICE_ARG_COUNT_MAX; i++)
            {
                data[i] = NO_DATA;
            }
            
            for(int i=0; i<4; i++)
                quaternion[i] = NO_DATA;
            
            for(int i=0; i<3; i++ )
                orientationMatrix[i] = NO_DATA;
        }
        
    
};
    
};
    
#endif /* MotionCaptureData_h */

