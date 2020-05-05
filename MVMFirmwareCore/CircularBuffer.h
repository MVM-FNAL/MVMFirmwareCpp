// CircularBuffer.h
#ifdef DOXYGEN
#define int32_t int
#define uint32_t int
#else
#include <stdint.h>
#endif

#ifndef _CIRCULARBUFFER_h
#define _CIRCULARBUFFER_h

class CircularBuffer {

private:
	float* buffer;
	int _size;
public:
	CircularBuffer(int32_t size);
	~CircularBuffer();
	void PushData(float data);
	float GetData(int32_t delay);
};


#endif


//                  #     # ### 
//                  ##    #  #  
//                  # #   #  #  
//                  #  #  #  #  
//                  #   # #  #  
//                  #    ##  #  
//                  #     # ### 
//
// Nuclear Instruments 2020 - All rights reserved
// Any commercial use of this code is forbidden
// Contact info@nuclearinstruments.eu
