// CircularBuffer.h
#include <stdint.h>

#ifndef _CIRCULARBUFFER_h
#define _CIRCULARBUFFER_h

/**
 * \brief Delay buffer class
 * 
 * This class allows to create arbitrary delay
 * Memory is dynamically allocated by Constructor 
 */
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
