// 
// 
// 

#include "CircularBuffer.h"
#include <string.h>

/**
 * \brief	Constructor that allocate memory for delay buffer
 * 
 * Memory is dynamically allocated
 * 
 * \param size	Size in word of buffer
 */
CircularBuffer::CircularBuffer(int32_t size)
{
	_size = size;
	buffer = NULL;
	buffer = new float [size] ;
	for (int i = 0;i < size;i++)
		buffer[i] = 0;
}

/**
 * \brief	Destructor of the class. Release buffer memory
 * 
 */
CircularBuffer::~CircularBuffer()
{
	if (buffer != NULL)
		delete buffer;
}

/**
 * \brief	Push new data in the buffer
 * 
 * \param data		New sample
 */
void CircularBuffer::PushData(float data)
{
	if (buffer == NULL) false;
	for (int i = 1; i < _size;i++)
	{
		buffer[_size - i] = buffer[_size - i - 1];
	}
	buffer[0] = data;
}

/**
 * \brief   Pop data from buffer
 * 
 * Data are not destroyed
 * 
 * \param delay     Delay in samples from head of the buffer
 * \return          data
 */
float CircularBuffer::GetData(int32_t delay)
{
	if (delay >= _size)
		return 0;
	else
		return buffer[delay];
}

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
