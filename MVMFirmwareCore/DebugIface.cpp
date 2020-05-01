// 
// 
// 

#include "DebugIface.h"

/**
 * \brief   Init Debug class. Require a verbosity level and a pointer to HW
 * 
 * This HW pointer is used to access to the PrintLineDebugConsole function
 * 
 * Every message with a priority equal or higher of verbose_level is print
 * 
 * \param _vs       Verbosity level
 * \param _hwi      Pointer to hardware class
 */
void DebugIfaceClass::Init(verbose_level _vs, HW* _hwi)
{
	hwi = _hwi;
	vsl = _vs;
}

/**
 * \brief   Change verbosity level
 * 
 * \param _vs       Verbosity class
 */
void DebugIfaceClass::SetVerboseLevel(verbose_level _vs)
{
	vsl = _vs;
}

/**
 * \brief   Print a message on the debug console
 * 
 * \param source        DBG_CODE for code print, DBG_KERNEL for kernel print
 * \param vl            Verbosity level
 * \param s             String to be print
 */
void DebugIfaceClass::DbgPrint(dbg_source source, verbose_level vl, String s)
{
	String msg;
	if (vl <= vsl)
	{
		String src = (source == DBG_CODE ? "CODE" : "KERNEL");
		msg = "[" + src + "] - " + s;
		hwi->PrintLineDebugConsole(msg);
	}
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
