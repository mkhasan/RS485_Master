/*
 * config.h
 *
 *  Created on: May 7, 2018
 *      Author: usrc
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define SHIFT_0_A 0.0
#define SHIFT_1_A 0.0
#define SHIFT_2_A 0.0
#define SHIFT_3_A 0.0
#define SHIFT_4_A 0.0

#define SHIFT_5_A 0.0
#define SHIFT_6_A 0.0
#define SHIFT_7_A 0.0
#define SHIFT_8_A 0.0
#define SHIFT_9_A 0.0

#define SHIFT_10_A 0.0
#define SHIFT_11_A 0.0
#define SHIFT_12_A 0.0
#define SHIFT_13_A 0.0
#define SHIFT_14_A 0.0

#define SHIFT_15_A 0.0

#define SHIFT_0_B 0.0
#define SHIFT_1_B 0.0
#define SHIFT_2_B 0.0
#define SHIFT_3_B 0.0
#define SHIFT_4_B 0.0

#define SHIFT_5_B 0.0
#define SHIFT_6_B 0.0
#define SHIFT_7_B 0.0
#define SHIFT_8_B 0.0
#define SHIFT_9_B 0.0

#define SHIFT_10_B 0.0
#define SHIFT_11_B 0.0
#define SHIFT_12_B 0.0
#define SHIFT_13_B 0.0
#define SHIFT_14_B 0.0

#define SHIFT_15_B 3.64


/////////////////  do not touch below  ////////////////////

#define CONVERT_TO_DEF( _c, X ) case _c: return (SHIFT ## _ ## _c ## _ ## X )

/*
#define GET_DEF_VALUES(Y,T) T get_def_values ## _ ## Y( int rc)  \
{ \
	return 0.1; \
}

*/


#define GET_DEF_VALUES(Y,T) T get_def_values## _ ##Y( int rc)  \
{ \
	switch (rc) \
	{	\
		CONVERT_TO_DEF( 0, Y );	\
		CONVERT_TO_DEF( 1, Y );	\
		CONVERT_TO_DEF( 2, Y );	\
		CONVERT_TO_DEF( 3, Y );	\
		CONVERT_TO_DEF( 4, Y );	\
		\
		CONVERT_TO_DEF( 5, Y );	\
		CONVERT_TO_DEF( 6, Y );	\
		CONVERT_TO_DEF( 7, Y );	\
		CONVERT_TO_DEF( 8, Y );	\
		CONVERT_TO_DEF( 9, Y );	\
		\
		CONVERT_TO_DEF( 10, Y ); \
		CONVERT_TO_DEF( 11, Y ); \
		CONVERT_TO_DEF( 12, Y ); \
		CONVERT_TO_DEF( 13, Y ); \
		CONVERT_TO_DEF( 14, Y ); \
		\
		CONVERT_TO_DEF( 15, Y ); \
		default : return -1; \
	}	\
}



#endif /* SRC_CONFIG_H_ */
