//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

//#define MY_PI 3.1415926
//#define TWO_PI (2.0* MY_PI)

#define PERFORMANCE_TEST

//#define __NOAA__
//#define __SSAA__
#define __MSAA__


#ifdef __SSAA__
#define SS_SCALE 2
#endif

#ifdef __MSAA__
#define MS_SCALE 2
#endif

#endif //RASTERIZER_GLOBAL_H
