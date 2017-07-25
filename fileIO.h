/* 

****************This file is for hand dimension file I/O operations**************** 

Made by:	Niko Niemi as a part of his Master's thesis
Date:		11.4.2017

- Header file
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Indexdimensions{
	float length;
	float width;
}indexdim;

typedef struct Middledimensions {
	float length;
	float width;
}middledim;

typedef struct Ringdimensions {
	float length;
	float width;
}ringdim;

typedef struct Pinkiedimensions {
	float length;
	float width;
}pinkiedim;

typedef struct Thumbdimensions {
	float length;
	float width;
}thumbdim;

typedef struct Palm {
	float length;
	float breadth;
	float widthfinger;
	float palmdepth;
}palmdim;

indexdim readIndex1Dim();
indexdim readIndex2Dim();

middledim readMiddle1Dim();
middledim readMiddle2Dim();

ringdim readRing1Dim();
ringdim readRing2Dim();

pinkiedim readPinkie1Dim();
pinkiedim readPinkie2Dim();

thumbdim readThumb1Dim();
thumbdim readThumb2Dim();
thumbdim readThumb3Dim();

palmdim readPalmDim();
