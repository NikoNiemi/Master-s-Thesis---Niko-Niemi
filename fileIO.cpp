/*

****************This file is for hand dimension file I/O operations****************

Made by:	Niko Niemi as a part of his Master's Thesis
Date:		11.4.2017

- This C-code is used to build a static library
- The static library must be build and included (linked) in the ODE hand project
*/

#define _CRT_SECURE_NO_WARNINGS 1

#include "fileIO.h"

// If the handmodel is located in a different folder, please specify it here
#define handmodelFolder "../../Matlab Hand project/01 NIKO/Piecesolid/"

indexdim readIndex1Dim()
{
	indexdim Index1Dim;
	char lineBuffer[100],filenameBuffer[150], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.index1.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Index1Dim.length = dim[0];
	Index1Dim.width = dim[1];
	return(Index1Dim);
}

indexdim readIndex2Dim()
{
	indexdim Index2Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.index2.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}
	Index2Dim.length = dim[0];
	Index2Dim.width = dim[1];
	return(Index2Dim);
}

middledim readMiddle1Dim()
{
	middledim Middle1Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.middle1.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Middle1Dim.length = dim[0];
	Middle1Dim.width = dim[1];
	return(Middle1Dim);
}

middledim readMiddle2Dim()
{
	middledim Middle2Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.middle2.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Middle2Dim.length = dim[0];
	Middle2Dim.width = dim[1];
	return(Middle2Dim);
}

ringdim readRing1Dim()
{
	ringdim Ring1Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.ring1.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Ring1Dim.length = dim[0];
	Ring1Dim.width = dim[1];
	return(Ring1Dim);
}

ringdim readRing2Dim()
{
	ringdim Ring2Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.ring2.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Ring2Dim.length = dim[0];
	Ring2Dim.width = dim[1];
	return(Ring2Dim);
}

pinkiedim readPinkie1Dim()
{
	pinkiedim Pinkie1Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.pinkie1.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Pinkie1Dim.length = dim[0];
	Pinkie1Dim.width = dim[1];
	return(Pinkie1Dim);
}

pinkiedim readPinkie2Dim()
{
	pinkiedim Pinkie2Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.pinkie2.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Pinkie2Dim.length = dim[0];
	Pinkie2Dim.width = dim[1];
	return(Pinkie2Dim);
}

thumbdim readThumb1Dim()
{
	thumbdim Thumb1Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.thumb1.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Thumb1Dim.length = dim[0];
	Thumb1Dim.width = dim[1];
	return(Thumb1Dim);
}

thumbdim readThumb2Dim()
{
	thumbdim Thumb2Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.thumb2.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Thumb2Dim.length = dim[0];
	Thumb2Dim.width = dim[1];
	return(Thumb2Dim);
}

thumbdim readThumb3Dim()
{
	thumbdim Thumb3Dim;
	char lineBuffer[100], filenameBuffer[120], *string;
	int i = 0;
	float dim[2];
	const char delimitter[20] = "\=\n";

	FILE * fp;
	snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.thumb3.txt", handmodelFolder);
	fp = fopen(filenameBuffer, "r");
	if (fp != NULL) {
		while (fscanf(fp, "%s", lineBuffer) != NULL) {
			string = strtok(lineBuffer, delimitter);
			dim[i] = atof(strtok(NULL, delimitter));
			i++;
			if (i == 2) {
				break;
			}
		}
		fclose(fp);
	}
	else
	{
		perror("Couldn't open file\n");
	}

	Thumb3Dim.length = dim[0];
	Thumb3Dim.width = dim[1];
	return(Thumb3Dim);
}

palmdim readPalmDim() {
	{
		palmdim PalmDim;
		char lineBuffer[100], filenameBuffer[120], *string;
		int i = 0;
		float dim[4];
		const char delimitter[20] = "\=\n";

		FILE * fp;
		snprintf(filenameBuffer, sizeof(filenameBuffer), "%sequationsout.palm.txt", handmodelFolder);
		fp = fopen(filenameBuffer, "r");
		if (fp != NULL) {
			while (fscanf(fp, "%s", lineBuffer) != NULL) {
				string = strtok(lineBuffer, delimitter);
				dim[i] = atof(strtok(NULL, delimitter));
				i++;
				if (i == 4) {
					break;
				}
			}
			fclose(fp);
		}
		else
		{
			perror("Couldn't open file\n");
		}

		PalmDim.breadth = dim[0];
		PalmDim.length = dim[1];
		PalmDim.widthfinger = dim[2];
		PalmDim.palmdepth = dim[3];
		return(PalmDim);
	}
}

