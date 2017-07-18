/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
* All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
*                                                                       *
* This library is free software; you can redistribute it and/or         *
* modify it under the terms of EITHER:                                  *
*   (1) The GNU Lesser General Public License as published by the Free  *
*       Software Foundation; either version 2.1 of the License, or (at  *
*       your option) any later version. The text of the GNU Lesser      *
*       General Public License is included with this library in the     *
*       file LICENSE.TXT.                                               *
*   (2) The BSD-style license that is included with this library in     *
*       the file LICENSE-BSD.TXT.                                       *
*                                                                       *
* This library is distributed in the hope that it will be useful,       *
* but WITHOUT ANY WARRANTY; without even the implied warranty of        *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
* LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
*                                                                       *
*************************************************************************/

// Last updated: 8.6.2017
// Done by Niko Niemi as a part of his Master's Thesis

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "fileIO.h"

// If double precision math is used (double precision is recommended)
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

/*============ THREE PARAMETERS HAVE TO BE DEFINED ============

Draw defines whether the simulation is drawn or not (uses drawstuff lib). Defined to 0 or 1.
	- if Draw is set to 1, the simulation times are not recorded
Consider_contact has to be defined to either 0 or 1 depending on whether contact is taken into account or not. 
HandControl defines whether PID control scheme is utilized (1) or not (0)

===========================================================*/

#define Draw 1
#if !Draw
	#define Time_results 1
#endif 
#define Consider_contact 0
#define HandControl 0 // if HandControl is used, simstep has to be 1ms or less (=<0.001) or otherwise PID values have to be tuned

#define SIMSTEP 0.001

#pragma region Densities&gravity
#define GRAVITY_X 0
#define GRAVITY_Y 0
#define GRAVITY_Z 0

#define DENSITY 1000
#pragma endregion

// Determine identifications for objects in the simulation environment
#pragma region WorldObjects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

static dBodyID Index[3], Middle[3], Pinkie[3], Ring[3], Thumb[3], Palm;
static dGeomID gIndex[3], gMiddle[3], gPinkie[3], gRing[3], gThumb[3], GPalm;
static dMass mIndex, mMiddle, mPinkie, mRing, mThumb, mPalm;
static dJointID joint_index[3], joint_middle[3], joint_pinkie[3], joint_ring[3], joint_thumb[3], joint_palm[3];

static dBodyID Index2[3], Middle2[3], Pinkie2[3], Ring2[3], Thumb2[3], Palm2;
static dGeomID gIndex2[3], gMiddle2[3], gPinkie2[3], gRing2[3], gThumb2[3], GPalm2;
static dMass mIndex2, mMiddle2, mPinkie2, mRing2, mThumb2, mPalm2;
static dJointID joint_index2[3], joint_middle2[3], joint_pinkie2[3], joint_ring2[3], joint_thumb2[3], joint_palm2[3];
#pragma endregion

//PID values and arrays for previous error of respective joint, as well as integral and derivative errors
#pragma region PID
const double PID[] = { 1,0.1,0.01 };
double prevError[] =		{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
double integralError[] =	{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
double derivativeError[] =	{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
#pragma endregion

//Position vector values, rotation matrix and scale factor
#pragma region Pos&Rot
dMatrix3 RotationMatrix;
dVector3 RotationAxis = { 1,0,0 };
dReal RotationAngle = -M_PI_2;

#define POSITION_X 0		// for right hand initial X position
#define POSITION_Y 0		
#define POSITION_Z 0.3		
#define POSITION_X2 -0.3	// for left hand initial X position
#define POSITION_Y2 0
#define POSITION_Z2 0.3
#define SCALE_FACTOR 1000	// Scale factor, finger dimensions are scaled 1/1000 of the sizes in the .txt files due to them being in millimeters
#pragma endregion

// this is called by dSpaceCollide when two objects in space are potentially colliding.
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	dContact contact;
	if (b1 && b2 && dAreConnected(b1, b2)) return;

	//contact.surface.mode = dContactApprox0;
	contact.surface.mode = dContactApprox1;

	// friction parameter
	contact.surface.mu = 0.2;
	contact.surface.mu2 = 0.2;

	// bounce is the amount of "bouncyness".
	contact.surface.bounce = .09;

	// bounce_vel is the minimum incoming velocity to cause a bounce
	contact.surface.bounce_vel = .01;

	// constraint force mixing parameter
	contact.surface.soft_cfm = 0.1;
	if (int numc = dCollide(o1, o2, 1, &contact.geom, sizeof(dContact))) {
		dJointID c = dJointCreateContact(world, contactgroup, &contact);
		dJointAttach(c, b1, b2);
	}
}

// start simulation - set viewpoint, world ERP and damping
static void start()
{
	static float xyz[3] = { -0.5,0,0.6 };
	static float hpr[3] = { 0,-15,0 };
	dsSetViewpoint(xyz, hpr);
	dWorldSetERP(world,0.5);
	dWorldSetDamping(world, 1e-2, 1e-3);
}

// This function applies torques to joints and controls them using PID controllers. Kp, Ki, Kd account for PID coefficients (fine tune @ ln 79), setValue is the goal angle of the joint.
// Each joint has an individual counter so that the previous errors as well as integrals and derivatives can be stored to the array described earlier (ln 77). 
// If more bodies need to be added to the system it is worthwhile to consider changing from manual arrays to memory allocated structures.
void PIDFunction(float Kp, float Ki, float Kd, dJointID jointID,double setValue,int counter) {
	dJointID joint = jointID;
	const dReal rotValuedt = dJointGetHingeAngleRate(joint);
	const dReal rotValue = dJointGetHingeAngle(joint);
	double error = setValue - fabs(rotValue);
	integralError[counter] += error;
	derivativeError[counter] = error - prevError[counter];
	dReal torque = Kp * error + Ki * integralError[counter] * SIMSTEP + Kd * derivativeError[counter] / SIMSTEP;
	dJointAddHingeTorque(joint, torque);
	prevError[counter] = error;
}

// Transform coordinate systems to account for initial rotation matrix
float coordinateTransform(float ux, float uy, float uz,int xyz) {
	float coordinate;
	if (xyz == 0) {
		coordinate = (RotationMatrix[0] * ux + RotationMatrix[1] * uy + RotationMatrix[2] * uz);
		return coordinate;
	}
	if (xyz == 1) {
		coordinate = (RotationMatrix[4] * ux + RotationMatrix[5] * uy + RotationMatrix[6] * uz);
		return coordinate;
	}
	if (xyz == 2) {
		coordinate = (RotationMatrix[8] * ux + RotationMatrix[9] * uy + RotationMatrix[10] * uz);
		return coordinate;
	}
}

// Simulation loop
static void simLoop(int pause)
{
	static double time = 0;
	const dReal *pos;
	const dReal *R;

	time += SIMSTEP;
	if (!pause) {
#if Consider_contact // Consider_contact is defined in the beginning of this file
		// find collisions and add contact joints
		dSpaceCollide(space, 0, &nearCallback);
#endif // 1

		// step the simulation
		dWorldStep(world, SIMSTEP);
		
		// remove all contact joints
		dJointGroupEmpty(contactgroup);
		dVector3 sides;
		dVector3 capsuleRad;
		dVector3 capsuleLen;
#if Draw
		// drawmodes = 1 or 0
		dsSetDrawMode(0);

		// Draw visualization geometries
		pos = dGeomGetPosition(GPalm);
		R = dGeomGetRotation(GPalm);
		dGeomBoxGetLengths(GPalm, sides);
		dsDrawBox(pos, R, sides);

		pos = dGeomGetPosition(GPalm2);
		R = dGeomGetRotation(GPalm2);
		dGeomBoxGetLengths(GPalm2, sides);
		dsDrawBox(pos, R, sides);

		for (int i = 0; i < 3; i++) {
			
			pos = dGeomGetPosition(gIndex[i]);
			R = dGeomGetRotation(gIndex[i]);
			dGeomCapsuleGetParams(gIndex[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gIndex2[i]);
			R = dGeomGetRotation(gIndex2[i]);
			dGeomCapsuleGetParams(gIndex2[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);
			
			pos = dGeomGetPosition(gMiddle[i]);
			R = dGeomGetRotation(gMiddle[i]);
			dGeomCapsuleGetParams(gMiddle[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gMiddle2[i]);
			R = dGeomGetRotation(gMiddle2[i]);
			dGeomCapsuleGetParams(gMiddle2[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);
			
			pos = dGeomGetPosition(gRing[i]);
			R = dGeomGetRotation(gRing[i]);
			dGeomCapsuleGetParams(gRing[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gRing2[i]);
			R = dGeomGetRotation(gRing2[i]);
			dGeomCapsuleGetParams(gRing2[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gPinkie[i]);
			R = dGeomGetRotation(gPinkie[i]);
			dGeomCapsuleGetParams(gPinkie[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gPinkie2[i]);
			R = dGeomGetRotation(gPinkie2[i]);
			dGeomCapsuleGetParams(gPinkie2[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gThumb[i]);
			R = dGeomGetRotation(gThumb[i]);
			dGeomCapsuleGetParams(gThumb[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);

			pos = dGeomGetPosition(gThumb2[i]);
			R = dGeomGetRotation(gThumb2[i]);
			dGeomCapsuleGetParams(gThumb2[i], capsuleRad, capsuleLen);
			dsDrawCapsule(pos, R, *capsuleLen, *capsuleRad);
		}
	
#endif
#if HandControl
		// This is where handcontrol is called, some function calls scale the PID values for tuning purposes
		// Starts control at time of 0.01s (or 0.02 for fingers)
		for (int i = 0; i < 6; i++) {
			//Palm
			if (i == 0 && time > 0.01) {
				PIDFunction(PID[0], PID[1], PID[2], joint_palm[0], M_PI_2, i);
				PIDFunction(PID[0], PID[1], PID[2], joint_palm2[0], M_PI_2, i+1);
			}
			//Index
			if (i == 1 && time > 0.02) {
				for (int k = 2; k < 5; k++) {
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_index[k - 2], 0, k);
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_index2[k - 2], 0, k+3);
				}
			}
			//Middle
			if (i == 2 && time > 0.02) {
				for (int k = 8; k < 11; k++) {
					PIDFunction(PID[0] / 4, PID[1] / 8, PID[2] / 32, joint_middle[k - 8], 1.3, k);
					PIDFunction(PID[0] / 4, PID[1] / 8, PID[2] / 32, joint_middle2[k - 8], 1.3, k+3);
				}
			}
			//Ring
			if (i == 3 && time > 0.02) {
				for (int k = 14; k < 17; k++) {
					PIDFunction(PID[0] / 4, PID[1] / 8, PID[2] / 32, joint_ring[k - 14], 1.3, k);
					PIDFunction(PID[0] / 4, PID[1] / 8, PID[2] / 32, joint_ring2[k - 14], 1.3, k+3);
				}
			}
			//Pinkie
			if (i == 4 && time > 0.02) {
				for (int k = 20; k < 23; k++) {
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_pinkie[k - 20], 0, k);
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_pinkie2[k - 20], 0, k+3);
				}
			}
			//Thumb
			// Requires a special PID since the PID function introduced earlier only applies hinge torques and thumb requires universal torques, idea is the same as before.
			if (i == 5 && time > 0.02) {
#pragma region Thumb1 (right) first joint control
				double setvalue1 = 0.2;
				double setvalue2 = 0.2;
				const dReal rotValue1 = dJointGetUniversalAngle1(joint_thumb[0]);
				const dReal rotValue2 = dJointGetUniversalAngle2(joint_thumb[0]);
				double error1 = setvalue1 - fabs(rotValue1);
				double error2 = setvalue2 - rotValue2;
				integralError[26] += error1;
				integralError[27] += error2;
				derivativeError[26] = error1 - prevError[26];
				derivativeError[27] = error2 - prevError[27];
				dReal torque1 = (PID[0] / 8) * error1 + (PID[1] / 8) * integralError[26] * SIMSTEP + (PID[2] / 64) * derivativeError[26] / SIMSTEP;
				dReal torque2 = (PID[0] / 8) * error2 + (PID[1] / 8) * integralError[27] * SIMSTEP + (PID[2] / 64) * derivativeError[27] / SIMSTEP;
				dJointAddUniversalTorques(joint_thumb[0], torque1, torque2);
				prevError[26] = error1;
				prevError[27] = error2;
#pragma endregion

#pragma region Thumb2 (left) first joint control
				double setvalue1_2 = -0.2;
				double setvalue2_2 = -0.2;
				const dReal rotValue1_2 = dJointGetUniversalAngle1(joint_thumb2[0]);
				const dReal rotValue2_2 = dJointGetUniversalAngle2(joint_thumb2[0]);
				double error1_2 = setvalue1_2 - fabs(rotValue1);
				double error2_2 = setvalue2_2 - rotValue2;
				integralError[28] += error1_2;
				integralError[29] += error2_2;
				derivativeError[28] = error1_2 - prevError[28];
				derivativeError[29] = error2_2 - prevError[29];
				dReal torque1_2 = (PID[0] / 8) * error1_2 + (PID[1] / 8) * integralError[28] * SIMSTEP + (PID[2] / 64) * derivativeError[28] / SIMSTEP;
				dReal torque2_2 = (PID[0] / 8) * error2_2 + (PID[1] / 8) * integralError[29] * SIMSTEP + (PID[2] / 64) * derivativeError[29] / SIMSTEP;
				dJointAddUniversalTorques(joint_thumb2[0], torque1_2, torque2_2);
				prevError[28] = error1_2;
				prevError[29] = error2_2;
#pragma endregion

				for (int k = 30; k < 32; k++) {
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_thumb[k - 29], 0, k);
					PIDFunction(PID[0] / 8, PID[1] / 8, PID[2] / 32, joint_thumb2[k - 29], M_PI_4, k+2);
				}
			}
		}
#endif
	}
}

// Main function, also contains body&geom configs
int main(int argc, char **argv)
{

#pragma region MainStart

	// form rotation matrix based on global variables
	dRFromAxisAndAngle(RotationMatrix, RotationAxis[0], RotationAxis[1], RotationAxis[2], RotationAngle);
	int i;

	// setup pointers to drawstuff callback functions
#if Draw
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.command = 0;
	fn.path_to_textures = "../../drawstuff/textures";
#endif

	dInitODE();
	char *filename;

	// Results are timed if Time_results is defined i.e. Draw is defined as 0
#ifdef Time_results
	if (Consider_contact) {
		filename = "data_contact.txt";
	}
	else
	{
		filename = "data.txt";
	}
	FILE * data = fopen(filename, "w");
#ifdef dDOUBLE
	fprintf(data, "-----------Double precision math-----------\n");
#else
	fprintf(data, "-----------Single precision math-----------\n");
#endif

#endif

	// Create world
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	dWorldSetGravity(world, GRAVITY_X, GRAVITY_Y, GRAVITY_Z);
	dWorldSetCFM(world, 0);
	dCreatePlane(space, 0, 0, 1, 0);
	contactgroup = dJointGroupCreate(0);

#pragma endregion

#pragma region Handright

#pragma region PalmCreation

	// Create and set the palm first
	palmdim Palmdimensions;
	Palmdimensions = readPalmDim();
	const float palm_length_scaled = Palmdimensions.length / SCALE_FACTOR;
	const float palm_breadth_scaled = Palmdimensions.breadth / SCALE_FACTOR;
	const float palm_thickness_scaled = Palmdimensions.palmdepth / SCALE_FACTOR;

	Palm = dBodyCreate(world);
	dMassSetBox(&mPalm, DENSITY, palm_length_scaled, palm_breadth_scaled, palm_thickness_scaled);
	GPalm = dCreateBox(space, palm_breadth_scaled, palm_thickness_scaled, palm_length_scaled);
	dBodySetMass(Palm, &mPalm);
	dGeomSetBody(GPalm, Palm);
	dGeomSetPosition(GPalm, POSITION_X, POSITION_Y, POSITION_Z);
	dGeomSetRotation(GPalm, RotationMatrix);
#pragma endregion

#pragma region FingerBod

	// Create finger bodies
	for (int i = 0; i < 3; i++) {
		Index[i] = dBodyCreate(world);
		Middle[i] = dBodyCreate(world);
		Ring[i] = dBodyCreate(world);
		Pinkie[i] = dBodyCreate(world);
		Thumb[i] = dBodyCreate(world);
	}
#pragma endregion

#pragma region CapsuleDimensions

	//Dimensions of capsules, here is where fileio.h & fileio.c is used and it has to be set up properly
	indexdim dim_i;
	indexdim dim_i2;
	dim_i = readIndex1Dim();
	dim_i2 = readIndex2Dim();
	const float index_length1_scaled = dim_i.length / SCALE_FACTOR;			// this is the first (right) index length of the first hand
	const float index_width1_scaled = dim_i.width / (2 * SCALE_FACTOR);		// this is the first (right) index radius of the first hand
	const float index_length2_scaled = dim_i2.length / SCALE_FACTOR;		// this is the second (right) index length of the first hand
	const float index_width2_scaled = dim_i2.width / (2 * SCALE_FACTOR);	// this is the second (right) index radius of the first hand

	middledim dim_m;
	middledim dim_m2;
	dim_m = readMiddle1Dim();
	dim_m2 = readMiddle2Dim();
	const float middle_length1_scaled = dim_m.length / SCALE_FACTOR;
	const float middle_width1_scaled = dim_m.width / (2 * SCALE_FACTOR);
	const float middle_length2_scaled = dim_m2.length / SCALE_FACTOR;
	const float middle_width2_scaled = dim_m2.width / (2 * SCALE_FACTOR);

	ringdim dim_r;
	ringdim dim_r2;
	dim_r = readRing1Dim();
	dim_r2 = readRing2Dim();
	const float ring_length1_scaled = dim_r.length / SCALE_FACTOR;
	const float ring_width1_scaled = dim_r.width / (2 * SCALE_FACTOR);
	const float ring_length2_scaled = dim_r2.length / SCALE_FACTOR;
	const float ring_width2_scaled = dim_r2.width / (2 * SCALE_FACTOR);

	pinkiedim dim_p;
	pinkiedim dim_p2;
	dim_p = readPinkie1Dim();
	dim_p2 = readPinkie2Dim();
	const float pinkie_length1_scaled = dim_p.length / SCALE_FACTOR;
	const float pinkie_width1_scaled = dim_p.width / (2 * SCALE_FACTOR);
	const float pinkie_length2_scaled = dim_p2.length / SCALE_FACTOR;
	const float pinkie_width2_scaled = dim_p2.width / (2 * SCALE_FACTOR);

	thumbdim dim_t;
	thumbdim dim_t2;
	thumbdim dim_t3;
	dim_t = readThumb1Dim();
	dim_t2 = readThumb2Dim();
	dim_t3 = readThumb3Dim();
	const float thumb_length1_scaled = dim_t.length / SCALE_FACTOR;
	const float thumb_width1_scaled = dim_t.width / (2 * SCALE_FACTOR);
	const float thumb_length2_scaled = dim_t2.length / SCALE_FACTOR;
	const float thumb_width2_scaled = dim_t2.width / (2 * SCALE_FACTOR);
	const float thumb_length3_scaled = dim_t3.length / SCALE_FACTOR;
	const float thumb_width3_scaled = dim_t3.width / (2 * SCALE_FACTOR);

#pragma endregion

#pragma region Masses, geoms

	//Create masses
	dMassSetCapsule(&mIndex, DENSITY, 1, index_width1_scaled, index_length1_scaled);
	dMassSetCapsule(&mMiddle, DENSITY, 1, middle_width1_scaled, middle_length1_scaled);
	dMassSetCapsule(&mRing, DENSITY, 1, ring_width1_scaled, ring_length1_scaled);
	dMassSetCapsule(&mPinkie, DENSITY, 1, pinkie_width1_scaled, pinkie_length1_scaled);
	dMassSetCapsule(&mThumb, DENSITY, 1, thumb_width1_scaled, thumb_length1_scaled);

	// Create geometries to be used in simulation
	for (i = 0; i < 3; i++) {
		if (i == 0) {
			gIndex[i] = dCreateCapsule(space, index_width1_scaled, index_length1_scaled);
			gMiddle[i] = dCreateCapsule(space, middle_width1_scaled, middle_length1_scaled);
			gRing[i] = dCreateCapsule(space, ring_width1_scaled, ring_length1_scaled);
			gPinkie[i] = dCreateCapsule(space, pinkie_width1_scaled, pinkie_length1_scaled);
			gThumb[i] = dCreateCapsule(space, thumb_width1_scaled, thumb_length1_scaled);
		}

		if (i == 1 || i == 2) {
			gIndex[i] = dCreateCapsule(space, index_width2_scaled, index_length2_scaled);
			gMiddle[i] = dCreateCapsule(space, middle_width2_scaled, middle_length2_scaled);
			gRing[i] = dCreateCapsule(space, ring_width2_scaled, ring_length2_scaled);
			gPinkie[i] = dCreateCapsule(space, pinkie_width2_scaled, pinkie_length2_scaled);
			if (i == 2) {
				gThumb[i] = dCreateCapsule(space, thumb_width3_scaled, thumb_length3_scaled);
				break;
			}
			gThumb[i] = dCreateCapsule(space, thumb_width2_scaled, thumb_length2_scaled);
		}
	}

	// Set afore defined mass properties to bodies
	for (i = 0; i < 3; i++) {
		dBodySetMass(Index[i], &mIndex);
		dBodySetMass(Middle[i], &mMiddle);
		dBodySetMass(Ring[i], &mRing);
		dBodySetMass(Pinkie[i], &mPinkie);
		if (i == 2) {
			dBodySetMass(Thumb[i], &mThumb);
			break;
		}
		dBodySetMass(Thumb[i], &mThumb);
	}

	// Set bodies' geometries
	for (i = 0; i < 3; i++) {
		dGeomSetBody(gIndex[i], Index[i]);
		dGeomSetBody(gMiddle[i], Middle[i]);
		dGeomSetBody(gRing[i], Ring[i]);
		dGeomSetBody(gPinkie[i], Pinkie[i]);
		if (i == 2) {
			dGeomSetBody(gThumb[i], Thumb[i]);
			break;
		}
		dGeomSetBody(gThumb[i], Thumb[i]);
	}
#pragma endregion

#pragma region SetPosAndRot

	//  rp^i = R^i + A^i * u^i    
	//	R^i is the initial position vector (Defined parameters) A^i is the rotation matrix and u^i is the distance from palm center to finger center


	// Math operations to clarify placements
	const float fingerGap = (palm_breadth_scaled - pinkie_width1_scaled * 2 - index_width1_scaled * 2 - middle_width1_scaled * 2 - ring_width1_scaled * 2) / 3;

	// For each finger, 3 arrays are created, each consisting of 6 elements. 0, 2 and 4 are joint locations and 1, 3 and 5 are finger body locations
	// rp[coordinate]_[finger]_Array therefore includes the global positions of those aforementioned elements
	// u[coordinate_[finger] includes the position of the element in body coordinates. These are not stored as they are only used to calculate rp[coordinate]_[finger]_Array

	float ux_index;
	float uy_index;
	float uz_index;
	float rpx_index_Array[6];
	float rpy_index_Array[6];
	float rpz_index_Array[6];

	float ux_middle;
	float uy_middle;
	float uz_middle;
	float rpx_middle_Array[6];
	float rpy_middle_Array[6];
	float rpz_middle_Array[6];

	float ux_ring;
	float uy_ring;
	float uz_ring;
	float rpx_ring_Array[6];
	float rpy_ring_Array[6];
	float rpz_ring_Array[6];

	float ux_pinkie;
	float uy_pinkie;
	float uz_pinkie;
	float rpx_pinkie_Array[6];
	float rpy_pinkie_Array[6];
	float rpz_pinkie_Array[6];

	float ux_thumb;
	float uy_thumb;
	float uz_thumb;
	float rpx_thumb_Array[6];
	float rpy_thumb_Array[6];
	float rpz_thumb_Array[6];

	// This is for index
	for (int i = 0; i < 6; i++) {
		if (i < 3) {		// First three are for first joint location, first body location and second joint location
			ux_index = -(palm_breadth_scaled / 2) + (index_width1_scaled);
			uy_index = 0;
			uz_index = (palm_length_scaled / 2) + i*(index_length1_scaled / 2) + (fabs(i*(i - 1)) / 2)*index_width1_scaled / 2; //This distance has been reached through experimental research, 
																																//as dimensions cannot be positioned just by i*index_len/2 because first and last
																																//finger capsule would overlap creating issues with ODE
																																// SAME GOES FOR OTHER FINGES AND JOINT LOCATIONS!!
			rpx_index_Array[i] = POSITION_X + coordinateTransform(ux_index, uy_index, uz_index, 0);
			rpy_index_Array[i] = POSITION_Y + coordinateTransform(ux_index, uy_index, uz_index, 1);
			rpz_index_Array[i] = POSITION_Z + coordinateTransform(ux_index, uy_index, uz_index, 2);
		}
		else if (i < 5) {	// four and five are for second body location and third joint location
			ux_index = -(palm_breadth_scaled / 2) + (index_width1_scaled);
			uy_index = 0;
			uz_index += fabs(i - 4)*index_width2_scaled / 2 + (i - 2)*index_length2_scaled / 2 - (i - 3)*index_width2_scaled / 3;
			rpx_index_Array[i] = POSITION_X + coordinateTransform(ux_index, uy_index, uz_index, 0);
			rpy_index_Array[i] = POSITION_Y + coordinateTransform(ux_index, uy_index, uz_index, 1);
			rpz_index_Array[i] = POSITION_Z + coordinateTransform(ux_index, uy_index, uz_index, 2);
		}
		else {				// five is for third body location
			ux_index = -(palm_breadth_scaled / 2) + (index_width1_scaled);
			uy_index = 0;
			uz_index += index_length2_scaled / 3 + index_length2_scaled / 2;
			rpx_index_Array[i] = POSITION_X + coordinateTransform(ux_index, uy_index, uz_index, 0);
			rpy_index_Array[i] = POSITION_Y + coordinateTransform(ux_index, uy_index, uz_index, 1);
			rpz_index_Array[i] = POSITION_Z + coordinateTransform(ux_index, uy_index, uz_index, 2);
		}
	}

	// This is for middle etc.
	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_middle = ux_index + index_width1_scaled + middle_width1_scaled + fingerGap;
			uy_middle = 0;
			uz_middle = (palm_length_scaled / 2) + i*(middle_length1_scaled / 2);
			rpx_middle_Array[i] = POSITION_X + coordinateTransform(ux_middle, uy_middle, uz_middle, 0);
			rpy_middle_Array[i] = POSITION_Y + coordinateTransform(ux_middle, uy_middle, uz_middle, 1);
			rpz_middle_Array[i] = POSITION_Z + coordinateTransform(ux_middle, uy_middle, uz_middle, 2);
		}
		else if (i < 5) {
			ux_middle = ux_index + index_width1_scaled + middle_width1_scaled + fingerGap;
			uy_middle = 0;
			uz_middle += (i - 2)*middle_length2_scaled / 2 + fabs(i - 4)* middle_length2_scaled / 2 - (i - 3)*middle_length2_scaled / 3;
			rpx_middle_Array[i] = POSITION_X + coordinateTransform(ux_middle, uy_middle, uz_middle, 0);
			rpy_middle_Array[i] = POSITION_Y + coordinateTransform(ux_middle, uy_middle, uz_middle, 1);
			rpz_middle_Array[i] = POSITION_Z + coordinateTransform(ux_middle, uy_middle, uz_middle, 2);
		}
		else {
			ux_middle = ux_index + index_width1_scaled + middle_width1_scaled + fingerGap;
			uy_middle = 0;
			uz_middle += middle_length2_scaled / 3 + middle_length2_scaled / 2;
			rpx_middle_Array[i] = POSITION_X + coordinateTransform(ux_middle, uy_middle, uz_middle, 0);
			rpy_middle_Array[i] = POSITION_Y + coordinateTransform(ux_middle, uy_middle, uz_middle, 1);
			rpz_middle_Array[i] = POSITION_Z + coordinateTransform(ux_middle, uy_middle, uz_middle, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_ring = ux_middle + middle_width1_scaled + ring_width1_scaled + fingerGap;
			uy_ring = 0;
			uz_ring = (palm_length_scaled / 2) + i*(ring_length1_scaled / 2);
			rpx_ring_Array[i] = POSITION_X + coordinateTransform(ux_ring, uy_ring, uz_ring, 0);
			rpy_ring_Array[i] = POSITION_Y + coordinateTransform(ux_ring, uy_ring, uz_ring, 1);
			rpz_ring_Array[i] = POSITION_Z + coordinateTransform(ux_ring, uy_ring, uz_ring, 2);
		}
		else if (i < 5) {
			ux_ring = ux_middle + middle_width1_scaled + ring_width1_scaled + fingerGap;
			uy_ring = 0;
			uz_ring += (i - 2)*ring_length2_scaled / 2 + fabs(i - 4)* ring_length2_scaled / 2 - (i - 3)*ring_length2_scaled / 3;
			rpx_ring_Array[i] = POSITION_X + coordinateTransform(ux_ring, uy_ring, uz_ring, 0);
			rpy_ring_Array[i] = POSITION_Y + coordinateTransform(ux_ring, uy_ring, uz_ring, 1);
			rpz_ring_Array[i] = POSITION_Z + coordinateTransform(ux_ring, uy_ring, uz_ring, 2);
		}
		else {
			ux_ring = ux_middle + middle_width1_scaled + ring_width1_scaled + fingerGap;
			uy_ring = 0;
			uz_ring += ring_length2_scaled / 3 + ring_length2_scaled / 2;
			rpx_ring_Array[i] = POSITION_X + coordinateTransform(ux_ring, uy_ring, uz_ring, 0);
			rpy_ring_Array[i] = POSITION_Y + coordinateTransform(ux_ring, uy_ring, uz_ring, 1);
			rpz_ring_Array[i] = POSITION_Z + coordinateTransform(ux_ring, uy_ring, uz_ring, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_pinkie = ux_ring + ring_width1_scaled + pinkie_width1_scaled + fingerGap;
			uy_pinkie = 0;
			uz_pinkie = (palm_length_scaled / 2) + i*(pinkie_length1_scaled / 2);
			rpx_pinkie_Array[i] = POSITION_X + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 0);
			rpy_pinkie_Array[i] = POSITION_Y + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 1);
			rpz_pinkie_Array[i] = POSITION_Z + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 2);
		}
		else if (i < 5) {
			ux_pinkie = ux_ring + ring_width1_scaled + pinkie_width1_scaled + fingerGap;
			uy_pinkie = 0;
			uz_pinkie += (i - 2)*pinkie_length2_scaled / 2 + fabs(i - 4)* pinkie_length2_scaled / 2 - (i - 3)*pinkie_length2_scaled / 3;
			rpx_pinkie_Array[i] = POSITION_X + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 0);
			rpy_pinkie_Array[i] = POSITION_Y + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 1);
			rpz_pinkie_Array[i] = POSITION_Z + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 2);
		}
		else {
			ux_pinkie = ux_ring + ring_width1_scaled + pinkie_width1_scaled + fingerGap;
			uy_pinkie = 0;
			uz_pinkie += pinkie_length2_scaled / 3 + pinkie_length2_scaled / 2;
			rpx_pinkie_Array[i] = POSITION_X + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 0);
			rpy_pinkie_Array[i] = POSITION_Y + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 1);
			rpz_pinkie_Array[i] = POSITION_Z + coordinateTransform(ux_pinkie, uy_pinkie, uz_pinkie, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_thumb = -(palm_breadth_scaled / 2) + (cos(M_PI_4)*thumb_length1_scaled - 0.33*thumb_length1_scaled - i*cos(M_PI_4)*(thumb_length1_scaled / 2));
			uy_thumb = 0;
			uz_thumb = -(palm_length_scaled / 2) + (i + 1)*sin(M_PI_4)*thumb_length1_scaled / 2;
			rpx_thumb_Array[i] = POSITION_X + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 0);
			rpy_thumb_Array[i] = POSITION_Y + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 1);
			rpz_thumb_Array[i] = POSITION_Z + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 2);
		}
		else if (i < 5) {
			ux_thumb += -(i - 2)*cos(M_PI_4)*((thumb_length2_scaled / 2) - (i - 3)*thumb_length2_scaled / 5);
			uy_thumb = 0;
			uz_thumb += (i - 2)*sin(M_PI_4)*(thumb_length2_scaled / 2 - (i - 3)*thumb_length2_scaled / 5);
			rpx_thumb_Array[i] = POSITION_X + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 0);
			rpy_thumb_Array[i] = POSITION_Y + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 1);
			rpz_thumb_Array[i] = POSITION_Z + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 2);
		}
		else {
			ux_thumb += -cos(M_PI_4)*((thumb_length3_scaled / 2) + (i - 3)*thumb_length2_scaled / 5);
			uy_thumb = 0;
			uz_thumb += sin(M_PI_4)*(thumb_length3_scaled / 2 + (i - 3)*thumb_length2_scaled / 5);
			rpx_thumb_Array[i] = POSITION_X + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 0);
			rpy_thumb_Array[i] = POSITION_Y + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 1);
			rpz_thumb_Array[i] = POSITION_Z + coordinateTransform(ux_thumb, uy_thumb, uz_thumb, 2);
		}
	}


	// Rotation matrix for thumb, which is rotated 45 degrees ; rotMatrixThumb = thumb
	dMatrix3 rotMatrix;
	dRFromAxisAndAngle(rotMatrix, 0, 1, 0, (-M_PI_4)); // Dont change this, otherwise thumb alingment would change
	dMatrix3 rotMatrixThumb;
	dMultiply0(rotMatrixThumb, RotationMatrix, rotMatrix, 3, 3, 3); // rotationmatrix multiplication

	// Set positions and rotations for fingers
	for (int i = 0; i < 3; i++) {
		dGeomSetPosition(gIndex[i], rpx_index_Array[(i * 2) + 1], rpy_index_Array[(i * 2) + 1], rpz_index_Array[(i * 2) + 1]);
		dGeomSetPosition(gMiddle[i], rpx_middle_Array[(i * 2) + 1], rpy_middle_Array[(i * 2) + 1], rpz_middle_Array[(i * 2) + 1]);
		dGeomSetPosition(gRing[i], rpx_ring_Array[(i * 2) + 1], rpy_ring_Array[(i * 2) + 1], rpz_ring_Array[(i * 2) + 1]);
		dGeomSetPosition(gPinkie[i], rpx_pinkie_Array[(i * 2) + 1], rpy_pinkie_Array[(i * 2) + 1], rpz_pinkie_Array[(i * 2) + 1]);
		dGeomSetPosition(gThumb[i], rpx_thumb_Array[(i * 2) + 1], rpy_thumb_Array[(i * 2) + 1], rpz_thumb_Array[(i * 2) + 1]);
		dGeomSetRotation(gIndex[i], RotationMatrix);
		dGeomSetRotation(gMiddle[i], RotationMatrix);
		dGeomSetRotation(gRing[i], RotationMatrix);
		dGeomSetRotation(gPinkie[i], RotationMatrix);
		dGeomSetRotation(gThumb[i], rotMatrixThumb);
	}

#pragma endregion

#pragma region SetJoints

	joint_palm[0] = dJointCreateHinge(world, 0);
	dJointAttach(joint_palm[0], Palm, 0);
	dJointSetHingeAnchor(joint_palm[0], POSITION_X, POSITION_Y, POSITION_Z);
	dJointSetHingeAxis(joint_palm[0], 0, 0, 1);

	// Axis for the index, middle, ring and pinkie finger hinges
	float hingeAxis[3];
	hingeAxis[0] = coordinateTransform(1, 0, 0, 0);
	hingeAxis[1] = coordinateTransform(1, 0, 0, 1);
	hingeAxis[2] = coordinateTransform(1, 0, 0, 2);

	float thumbHinge[3];
	thumbHinge[0] = coordinateTransform(1, 0, 3, 0);
	thumbHinge[1] = coordinateTransform(1, 0, 3, 1);
	thumbHinge[2] = coordinateTransform(1, 0, 3, 2);

	// Axis for the first universal axis of the first joint of the thumb
	float thumbAxis1[3];
	thumbAxis1[0] = coordinateTransform(1, 0, 1, 0);
	thumbAxis1[1] = coordinateTransform(1, 0, 1, 1);
	thumbAxis1[2] = coordinateTransform(1, 0, 1, 2);

	// Axis for the second universal axis of the first joint of the thumb
	float thumbAxis2[3];
	thumbAxis2[0] = coordinateTransform(0, 1, 0, 0);
	thumbAxis2[1] = coordinateTransform(0, 1, 0, 1);
	thumbAxis2[2] = coordinateTransform(0, 1, 0, 2);


	for (int i = 0; i < 3; i++) {
		joint_index[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_index[i], Palm, Index[i]);
		}
		else {
			dJointAttach(joint_index[i], Index[i - 1], Index[i]);
		}
		dJointSetHingeAnchor(joint_index[i], rpx_index_Array[i * 2], rpy_index_Array[i * 2], rpz_index_Array[i * 2]);
		dJointSetHingeAxis(joint_index[i], hingeAxis[0], hingeAxis[1], hingeAxis[2]);
		dJointSetHingeParam(joint_index[i], dParamHiStop, M_PI_2); // these params don't allow the joint to move more than 90 degrees or go too much to the "negative" side, lostop should be less than 0
		dJointSetHingeParam(joint_index[i], dParamLoStop, -0.01);
	}
	for (int i = 0; i < 3; i++) {
		joint_middle[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_middle[i], Palm, Middle[i]);
		}
		else {
			dJointAttach(joint_middle[i], Middle[i - 1], Middle[i]);
		}
		dJointSetHingeAnchor(joint_middle[i], rpx_middle_Array[i * 2], rpy_middle_Array[i * 2], rpz_middle_Array[i * 2]);
		dJointSetHingeAxis(joint_middle[i], hingeAxis[0], hingeAxis[1], hingeAxis[2]);
		dJointSetHingeParam(joint_middle[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_middle[i], dParamLoStop, -0.01);
	}

	for (int i = 0; i < 3; i++) {
		joint_ring[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_ring[i], Palm, Ring[i]);
		}
		else {
			dJointAttach(joint_ring[i], Ring[i - 1], Ring[i]);
		}
		dJointSetHingeAnchor(joint_ring[i], rpx_ring_Array[i * 2], rpy_ring_Array[i * 2], rpz_ring_Array[i * 2]);
		dJointSetHingeAxis(joint_ring[i], hingeAxis[0], hingeAxis[1], hingeAxis[2]);
		dJointSetHingeParam(joint_ring[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_ring[i], dParamLoStop, -0.01);
	}

	for (int i = 0; i < 3; i++) {
		joint_pinkie[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_pinkie[i], Palm, Pinkie[i]);
		}
		else {
			dJointAttach(joint_pinkie[i], Pinkie[i - 1], Pinkie[i]);
		}
		dJointSetHingeAnchor(joint_pinkie[i], rpx_pinkie_Array[i * 2], rpy_pinkie_Array[i * 2], rpz_pinkie_Array[i * 2]);
		dJointSetHingeAxis(joint_pinkie[i], hingeAxis[0], hingeAxis[1], hingeAxis[2]);
		dJointSetHingeParam(joint_pinkie[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_pinkie[i], dParamLoStop, -0.01);
	}
	// Some problems arose with the universal joints as they are not as stable as hinge joints, a lot of parameters are defined that should help with stability
	// Decreasing timestep will also help
	joint_thumb[0] = dJointCreateUniversal(world, 0);
	dJointAttach(joint_thumb[0], Palm, Thumb[0]);
	dJointSetUniversalAnchor(joint_thumb[0], rpx_thumb_Array[0], rpy_thumb_Array[0], rpz_thumb_Array[0]);
	dJointSetUniversalAxis1(joint_thumb[0], thumbAxis1[0], thumbAxis1[1], thumbAxis1[2]);
	dJointSetUniversalAxis2(joint_thumb[0], thumbAxis2[0], thumbAxis2[1], thumbAxis2[2]);
	dJointSetUniversalParam(joint_thumb[0], dParamHiStop1, dInfinity);
	dJointSetUniversalParam(joint_thumb[0], dParamLoStop1, -dInfinity);
	dJointSetUniversalParam(joint_thumb[0], dParamHiStop2, dInfinity);
	dJointSetUniversalParam(joint_thumb[0], dParamLoStop2, -dInfinity);		
	dJointSetUniversalParam(joint_thumb[0], dParamHiStop1, M_PI_4);
	dJointSetUniversalParam(joint_thumb[0], dParamLoStop1, -0.01);
	dJointSetUniversalParam(joint_thumb[0], dParamHiStop2, 0.5);
	dJointSetUniversalParam(joint_thumb[0], dParamLoStop2, -0.1);	
	dJointSetUniversalParam(joint_thumb[0], dParamFudgeFactor1, 0.2);
	dJointSetUniversalParam(joint_thumb[0], dParamFudgeFactor2, 0.2);
	dJointSetUniversalParam(joint_thumb[0], dParamBounce1, 0.2);
	dJointSetUniversalParam(joint_thumb[0], dParamBounce2, 0.2);


	for (int i = 1; i < 3; i++) {
		joint_thumb[i] = dJointCreateHinge(world, 0);
		dJointAttach(joint_thumb[i], Thumb[i - 1], Thumb[i]);
		dJointSetHingeAnchor(joint_thumb[i], rpx_thumb_Array[i * 2], rpy_thumb_Array[i * 2], rpz_thumb_Array[i * 2]);
		dJointSetHingeAxis(joint_thumb[i], thumbHinge[0], thumbHinge[1], thumbHinge[2]);
		dJointSetHingeParam(joint_thumb[i], dParamHiStop, M_PI_4);
		dJointSetHingeParam(joint_thumb[i], dParamLoStop, -0.01);

	}

#pragma endregion

#pragma endregion

#pragma region Handleft
	// Left hand is done in a similar manner to right hand so comments are minimal
#pragma region Palm2Creation

	// Create and set the palm first
	palmdim Palm2dimensions;
	Palm2dimensions = readPalmDim();
	const float palm2_length_scaled = Palm2dimensions.length / SCALE_FACTOR;
	const float palm2_breadth_scaled = Palm2dimensions.breadth / SCALE_FACTOR;
	const float palm2_thickness_scaled = Palm2dimensions.palmdepth / SCALE_FACTOR;

	Palm2 = dBodyCreate(world);
	dMassSetBox(&mPalm2, DENSITY, palm2_length_scaled, palm2_breadth_scaled, palm2_thickness_scaled);
	GPalm2 = dCreateBox(space, palm2_breadth_scaled, palm2_thickness_scaled, palm2_length_scaled);
	dBodySetMass(Palm2, &mPalm2);
	dGeomSetBody(GPalm2, Palm2);
	dGeomSetPosition(GPalm2, POSITION_X2, POSITION_Y2, POSITION_Z2);
	dGeomSetRotation(GPalm2, RotationMatrix);
#pragma endregion

#pragma region FingerBod

	// Create finger objects

	for (int i = 0; i < 3; i++) {
		Index2[i] = dBodyCreate(world);
		Middle2[i] = dBodyCreate(world);
		Ring2[i] = dBodyCreate(world);
		Pinkie2[i] = dBodyCreate(world);
		Thumb2[i] = dBodyCreate(world);
	}

#pragma endregion

#pragma region CapsuleDimensions

	//Dimensions of capsules
	indexdim dim2_i;
	indexdim dim2_i2;
	dim2_i = readIndex1Dim();
	dim2_i2 = readIndex2Dim();
	const float index2_length1_scaled = dim2_i.length / SCALE_FACTOR;		// this is the first index length of the second hand
	const float index2_width1_scaled = dim2_i.width / (2 * SCALE_FACTOR);	// this is the first index radius of the second hand
	const float index2_length2_scaled = dim2_i2.length / SCALE_FACTOR;		// this is the second index length of the second hand
	const float index2_width2_scaled = dim2_i2.width / (2 * SCALE_FACTOR);	// this is the second index radius of the second hand

	middledim dim2_m;
	middledim dim2_m2;
	dim2_m = readMiddle1Dim();
	dim2_m2 = readMiddle2Dim();
	const float middle2_length1_scaled = dim2_m.length / SCALE_FACTOR;
	const float middle2_width1_scaled = dim2_m.width / (2 * SCALE_FACTOR);
	const float middle2_length2_scaled = dim2_m2.length / SCALE_FACTOR;
	const float middle2_width2_scaled = dim2_m2.width / (2 * SCALE_FACTOR);

	ringdim dim2_r;
	ringdim dim2_r2;
	dim2_r = readRing1Dim();
	dim2_r2 = readRing2Dim();
	const float ring2_length1_scaled = dim2_r.length / SCALE_FACTOR;
	const float ring2_width1_scaled = dim2_r.width / (2 * SCALE_FACTOR);
	const float ring2_length2_scaled = dim2_r2.length / SCALE_FACTOR;
	const float ring2_width2_scaled = dim2_r2.width / (2 * SCALE_FACTOR);

	pinkiedim dim2_p;
	pinkiedim dim2_p2;
	dim2_p = readPinkie1Dim();
	dim2_p2 = readPinkie2Dim();
	const float pinkie2_length1_scaled = dim2_p.length / SCALE_FACTOR;
	const float pinkie2_width1_scaled = dim2_p.width / (2 * SCALE_FACTOR);
	const float pinkie2_length2_scaled = dim2_p2.length / SCALE_FACTOR;
	const float pinkie2_width2_scaled = dim2_p2.width / (2 * SCALE_FACTOR);

	thumbdim dim2_t;
	thumbdim dim2_t2;
	thumbdim dim2_t3;
	dim2_t = readThumb1Dim();
	dim2_t2 = readThumb2Dim();
	dim2_t3 = readThumb3Dim();
	const float thumb2_length1_scaled = dim2_t.length / SCALE_FACTOR;
	const float thumb2_width1_scaled = dim2_t.width / (2 * SCALE_FACTOR);
	const float thumb2_length2_scaled = dim2_t2.length / SCALE_FACTOR;
	const float thumb2_width2_scaled = dim2_t2.width / (2 * SCALE_FACTOR);
	const float thumb2_length3_scaled = dim2_t3.length / SCALE_FACTOR;
	const float thumb2_width3_scaled = dim2_t3.width / (2 * SCALE_FACTOR);

#pragma endregion

#pragma region Masses, geoms
	//Create masses

	dMassSetCapsule(&mIndex2, DENSITY, 1, index2_width1_scaled, index2_length1_scaled);
	dMassSetCapsule(&mMiddle2, DENSITY, 1, middle2_width1_scaled, middle2_length1_scaled);
	dMassSetCapsule(&mRing2, DENSITY, 1, ring2_width1_scaled, ring2_length1_scaled);
	dMassSetCapsule(&mPinkie2, DENSITY, 1, pinkie2_width1_scaled, pinkie2_length1_scaled);
	dMassSetCapsule(&mThumb2, DENSITY, 1, thumb2_width1_scaled, thumb2_length1_scaled);


	// Create geometries to be used in simulation

	for (i = 0; i < 3; i++) {
		if (i == 0) {
			gIndex2[i] = dCreateCapsule(space, index2_width1_scaled, index2_length1_scaled);
			gMiddle2[i] = dCreateCapsule(space, middle2_width1_scaled, middle2_length1_scaled);
			gRing2[i] = dCreateCapsule(space, ring2_width1_scaled, ring2_length1_scaled);
			gPinkie2[i] = dCreateCapsule(space, pinkie2_width1_scaled, pinkie2_length1_scaled);
			gThumb2[i] = dCreateCapsule(space, thumb2_width1_scaled, thumb2_length1_scaled);


		}
		if (i == 1 || i == 2) {
			gIndex2[i] = dCreateCapsule(space, index2_width2_scaled, index2_length2_scaled);
			gMiddle2[i] = dCreateCapsule(space, middle2_width2_scaled, middle2_length2_scaled);
			gRing2[i] = dCreateCapsule(space, ring2_width2_scaled, ring2_length2_scaled);
			gPinkie2[i] = dCreateCapsule(space, pinkie2_width2_scaled, pinkie2_length2_scaled);
			if (i == 2) {
				gThumb2[i] = dCreateCapsule(space, thumb2_width3_scaled, thumb2_length3_scaled);
				break;
			}
			gThumb2[i] = dCreateCapsule(space, thumb2_width2_scaled, thumb2_length2_scaled);
		}
	}


	// Set afore defined mass properties to bodies

	for (i = 0; i < 3; i++) {
		dBodySetMass(Index2[i], &mIndex2);
		dBodySetMass(Middle2[i], &mMiddle2);
		dBodySetMass(Ring2[i], &mRing2);
		dBodySetMass(Pinkie2[i], &mPinkie2);
		if (i == 2) {
			dBodySetMass(Thumb2[i], &mThumb2);
			break;
		}
		dBodySetMass(Thumb2[i], &mThumb2);
	}

	// Set bodies' geometries

	for (i = 0; i < 3; i++) {
		dGeomSetBody(gIndex2[i], Index2[i]);
		dGeomSetBody(gMiddle2[i], Middle2[i]);
		dGeomSetBody(gRing2[i], Ring2[i]);
		dGeomSetBody(gPinkie2[i], Pinkie2[i]);
		if (i == 2) {
			dGeomSetBody(gThumb2[i], Thumb2[i]);
			break;
		}
		dGeomSetBody(gThumb2[i], Thumb2[i]);
	}
#pragma endregion

#pragma region SetPosAndRot

	//  rp^i = R^i + A^i * u^i    
	//	R^i is the initial position vector (Defined parameters) A^i is the rotation matrix and u^i is the distance from palm center to fing2er center


	// Math operations to clarify placements
	const float fing2erGap = (palm2_breadth_scaled - pinkie2_width1_scaled * 2 - index2_width1_scaled * 2 - middle2_width1_scaled * 2 - ring2_width1_scaled * 2) / 3;

	float ux_index2;
	float uy_index2;
	float uz_index2;
	float rpx_index2_Array[6];
	float rpy_index2_Array[6];
	float rpz_index2_Array[6];

	float ux_middle2;
	float uy_middle2;
	float uz_middle2;
	float rpx_middle2_Array[6];
	float rpy_middle2_Array[6];
	float rpz_middle2_Array[6];

	float ux_ring2;
	float uy_ring2;
	float uz_ring2;
	float rpx_ring2_Array[6];
	float rpy_ring2_Array[6];
	float rpz_ring2_Array[6];

	float ux_pinkie2;
	float uy_pinkie2;
	float uz_pinkie2;
	float rpx_pinkie2_Array[6];
	float rpy_pinkie2_Array[6];
	float rpz_pinkie2_Array[6];

	float ux_thumb2;
	float uy_thumb2;
	float uz_thumb2;
	float rpx_thumb2_Array[6];
	float rpy_thumb2_Array[6];
	float rpz_thumb2_Array[6];

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_index2 = palm2_breadth_scaled / 2 - (index2_width1_scaled);
			uy_index2 = 0;
			uz_index2 = (palm2_length_scaled / 2) + i*(index2_length1_scaled / 2) + (fabs(i*(i - 1)) / 2)*index2_width1_scaled / 2;
			rpx_index2_Array[i] = POSITION_X2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 0);
			rpy_index2_Array[i] = POSITION_Y2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 1);
			rpz_index2_Array[i] = POSITION_Z2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 2);
		}
		else if (i < 5) {
			uy_index2 = 0;
			uz_index2 += fabs(i - 4)*index2_width2_scaled / 2 + (i - 2)*index2_length2_scaled / 2 - (i - 3)*index2_width2_scaled / 3;
			rpx_index2_Array[i] = POSITION_X2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 0);
			rpy_index2_Array[i] = POSITION_Y2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 1);
			rpz_index2_Array[i] = POSITION_Z2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 2);
		}
		else {
			uy_index2 = 0;
			uz_index2 +=  index2_length2_scaled / 3+ index2_length2_scaled/2;
			rpx_index2_Array[i] = POSITION_X2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 0);
			rpy_index2_Array[i] = POSITION_Y2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 1);
			rpz_index2_Array[i] = POSITION_Z2 + coordinateTransform(ux_index2, uy_index2, uz_index2, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_middle2 = ux_index2 - index2_width1_scaled - middle2_width1_scaled - fing2erGap;
			uy_middle2 = 0;
			uz_middle2 = (palm2_length_scaled / 2) + i*(middle2_length1_scaled / 2);
			rpx_middle2_Array[i] = POSITION_X2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 0);
			rpy_middle2_Array[i] = POSITION_Y2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 1);
			rpz_middle2_Array[i] = POSITION_Z2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 2);
		}
		else if (i < 5) {
			uy_middle2 = 0;
			uz_middle2 += (i - 2)*middle2_length2_scaled / 2 + fabs(i - 4)* middle2_length2_scaled / 2 - (i - 3)*middle2_length2_scaled / 3;
			rpx_middle2_Array[i] = POSITION_X2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 0);
			rpy_middle2_Array[i] = POSITION_Y2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 1);
			rpz_middle2_Array[i] = POSITION_Z2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 2);
		}
		else {
			uy_middle2 = 0;
			uz_middle2 += middle2_length2_scaled / 3 + middle2_length2_scaled / 2;
			rpx_middle2_Array[i] = POSITION_X2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 0);
			rpy_middle2_Array[i] = POSITION_Y2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 1);
			rpz_middle2_Array[i] = POSITION_Z2 + coordinateTransform(ux_middle2, uy_middle2, uz_middle2, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_ring2 = ux_middle2 - middle2_width1_scaled - ring2_width1_scaled - fing2erGap;
			uy_ring2 = 0;
			uz_ring2 = (palm2_length_scaled / 2) + i*(ring2_length1_scaled / 2);
			rpx_ring2_Array[i] = POSITION_X2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 0);
			rpy_ring2_Array[i] = POSITION_Y2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 1);
			rpz_ring2_Array[i] = POSITION_Z2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 2);
		}
		else if (i < 5) {
			uy_ring2 = 0;
			uz_ring2 += (i - 2)*ring2_length2_scaled / 2 + fabs(i - 4)* ring2_length2_scaled / 2 - (i - 3)*ring2_length2_scaled / 3;
			rpx_ring2_Array[i] = POSITION_X2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 0);
			rpy_ring2_Array[i] = POSITION_Y2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 1);
			rpz_ring2_Array[i] = POSITION_Z2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 2);
		}
		else {
			uy_ring2 = 0;
			uz_ring2 += ring2_length2_scaled / 3 + ring2_length2_scaled / 2;
			rpx_ring2_Array[i] = POSITION_X2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 0);
			rpy_ring2_Array[i] = POSITION_Y2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 1);
			rpz_ring2_Array[i] = POSITION_Z2 + coordinateTransform(ux_ring2, uy_ring2, uz_ring2, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_pinkie2 = ux_ring2 - ring2_width1_scaled - pinkie2_width1_scaled - fing2erGap;
			uy_pinkie2 = 0;
			uz_pinkie2 = (palm2_length_scaled / 2) + i*(pinkie2_length1_scaled / 2);
			rpx_pinkie2_Array[i] = POSITION_X2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 0);
			rpy_pinkie2_Array[i] = POSITION_Y2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 1);
			rpz_pinkie2_Array[i] = POSITION_Z2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 2);
		}
		else if (i < 5) {
			uy_pinkie2 = 0;
			uz_pinkie2 += (i - 2)*pinkie2_length2_scaled / 2 + fabs(i - 4)* pinkie2_length2_scaled / 2 - (i - 3)*pinkie2_length2_scaled / 3;
			rpx_pinkie2_Array[i] = POSITION_X2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 0);
			rpy_pinkie2_Array[i] = POSITION_Y2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 1);
			rpz_pinkie2_Array[i] = POSITION_Z2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 2);
		}
		else {
			uy_pinkie2 = 0;
			uz_pinkie2 += pinkie2_length2_scaled / 3 + pinkie2_length2_scaled / 2;
			rpx_pinkie2_Array[i] = POSITION_X2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 0);
			rpy_pinkie2_Array[i] = POSITION_Y2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 1);
			rpz_pinkie2_Array[i] = POSITION_Z2 + coordinateTransform(ux_pinkie2, uy_pinkie2, uz_pinkie2, 2);
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ux_thumb2 = (palm2_breadth_scaled / 2) - (cos(M_PI_4)*thumb2_length1_scaled - 0.33*thumb2_length1_scaled - i*cos(M_PI_4)*(thumb2_length1_scaled / 2));
			uy_thumb2 = 0;
			uz_thumb2 = -(palm2_length_scaled / 2) + (i + 1)*sin(M_PI_4)*thumb2_length1_scaled / 2;
			rpx_thumb2_Array[i] = POSITION_X2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 0);
			rpy_thumb2_Array[i] = POSITION_Y2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 1);
			rpz_thumb2_Array[i] = POSITION_Z2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 2);
		}
		else if (i < 5) {
			ux_thumb2 += (i - 2)*cos(M_PI_4)*((thumb2_length2_scaled / 2) - (i - 3)*thumb2_length2_scaled / 5);
			uy_thumb2 = 0;
			uz_thumb2 += (i - 2)*sin(M_PI_4)*(thumb2_length2_scaled / 2 - (i - 3)*thumb2_length2_scaled / 5);
			rpx_thumb2_Array[i] = POSITION_X2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 0);
			rpy_thumb2_Array[i] = POSITION_Y2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 1);
			rpz_thumb2_Array[i] = POSITION_Z2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 2);
		}
		else {
			ux_thumb2 += cos(M_PI_4)*((thumb2_length3_scaled / 2) + (i - 3)*thumb2_length2_scaled / 5);
			uy_thumb2 = 0;
			uz_thumb2 += sin(M_PI_4)*(thumb2_length3_scaled / 2 + (i - 3)*thumb2_length2_scaled / 5);
			rpx_thumb2_Array[i] = POSITION_X2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 0);
			rpy_thumb2_Array[i] = POSITION_Y2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 1);
			rpz_thumb2_Array[i] = POSITION_Z2 + coordinateTransform(ux_thumb2, uy_thumb2, uz_thumb2, 2);
		}
	}

	// Rotation matrices for rotMatrix = index2,middle2,ring2,pinkie2 ; rotMatrixThumb2 = thumb2
	dMatrix3 rotMatrix2;
	dRFromAxisAndAngle(rotMatrix2, 0, 1, 0, (M_PI_4));
	dMatrix3 rotMatrixThumb2;
	dMultiply0(rotMatrixThumb2, RotationMatrix, rotMatrix2, 3, 3, 3);

	// Set positions and rotations for fing2ers
	for (int i = 0; i < 3; i++) {
		dGeomSetPosition(gIndex2[i], rpx_index2_Array[(i * 2) + 1], rpy_index2_Array[(i * 2) + 1], rpz_index2_Array[(i * 2) + 1]);
		dGeomSetPosition(gMiddle2[i], rpx_middle2_Array[(i * 2) + 1], rpy_middle2_Array[(i * 2) + 1], rpz_middle2_Array[(i * 2) + 1]);
		dGeomSetPosition(gRing2[i], rpx_ring2_Array[(i * 2) + 1], rpy_ring2_Array[(i * 2) + 1], rpz_ring2_Array[(i * 2) + 1]);
		dGeomSetPosition(gPinkie2[i], rpx_pinkie2_Array[(i * 2) + 1], rpy_pinkie2_Array[(i * 2) + 1], rpz_pinkie2_Array[(i * 2) + 1]);
		dGeomSetPosition(gThumb2[i], rpx_thumb2_Array[(i * 2) + 1], rpy_thumb2_Array[(i * 2) + 1], rpz_thumb2_Array[(i * 2) + 1]);
		dGeomSetRotation(gIndex2[i], RotationMatrix);
		dGeomSetRotation(gMiddle2[i], RotationMatrix);
		dGeomSetRotation(gRing2[i], RotationMatrix);
		dGeomSetRotation(gPinkie2[i], RotationMatrix);
		dGeomSetRotation(gThumb2[i], rotMatrixThumb2);
	}

#pragma endregion

#pragma region SetJoints

	joint_palm2[0] = dJointCreateHinge(world, 0);
	dJointAttach(joint_palm2[0], Palm2, 0);
	dJointSetHingeAnchor(joint_palm2[0], POSITION_X2, POSITION_Y2, POSITION_Z2);
	dJointSetHingeAxis(joint_palm2[0], 0, 0, 1);

	float hinge2Axis[3];
	hinge2Axis[0] = coordinateTransform(1, 0, 0, 0);
	hinge2Axis[1] = coordinateTransform(1, 0, 0, 1);
	hinge2Axis[2] = coordinateTransform(1, 0, 0, 2);

	float thumb2Hinge[3];
	thumb2Hinge[0] = coordinateTransform(-1, 0, 3, 0);
	thumb2Hinge[1] = coordinateTransform(-1, 0, 3, 1);
	thumb2Hinge[2] = coordinateTransform(-1, 0, 3, 2);

	float thumb2Axis1[3];
	thumb2Axis1[0] = coordinateTransform(-1, 0, 1, 0);
	thumb2Axis1[1] = coordinateTransform(-1, 0, 1, 1);
	thumb2Axis1[2] = coordinateTransform(-1, 0, 1, 2);

	float thumb2Axis2[3];
	thumb2Axis2[0] = coordinateTransform(0, 1, 0, 0);
	thumb2Axis2[1] = coordinateTransform(0, 1, 0, 1);
	thumb2Axis2[2] = coordinateTransform(0, 1, 0, 2);


	for (int i = 0; i < 3; i++) {
		joint_index2[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_index2[i], Palm2, Index2[i]);
		}
		else {
			dJointAttach(joint_index2[i], Index2[i - 1], Index2[i]);
		}
		dJointSetHingeAnchor(joint_index2[i], rpx_index2_Array[i * 2], rpy_index2_Array[i * 2], rpz_index2_Array[i * 2]);
		dJointSetHingeAxis(joint_index2[i], hinge2Axis[0], hinge2Axis[1], hinge2Axis[2]);
		dJointSetHingeParam(joint_index2[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_index2[i], dParamLoStop, -0.01);
	}

	for (int i = 0; i < 3; i++) {
		joint_middle2[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_middle2[i], Palm2, Middle2[i]);
		}
		else {
			dJointAttach(joint_middle2[i], Middle2[i - 1], Middle2[i]);
		}
		dJointSetHingeAnchor(joint_middle2[i], rpx_middle2_Array[i * 2], rpy_middle2_Array[i * 2], rpz_middle2_Array[i * 2]);
		dJointSetHingeAxis(joint_middle2[i], hinge2Axis[0], hinge2Axis[1], hinge2Axis[2]);
		dJointSetHingeParam(joint_middle2[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_middle2[i], dParamLoStop, -0.01);
	}

	for (int i = 0; i < 3; i++) {
		joint_ring2[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_ring2[i], Palm2, Ring2[i]);
		}
		else {
			dJointAttach(joint_ring2[i], Ring2[i - 1], Ring2[i]);
		}
		dJointSetHingeAnchor(joint_ring2[i], rpx_ring2_Array[i * 2], rpy_ring2_Array[i * 2], rpz_ring2_Array[i * 2]);
		dJointSetHingeAxis(joint_ring2[i], hinge2Axis[0], hinge2Axis[1], hinge2Axis[2]);
		dJointSetHingeParam(joint_ring2[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_ring2[i], dParamLoStop, -0.01);
	}

	for (int i = 0; i < 3; i++) {
		joint_pinkie2[i] = dJointCreateHinge(world, 0);
		if (i == 0) {
			dJointAttach(joint_pinkie2[i], Palm2, Pinkie2[i]);
		}
		else {
			dJointAttach(joint_pinkie2[i], Pinkie2[i - 1], Pinkie2[i]);
		}
		dJointSetHingeAnchor(joint_pinkie2[i], rpx_pinkie2_Array[i * 2], rpy_pinkie2_Array[i * 2], rpz_pinkie2_Array[i * 2]);
		dJointSetHingeAxis(joint_pinkie2[i], hinge2Axis[0], hinge2Axis[1], hinge2Axis[2]);
		dJointSetHingeParam(joint_pinkie2[i], dParamHiStop, M_PI_2);
		dJointSetHingeParam(joint_pinkie2[i], dParamLoStop, -0.01);
	}

	joint_thumb2[0] = dJointCreateUniversal(world, 0);
	dJointAttach(joint_thumb2[0], Palm2, Thumb2[0]);
	dJointSetUniversalAnchor(joint_thumb2[0], rpx_thumb2_Array[0], rpy_thumb2_Array[0], rpz_thumb2_Array[0]);
	dJointSetUniversalAxis1(joint_thumb2[0], thumb2Axis1[0], thumb2Axis1[1], thumb2Axis1[2]);
	dJointSetUniversalAxis2(joint_thumb2[0], thumb2Axis2[0], thumb2Axis2[1], thumb2Axis2[2]);
	dJointSetUniversalParam(joint_thumb2[0], dParamLoStop1, -dInfinity);
	dJointSetUniversalParam(joint_thumb2[0], dParamHiStop1, dInfinity);
	dJointSetUniversalParam(joint_thumb2[0], dParamLoStop2, -dInfinity);
	dJointSetUniversalParam(joint_thumb2[0], dParamHiStop2, dInfinity);
	dJointSetUniversalParam(joint_thumb2[0], dParamLoStop1, -M_PI_4);
	dJointSetUniversalParam(joint_thumb2[0], dParamHiStop1, 0.01);
	dJointSetUniversalParam(joint_thumb2[0], dParamLoStop2, -0.5);
	dJointSetUniversalParam(joint_thumb2[0], dParamHiStop2, 0.1);
	dJointSetUniversalParam(joint_thumb2[0], dParamFudgeFactor1, 0.2);
	dJointSetUniversalParam(joint_thumb2[0], dParamFudgeFactor2, 0.2);
	dJointSetUniversalParam(joint_thumb2[0], dParamBounce1, 0.2);
	dJointSetUniversalParam(joint_thumb2[0], dParamBounce2, 0.2);

	for (int i = 1; i < 3; i++) {
		joint_thumb2[i] = dJointCreateHinge(world, 0);
		dJointAttach(joint_thumb2[i], Thumb2[i - 1], Thumb2[i]);
		dJointSetHingeAnchor(joint_thumb2[i], rpx_thumb2_Array[i * 2], rpy_thumb2_Array[i * 2], rpz_thumb2_Array[i * 2]);
		dJointSetHingeAxis(joint_thumb2[i], thumb2Hinge[0], thumb2Hinge[1], thumb2Hinge[2]);
		dJointSetHingeParam(joint_thumb2[i], dParamLoStop, -M_PI_4);
		dJointSetHingeParam(joint_thumb2[i], dParamHiStop, 0.01);

	}
	
#pragma endregion

#pragma endregion

#pragma region MainEnd
	//run simulation
#ifdef Time_results // time how long it takes to solve a number of loops (i < )
	for (i = 0; i < 100; i++) {
		simLoop(0);
		dTimerNow(" Loop time: ");
	}
#endif // Time_results
#if Draw
	dsSimulationLoop(argc, argv, 1000, 1000, &fn);
#endif
	// clean up
	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
#ifdef Time_results
	dTimerEnd();
	dTimerReport(data, 1);
	fclose(data);
#endif // Time_results

	return 0;
#pragma endregion
}
