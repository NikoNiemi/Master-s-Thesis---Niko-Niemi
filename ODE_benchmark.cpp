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

// Last updated: 19.6.2017
// Done by Niko Niemi as a part of his Master's Thesis

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

// If double precision math is used
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#endif


/*============ TWO PARAMETERS HAVE TO BE DEFINED ============

Draw defines whether the simulation is drawn or not (uses drawstuff lib). Defined to 0 or 1.
- if Draw is set to 1, the simulation times are not recorded
Consider_contact has to be defined to either 0 or 1 depending on whether contact is taken into account or not.
- Accounts for only contact between slider and ground.

- When benchmarked, filenames were changed accordingly as well as NumberOfLoops and simstep. 

===========================================================*/

#define Draw 0
#if !Draw
#define Time_results 1
#endif 

#define Consider_contact 0

// Simulated time = NumberOfLoops * simstep
#define NumberOfLoops 50000
#define simstep 0.0001

// Densities are redundant since inertia and mass are specified
#define density_slider 8000
#define density_crank 3500
#define density_rod 6000

// Determine identifications for objects in the simulation environment
static dWorldID world;
static dSpaceID space;
static dGeomID slider_geom, crank_geom, connecting_rod_geom, support_geom[3];
static dJointID joint[6];
static dMass m_slider, m_crank, m_connecting_rod, m_support[2];
static dBodyID slider, crank, connecting_rod, support[3];
static dJointGroupID contactgroup;

double prevpos = 0;

/*	This is called by dSpaceCollide when two objects in space are potentially colliding. 
	Since contact condition is not used due to the prismatic constraint, these values don't affect accuracy benchmark. 
*/
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	dContact contact;
	if (b1 && b2 && dAreConnected(b1, b2)) return;

	// Contact.surface.mode = dContactApprox0;
	contact.surface.mode = dContactSoftERP | dContactApprox1;

	// Friction parameter
	contact.surface.soft_erp = 0.2;
	contact.surface.mu = 0.2;
	contact.surface.mu2 = 0.2;

	// Bounce is the amount of "bouncyness".
	contact.surface.bounce = .09;

	// Bounce_vel is the minimum incoming velocity to cause a bounce
	contact.surface.bounce_vel = .01;

	// Constraint force mixing parameter
	contact.surface.soft_cfm = 0.1;
	if (int numc = dCollide(o1, o2, 1, &contact.geom, sizeof(dContact))) {
		dJointID c = dJointCreateContact(world, contactgroup, &contact);
		dJointAttach(c, b1, b2);
	}
}

// Start simulation - set viewpoint (only needed in visualization)
static void start()
{
#ifdef Time_results 1
	dTimerStart("Start the timer\n\n");
#endif
	static float xyz[3] = { 0.1f,-0.6f,0.3f };
	static float hpr[3] = { 90.000f,-17.0000f,0.0000f };
	dsSetViewpoint(xyz, hpr);
	dWorldSetERP(world, 0.2);		// ERP should be a value between 0.2 - 0.8, to minimize effect on results 0.2 is used
	dWorldSetDamping(world, 0, 0);	// Damping is set to 0 for both rotational and translational movement
}

// simulation loop
static void simLoop(int pause)
{
	FILE *data = fopen("Slider_vel_step0,1ms_5s.txt","a");
	dJointAddHingeTorque(joint[1], 0.1);
	int i;
	static double time = 0;
	double speed;
	const dReal *pos;
	const dReal *R;

	if (!pause) {

#if Consider_contact // Consider_contact is defined in the beginning of this file
		// find collisions and add contact joints
		dSpaceCollide(space, 0, &nearCallback);
#endif // 1

		// step the simulation
		dWorldQuickStep(world, simstep);
		pos = dGeomGetPosition(slider_geom);		
		if (prevpos != 0) {
			speed = ((double)pos[0] - prevpos) / simstep; // Calculate speed of slider
			fprintf(data, "Slider speed: %lf\n", speed);
		}
		fclose(data);

		// Remove all contact joints
		dJointGroupEmpty(contactgroup);

		const dReal *paikka;
		prevpos = pos[0];

		// Draw each geometry. This is for verification purposes.
#if Draw
		//redraw sphere at new location
		dsSetDrawMode(0);
		dVector3 sides;

		for (i = 0; i < 5; i++) {
			if (i == 0) {
				dsSetColor(1, 0, 0);
				pos = dGeomGetPosition(slider_geom);
				dGeomBoxGetLengths(slider_geom, sides);
				R = dGeomGetRotation(slider_geom);
				dsDrawBox(pos, R, sides);
			}
			if (i == 1) {
				dsSetColor(0, 0, 1);
				pos = dGeomGetPosition(crank_geom);
				dGeomBoxGetLengths(crank_geom, sides);
				R = dGeomGetRotation(crank_geom);
				dsDrawBox(pos, R, sides);
			}
			if (i == 2) {
				dsSetColor(1, 1, 1);
				pos = dGeomGetPosition(connecting_rod_geom);
				R = dGeomGetRotation(connecting_rod_geom);
				dsDrawCylinder(pos, R, 0.3, 0.01);
			}
			if (i == 3 && Consider_contact) {
				pos = dGeomGetPosition(support_geom[0]);
				dGeomBoxGetLengths(support_geom[0], sides);
				R = dGeomGetRotation(support_geom[0]);
				dsDrawBox(pos, R, sides);
			}
			if (i == 4 && Consider_contact) {
				pos = dGeomGetPosition(support_geom[1]);
				dGeomBoxGetLengths(support_geom[1], sides);
				R = dGeomGetRotation(support_geom[1]);
				dsDrawBox(pos, R, sides);
			}

		}
		paikka = dBodyGetPosition(slider);
#endif
	}
}

int main(int argc, char **argv)
{
	int i;
	int k;
	char *filename;

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
	dInitODE2(0);
	dAllocateODEDataForThread(dAllocateMaskAll);

	// Results are timed if Time_results is defined i.e. Draw is defined as 0
#ifdef Time_results
	if (Consider_contact) {
		filename = "timebench_step5ms_contact.txt"; // This name has to be changed manually as benchmarks are performed
	}
	else
	{
		filename = "timebench_step5ms.txt";			// This name has to be changed manually as benchmarks are performed
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
	dWorldSetGravity(world, 0, 0, -9.81);
	dWorldSetCFM(world, 0);
	dCreatePlane(space, 0, 0, 1, 0);
	contactgroup = dJointGroupCreate(0);

	// Create objects
	slider = dBodyCreate(world);
	crank = dBodyCreate(world);
	connecting_rod = dBodyCreate(world);

	// Define inertia vectors (inertia matrix is symmetric and diagonal)
	dVector3 inertiaslider = { 0.0008, 0.00008,0.0008 };
	dVector3 inertiacrank = { 0.0001, 0.00001, 0.0001 };
	dVector3 inertiaconnecting_rod = { 0.004, 0.0004, 0.004 };

	////Define mass properties
	dMassSetParameters(&m_slider, 2.0, 0, 0, 0, inertiaslider[0], inertiaslider[1], inertiaslider[2], 0, 0, 0);
	dMassSetParameters(&m_crank, 0.12, 0, 0, 0, inertiacrank[0], inertiacrank[1], inertiacrank[2], 0, 0, 0);
	dMassSetParameters(&m_connecting_rod, 0.5, 0, 0, 0, inertiaconnecting_rod[0], inertiaconnecting_rod[1], inertiaconnecting_rod[2], 0, 0, 0);

	// Create geometries to be used in simulation
	slider_geom = dCreateBox(space, 0.08, 0.04, 0.05);			// Slider
	crank_geom = dCreateBox(space, 0.02, 0.08, 0.02);			// Crank
	connecting_rod_geom = dCreateCylinder(space, 0.01, 0.3);	// Connecting rod
	
	// Set afore defined mass properties to bodies
	dBodySetMass(slider, &m_slider);
	dBodySetMass(crank, &m_crank);
	dBodySetMass(connecting_rod, &m_connecting_rod);

	// Set bodies' geometries
	dGeomSetBody(slider_geom, slider);
	dGeomSetBody(crank_geom, crank);
	dGeomSetBody(connecting_rod_geom, connecting_rod);

	// Set correct positions for the geoms
	dGeomSetPosition(slider_geom, 0.277, 0, 0.026);
	dGeomSetPosition(crank_geom, 0, 0.06, 0.12);
	dGeomSetPosition(connecting_rod_geom, 0.1385, 0.01, 0.07);

	// Set correct rotation for connecting_rod
	dMatrix3 angle1;
	dMatrix3 angle2;
	dMatrix3 totangle;
	dReal kulma1 = 110 * M_PI / 180;
	dReal kulma2 = 3.814 * M_PI / 180;
	dRFromAxisAndAngle(angle1, 0, 1, 0, kulma1);
	dRFromAxisAndAngle(angle2, 1, 0, 0, kulma2);
	dMultiply0(totangle, angle1, angle2, 3, 3, 3);
	dGeomSetRotation(connecting_rod_geom, totangle);

	// Set corresponding joints
#if !Consider_contact
	joint[0] = dJointCreateSlider(world, 0);
	dJointAttach(joint[0], slider, 0);
	dJointSetSliderAxis(joint[0], 1, 0, 0);
	dJointSetSliderParam(joint[0], dParamVel, 100);
#endif
	joint[1] = dJointCreateHinge(world, 0);
	dJointAttach(joint[1], crank, 0);
	dJointSetHingeAnchor(joint[1], 0, 0.1, 0.12);
	dJointSetHingeAxis(joint[1], 1, 0, 0);
	dJointSetHingeParam(joint[1], dParamVel, 100);

	joint[2] = dJointCreateBall(world, 0);
	dJointAttach(joint[2], crank, connecting_rod);
	dJointSetBallAnchor(joint[2], 0, 0.02, 0.12);
	dJointSetBallParam(joint[2], dParamVel, 100);

	joint[3] = dJointCreateUniversal(world, 0);
	dJointAttach(joint[3], slider, connecting_rod);
	dJointSetUniversalAnchor(joint[3], 0.277, 0, 0.025);
	dJointSetUniversalAxis1(joint[3], 1, 0, 0);
	dJointSetUniversalAxis2(joint[3], 0, 1, 0);
	dJointSetUniversalParam(joint[3], dParamVel1, 100);
	dJointSetUniversalParam(joint[3], dParamVel2, 100);

	// If contact condition is considered, then supporting boxes are created around the slider
#if Consider_contact
	support[0] = dBodyCreate(world);
	support[1] = dBodyCreate(world);

	support_geom[0] = dCreateBox(space, 0.3, 0.04, 0.03);
	support_geom[1] = dCreateBox(space, 0.3, 0.04, 0.03);

	dGeomSetBody(support_geom[0], support[0]);
	dGeomSetBody(support_geom[1], support[1]);

	dMassSetBox(&m_support[0], 100, 0.5, 0.04, 0.05);
	dMassSetBox(&m_support[1], 100, 0.5, 0.04, 0.05);

	dBodySetMass(support[0], &m_support[0]);
	dBodySetMass(support[1], &m_support[1]);

	dGeomSetPosition(support_geom[0], 0.3, -0.042, 0.016);
	dGeomSetPosition(support_geom[1], 0.37, 0.042, 0.016);
#endif // Consider_contact

	//run simulation
#ifdef Time_results
	for (i = 0; i < NumberOfLoops; i++) {
		simLoop(0);
		dTimerNow("Loop time: ");
	}
#endif // Time_results

#if Draw
	dsSimulationLoop(argc, argv, 352, 288, &fn);
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
}
