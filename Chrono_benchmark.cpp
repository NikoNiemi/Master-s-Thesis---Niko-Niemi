//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

// Last updated: 8.6.2017
// Done by Niko Niemi as a part of his Master's Thesis

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include <iostream>
#include <fstream>
#include <string>
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverSOR.h"

// Use the namespace of Chrono
using namespace std;
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

/*	TIMESTEP defines the timestep used in the dynamic simulation
	DRAW determines whether a visualization is provided, if 0 benchmark is performed instead.
*/
#define TIMESTEP 0.01
#define DRAW 0

#if DRAW

int main(int argc, char* argv[]) {

	// Set path to Chrono data directory
	SetChronoDataPath(CHRONO_DATA_DIR);

	// Create a Chrono physical system
	ChSystem mphysicalSystem;

	// Set gravity
	mphysicalSystem.Set_G_acc(ChVector<>(0, -9.81, 0));

	// Create the Irrlicht visualization 
	ChIrrApp application(&mphysicalSystem, L"3D slidercrank", core::dimension2d<u32>(800, 600),
		false);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.6, 0.5, -1.1),
		core::vector3df(0, -0.6, 0));


	ChTimer<double> timer;

	// Create a fixed floor
	auto floorBody = std::make_shared<ChBodyEasyBox>(1.3, 0.1, 1,  // x, y, z dimensions
		1,				// density
		true,			// contact geometry
		true			// enable visualization geometry
		);
	floorBody->SetPos(ChVector<>(0, -0.05, 0));
	floorBody->SetBodyFixed(true);
	mphysicalSystem.Add(floorBody);

	// Create slider body
	auto slider = std::make_shared<ChBodyEasyBox>(0.08, 0.05, 0.04, 8000,true,true);
	ChVector<>Slider_inertia(8, 8, 0.0008);
	slider->SetInertia(ChMatrix33<>(Slider_inertia));
	slider->SetPos(ChVector<>(0.277, 0.026, 0));
	mphysicalSystem.Add(slider);

	// Create crank body
	auto crank = std::make_shared<ChBodyEasyBox>(0.02, 0.02, 0.08,3500,true,true);	
	ChVector<>crank_inertia(0.1, 0.1, 0.1);
	crank->SetInertia(ChMatrix33<>(crank_inertia));
	crank->SetPos(ChVector<>(0, 0.12, 0.06));
	mphysicalSystem.Add(crank);

	// Create connecting rod body
	auto connecting_rod = std::make_shared<ChBodyEasyCylinder>(0.01, 0.3,6000,false,true);
	ChVector<>connecting_rod_inertia(0.004, 0.004, 0.004);
	connecting_rod->SetInertia(ChMatrix33<>(connecting_rod_inertia));
	connecting_rod->SetPos(ChVector<>(0.1385, 0.07, 0.01));

	// Set connecting rod initial rotation using quaternions
	ChQuaternion<> qC = QUNIT;
	ChQuaternion<> qA = Q_from_AngAxis(3.8 * CH_C_DEG_TO_RAD, VECT_Y);
	ChQuaternion<> qB = Q_from_AngAxis(70 * CH_C_DEG_TO_RAD, VECT_Z);
	qC = qA * qB;
	connecting_rod->SetRot(qC);
	mphysicalSystem.Add(connecting_rod);

	// Constraints:

	// Either hinge or chlinkengine can be used for the crank and floor joint

	//auto hingeLink =
	//	std::make_shared<ChLinkMateGeneric>(true, true, true, false, true, true);  // x,y,z,Rx,Ry,Rz constrains
	//ChFrame<> link_position_abs(ChVector<>(0, 0.12, 0.1));
	//hingeLink->Initialize(crank,        // the 1st body to connect
	//	floorBody,           // the 2nd body to connect
	//	false,               // the two following frames are in absolute, not relative, coords.
	//	link_position_abs,   // the link reference attached to 1st body
	//	link_position_abs);  // the link reference attached to 2nd body
	//mphysicalSystem.Add(hingeLink);

	ChVector<> poscrank(0, 0.12, 0.1);
	auto link_engine = std::make_shared<ChLinkEngine>();
	link_engine->Initialize(floorBody, crank, ChCoordsys<>(poscrank, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
	link_engine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
	link_engine->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	link_engine->Set_tor_funct(std::make_shared<ChFunction_Const>(0.1));
	mphysicalSystem.Add(link_engine);

	auto sphericalLink =
		std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> link2_position_abs(ChVector<>(0, 0.12, 0.02));
	sphericalLink->Initialize(crank,        // the 1st body to connect
		connecting_rod,						// the 2nd body to connect
		false,								// the two following frames are in absolute coords
		link2_position_abs,					// the link reference attached to 1st body
		link2_position_abs);				// the link reference attached to 2nd body
	mphysicalSystem.Add(sphericalLink);		// Add the joint to the physical system

	auto universalLink =
		std::make_shared<ChLinkMateGeneric>(true, true, true, false, true, false);
	ChFrame<> link3_position_abs(ChVector<>(0.277, 0.025, 0));
	universalLink->Initialize(slider,connecting_rod,false,link3_position_abs,link3_position_abs);
	mphysicalSystem.Add(universalLink);

	auto prismaticLink =
		std::make_shared<ChLinkMateGeneric>(false, true, true, true, true, true);
	ChFrame<> link4_position_abs(ChVector<>(0.277, 0.012, 0));
	prismaticLink->Initialize(slider,floorBody,false,link4_position_abs,link4_position_abs);
	mphysicalSystem.Add(prismaticLink);

	// Create a marker to track slider
	auto my_marker_a = std::make_shared<ChMarker>();
	slider->AddMarker(my_marker_a);

	auto colorcrank = std::make_shared<ChColorAsset>();
	colorcrank->SetColor(ChColor(0, 0, 1));
	crank->AddAsset(colorcrank);
	auto colorslider = std::make_shared<ChColorAsset>();
	colorslider->SetColor(ChColor(1, 0, 0));
	slider->AddAsset(colorslider);
	auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(color);

	application.AssetBindAll();
	application.AssetUpdateAll();

	application.SetTimestep(0.01);
	mphysicalSystem.SetSolverType(ChSolver::Type::APGD);
	mphysicalSystem.SetMaxItersSolverSpeed(100);
	mphysicalSystem.SetSolverWarmStarting(true);
	mphysicalSystem.SetTol(0.0000001);
	mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

	double timing = 0;
	while (application.GetDevice()->run()) {
		timer.start();

		application.BeginScene();
		timing += 0.001;
		application.DrawAll();

		// Perform integration step
		application.DoStep();
		application.EndScene();
		timer.stop();
		unsigned long long time = timer.GetTimeMilliseconds();
		cout << "Time: "<< timing <<"\t Slider velocity: " << my_marker_a->GetAbsCoord_dt().pos.x() << endl;
		timer.reset();
	}

	return 0;
}
#endif // 0

#if !DRAW
int main(int argc, char* argv[]) {

	SetChronoDataPath(CHRONO_DATA_DIR);

	ChSystem mphysicalSystem;

	mphysicalSystem.Set_G_acc(ChVector<>(0, -9.81, 0));
	ChTimer<double> timer;

	auto floorBody = std::make_shared<ChBody>();
	floorBody->SetPos(ChVector<>(0, -0.05, 0));
	floorBody->SetBodyFixed(true);
	mphysicalSystem.Add(floorBody);

	// Create slider body
	auto slider = std::make_shared<ChBody>();
	slider->SetPos(ChVector<>(0.277, 0.026, 0));
	slider->SetPos_dt(ChVector<>(0, 0, 0));
	ChVector<>Slider_inertia(0.0001, 0.00001, 0.0001);
	slider->SetInertia(ChMatrix33<>(Slider_inertia));
	mphysicalSystem.Add(slider);

	// Create crank body
	auto crank = std::make_shared<ChBody>(); 
	crank->SetPos(ChVector<>(0, 0.12, 0.06));
	crank->SetPos_dt(ChVector<>(0, 0, 0));
	ChVector<>crank_inertia(0.0001, 0.0001, 0.0001);
	crank->SetInertia(ChMatrix33<>(crank_inertia));
	mphysicalSystem.Add(crank);

	// Create connecting rod body
	auto connecting_rod = std::make_shared<ChBody>();
	connecting_rod->SetPos(ChVector<>(0.1385, 0.07, 0.01));
	connecting_rod->SetPos_dt(ChVector<>(0, 0, 0));
	ChVector<>connecting_rod_inertia(0.004, 0.004, 0.004);
	connecting_rod->SetInertia(ChMatrix33<>(connecting_rod_inertia));

	// Set connecting rod initial rotation
	ChQuaternion<> qC = QUNIT;
	ChQuaternion<> qA = Q_from_AngAxis(3.8 * CH_C_DEG_TO_RAD, VECT_Y);
	ChQuaternion<> qB = Q_from_AngAxis(70 * CH_C_DEG_TO_RAD, VECT_Z);
	qC = qA * qB;
	connecting_rod->SetRot(qC);
	mphysicalSystem.Add(connecting_rod);

	// Constraints:
	//auto hingeLink =
	//	std::make_shared<ChLinkMateGeneric>(true, true, true, false, true, true);  // x,y,z,Rx,Ry,Rz constrains
	//hingeLink->Initialize(crank,        // the 1st body to connect
	//	floorBody,           // the 2nd body to connect
	//	false,               // the two following frames are in absolute, not relative, coords.
	//	link_position_abs,   // the link reference attached to 1st body
	//	link_position_abs);  // the link reference attached to 2nd body
	//mphysicalSystem.Add(hingeLink);

	ChVector<> poscrank(0, 0.12, 0.1);
	auto link_engine = std::make_shared<ChLinkEngine>();
	link_engine->Initialize(floorBody, crank, ChCoordsys<>(poscrank, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
	link_engine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
	link_engine->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	link_engine->Set_tor_funct(std::make_shared<ChFunction_Const>(0.5));
	mphysicalSystem.Add(link_engine);

	auto sphericalLink =
		std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);
	ChFrame<> link2_position_abs(ChVector<>(0, 0.12, 0.02));

	sphericalLink->Initialize(crank,connecting_rod,false,link2_position_abs,link2_position_abs);
	mphysicalSystem.Add(sphericalLink);

	auto universalLink =
		std::make_shared<ChLinkMateGeneric>(true, true, true, false, true, false);
	ChFrame<> link3_position_abs(ChVector<>(0.277, 0.025, 0));
	universalLink->Initialize(slider,connecting_rod,false,link3_position_abs,link3_position_abs);
	mphysicalSystem.Add(universalLink);

	auto prismaticLink =
		std::make_shared<ChLinkMateGeneric>(false, true, true, true, true, true);
	ChFrame<> link4_position_abs(ChVector<>(0.277, 0.012, 0));
	prismaticLink->Initialize(slider,floorBody,false,link4_position_abs,link4_position_abs);
	mphysicalSystem.Add(prismaticLink);

	auto my_marker_a = std::make_shared<ChMarker>();
	slider->AddMarker(my_marker_a);

#pragma region wrap

	// Choose the solver used, this menu is simply used to limit and display solvers
	int solver_choice = 7;
	switch (solver_choice) {
	case 0:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //SOR
		break;
	case 1:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //SYMMSOR
		break;
	case 3:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //SOR_MULTI
		break;
	case 5:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //BARZILAIBORWEIN
		break;
	case 7:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //APGD
		break;
	case 9:
		mphysicalSystem.SetSolverType(ChSolver::Type(solver_choice)); //SOLVER_DEM
		break;
	}

	// Choose a timestepper
	char* timestepping;
	int time_stepper = 1;
	switch (time_stepper) {
	case 0:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "EULER_IMPLICIT_LINEARIZED";
		break;
	case 1:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "EULER_IMPLICIT_PROJECTED";
		break;
	case 2:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "EULER_IMPLICIT";
		break;
	case 3:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "TRAPEZOIDAL";
		break;
	case 4:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "TRAPEZOIDAL_LINEARIZED";
		break;
	case 5:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "HHT";
		break;
	case 6:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "HEUN";
		break;
	case 7:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "RUNGEKUTTA45";
		break;
	case 8:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "EULER_EXPLICIT";
		break;	
	case 20:
		mphysicalSystem.SetTimestepperType(ChTimestepper::Type(time_stepper));
		timestepping = "Custom";
		break;
	default:
		timestepping = "Undefined";
		break;
	}

	ofstream simtime_data;
	simtime_data.open("Test results/APGD/Timing/Euler_implicit_projected/10ms_time_data.txt"); // filename has to be changed as different benchmarks are performed
	if (!simtime_data) {
		cerr << "Failed to open file. Exiting out...";
		exit(0); //failed to open file
	}

	simtime_data << "Timestepper used: " << timestepping << "\nStepsize: " << TIMESTEP << endl;
	simtime_data << "\nIteration:" << "\t" << "Simstep solve time (microseconds):" << endl;

	ofstream velocity_data;
	velocity_data.open("Test results/APGD/Velocity/Euler_implicit_projected/10ms_vel_data.txt"); // filename has to be changed as different benchmarks are performed
	if (!velocity_data) {
		cerr << "Failed to open file. Exiting out...";
		exit(0);
	}

	velocity_data << "Timestepper used: " << timestepping << endl;
	velocity_data << "\nTime in sim:" << "\t" << "Slider vel:" << endl;

	double stepCount = 0, totalTime = 0, totalSimtime = 0;
	// This loop performs dynamics analysis of an arbitrary simulated time
	while (stepCount < 200) { // the number has to be changed to accordingly ( number * TIMESTEP = simulated time)
		timer.start();
		mphysicalSystem.DoStepDynamics(TIMESTEP);
		timer.stop();
		unsigned long long time = timer.GetTimeMicroseconds();
		totalSimtime += time;
		simtime_data << stepCount << "\t\t" << time << "\n";
		velocity_data << totalTime << "\t\t" << my_marker_a->GetAbsCoord_dt().pos.x() << "\n";
		timer.reset();
		totalTime += TIMESTEP;
		stepCount++;
	}
	simtime_data << "\n" << "Total simtime:" << "\t" << totalSimtime << "\n";
	simtime_data.close();
	velocity_data.close();

	return 0;
}
#endif