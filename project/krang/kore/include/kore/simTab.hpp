/**
 * @file simTab.h
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Provides the tab interface for visualization programs.
 */

#pragma once

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>
#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/AllTabs.h>
#include <GRIPApp.h>
#include <collision/fcl_mesh/CollisionShapes.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <simulation/World.h>
#include <yui/GLFuncs.h>

namespace Krang {

/* ********************************************************************************************* */
/// Timer to display the center of mass measurements and control the robot
class Timer : public wxTimer {
public:
	void Notify ();								
};

/* ********************************************************************************************* */
/// Tab for examples of task constrained planning
class SimTab : public GRIPTab {
public: 

	Timer* timer;
  wxSizer* sizerFull;											///< The sizer in charge of the entire frame

public:
	// Mandatory interface functions

  SimTab(){};									///< Default constructor
  SimTab(wxWindow * parent, wxWindowID id = -1, const wxPoint & pos = wxDefaultPosition,
		const wxSize & size = wxDefaultSize, long style = wxTAB_TRAVERSAL);		
  virtual ~SimTab(){};				///< Destructor
  void OnButton(wxCommandEvent &evt);  		///< Handle button events
  void OnSlider(wxCommandEvent &evt) {}		///< Necessary for compilation (bug!)
  virtual void GRIPEventSimulationBeforeTimestep();  ///< To set joint torques before sim. step
	void GRIPEventRender();

public:
	// wxWidget stuff

  DECLARE_DYNAMIC_CLASS(SimTab)
	DECLARE_EVENT_TABLE()
};

}; // end of namespace
