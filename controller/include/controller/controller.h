#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>

//! Implements online MPC
/*!
   Controller implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class Controller {
  public:
	/**
	 * @brief Constructor for Controller
	 * @return Constructed object of type Controller
	 */
	Controller();

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void update();

};


#endif // CONTROLLER_H
