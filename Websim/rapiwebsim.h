#ifndef RAPIWEBSIM_H
#define RAPIWEBSIM_h

#include <iostream>
#include <RapiStage>
#include <boost/format.hpp>
#include "websim.hh"

namespace rapiwebsim {

class RapiWebSim : public websim::WebSim
{
public:
  /**
   * Default constructor.
   * @param hostname
   * @param port_number
   */
  RapiWebSim( const std::string& host, const unsigned short port);
  /**
   * Default destructor.
   */
  ~RapiWebSim();
  /**
   * Get the simulator name.
   * @returns simulator_name
   */
  inline std::string IdentificationString() {return "RapiWebSim";}
  /**
   * Get the version of the simulator.
   * @returns simulator_version
   */
  inline std::string VersionString() {return "0.0.1"; }
  /**
   * Starts the clock.
   * @return TRUE if successful
   */
  bool ClockStart();
  /**
   * Stops the clock.
   * @return TRUE if successful
   */
  bool ClockStop();
  /**
   * Runs the simulator clock.
   * @param seconds
   * @return TRUE if successful
   */
  bool ClockRunFor( double seconds );
  /**
   * Create a named model of the specified type.
   * @param model_name
   * @param model_type
   * @param response
   * @return TRUE if successful
   */
  bool CreateModel( const std::string& name,
                    const std::string& type,
                    std::string& response );
  /**
   * Delete a named model
   * @param model_name
   * @param response
   * @return TRUE if successful
   */
  bool DeleteModel( const std::string& name, std::string& response );
  /**
   * Gets the type of a named model.
   * @param model_name
   * @param model_type
   * @returns TRUE if successful
   */
  bool GetModelType( const std::string& name, std::string& type );
  /**
   * Gets the data of a named model.
   * @param model_name
   * @param response data 
   * @returns TRUE if successful
   */
  bool GetModelData( const std::string& name,
                     std::string& response,
                     websim::Format format,
                     void * xmlnode );
  /**
   * Set the pose, velocity and acceleration of a named model.
   * @param model_name
   * @param pose
   * @param velocity
   * @param acceleration
   * @return TRUE if successful
   */
  bool SetModelPVA( const std::string& name,
                    const websim::Pose& p,
                    const websim::Velocity& v,
                    const websim::Acceleration& a,
                    std::string& response );
  /**
   * Get the pose, velocity and acceleration of a named model.
   * @param model_name
   * @param t the time of the simulation returned by model
   * @param position
   * @param velocity
   * @param acceleration
   * @return TRUE if successful
   */
  bool GetModelPVA( const std::string& name,
                    websim::Time& t,
                    websim::Pose& p,
                    websim::Velocity& v,
                    websim::Acceleration& a,
                    std::string& response );
  /**
   * Get the extent and center of the model.
   * @param bc length
   * @param by width
   * @param bz height
   * @param center the center of the model
   * @returns TRUE if successful
   */
  bool GetModelGeometry( const std::string& name,
                         double& bx,
                         double& by,
                         double& bz,
                         websim::Pose& center,
                         std::string& response );
  /**
   * Gets the tree of the model.  If no model is specified, the method will
   * yield the tree of the world.
   * @param name the name of the model
   * @param response the output string contraining the tree in text|xml format
   * @param format specifies the format of the reponse (text|xml)
   * @return TRUE if successful
   */
  bool GetModelTree( const std::string& model,
                     websim::Format format,
                     std::string & response );
  /**
   * Gets the children of the model. 
   * @param name the name of the model
   * @param children the names of the children
   * @return TRUE if successful
   */
  bool GetModelChildren( const std::string& model,
                         std::vector<std::string>& children );
  /**
   * Get the current simulation time.
   * @return time time of simulation
   */
  websim::Time GetTime();
private:
  //Rapi::CCBRobot* mRobot;
  //CChatterboxCtrl* mRobotCtrl;
  //std::map<Rapi::CCBRobot*, CChatterboxCtrl*> mRobots;
};
} // namespace rapiwebsim
#endif //RAPIWEBSIM_H
