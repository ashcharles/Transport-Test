#include "rapiwebsim.h"

using namespace Rapi;
using namespace rapiwebsim;

//----------------------------------------------------------------------------
RapiWebSim::RapiWebSim( const std::string& host, const unsigned short port ) :
    websim::WebSim( host, port )
{
  std::cout << "RapiWebSim object created" << std::endl;
}
//----------------------------------------------------------------------------
RapiWebSim::~RapiWebSim()
{
  std::cout << "RapiWebSim object destroyed" << std::endl;
}
//----------------------------------------------------------------------------
bool RapiWebSim::ClockStart()
{
  PRT_ERR0( "Clock Start: method not implementable\n" );
  return false;
}
//----------------------------------------------------------------------------
bool RapiWebSim::ClockStop()
{
  PRT_ERR0( "Clock Stop: method not implementable\n" );
  return false;
}
//----------------------------------------------------------------------------
bool RapiWebSim::ClockRunFor( double seconds )
{
  PRT_ERR1( "ClockRunFor %3.2f [s]: method not implementable\n", seconds );
  return false;
}
//----------------------------------------------------------------------------
bool RapiWebSim::CreateModel( const std::string& name,
                              const std::string& type,
                              std::string& response )
{
//  mRobot = new CCBRobot();
//  if( mRobot->init() == 0 ) {
//    response = rapiError->print();
//    delete mRobot;
//    return false;
//  }
//  mRobotCtrl = new CChatterboxCtrl( mRobot );
  response = str( boost::format( "Created model with name %s of type %s\n" )
                  % name % type );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::DeleteModel( const std::string& name,
                              std::string& response )
{
  response = str( boost::format( "Deleted model with name %s\n" ) % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelType( const std::string& name,
                               std::string& type )
{
  type = str( boost::format( "Got type of model with name %s\n" ) % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelData( const std::string& name,
                               std::string& response,
                               websim::Format format = websim::TEXT,
							   void * xmlnode = NULL )
{
  response = str( boost::format( "Got data of model with name %s\n" ) % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::SetModelPVA( const std::string& name,
                              const websim::Pose& p,
                              const websim::Velocity& v,
                              const websim::Acceleration& a,
                              std::string& response )
{
  response = str( boost::format( "Set P%s, V%s & A%s of model %s\n" )
                  % p.String() % v.String() % a.String() % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelPVA( const std::string& name,
                              websim::Time& t,
                              websim::Pose& p,
                              websim::Velocity& v,
                              websim::Acceleration& a,
                              std::string& response )
{
  response = str( boost::format( "Get PVA of model %s\n" ) % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelGeometry( const std::string& name,
                                   double& bx,
                                   double& by,
                                   double& bz,
                                   websim::Pose& center,
                                   std::string& response )
{
  response = str( boost::format( "Get geometry of model %s\n" ) % name );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelTree( const std::string& model,
                               websim::Format format,
                               std::string& response )
{
  response = str( boost::format( "Get tree of model %s\n" ) % model );
  return true;
}
//----------------------------------------------------------------------------
bool RapiWebSim::GetModelChildren( const std::string& model,
                                   std::vector<std::string>& children )
{
  std::cout << "Get model children" << std::endl;
  return true;
}
//----------------------------------------------------------------------------
websim::Time RapiWebSim::GetTime()
{
  websim::Time t;
  return t;
}
//----------------------------------------------------------------------------
int main( int argc, char** argv )
{
  std::string host;
  unsigned short port;
  if( argc > 2) {
    host = argv[1];
    port = atoi( argv[2] );
  }
  else {
    host = "localhost"; 
    port = 8000;
  }
  
  RapiWebSim rws( host, port );
  rws.ClockStart();
  rws.ClockStop();
  rws.ClockRunFor( 10.0 );
  
  std::string name = "RapiRobot";
  std::string type = "Chatterbox";
  std::string response;
  rws.CreateModel( name, type, response );
  std::cout << response << std::endl;
  rws.DeleteModel( name, response );
  std::cout << response << std::endl;
  rws.GetModelType( name, response );
  std::cout << response << std::endl;
  rws.GetModelData( name, response, websim::TEXT, NULL );
  std::cout << response << std::endl;

  websim::Pose p( 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 );
  websim::Velocity v( -1.0, -2.0, -3.0 );
  websim::Acceleration a;
  websim::Time t( 10, 231 );
  rws.SetModelPVA( name, p, v, a, response );
  std::cout << response << std::endl;
  rws.GetModelPVA( name, t, p, v, a, response );
  std::cout << response << std::endl;
  double bx = 10.0;
  double by = 20.0;
  double bz = 30.0;
  rws.GetModelGeometry( name, bx, by, bz, p, response );
  std::cout << response << std::endl;
  rws.GetModelTree( name, websim::TEXT, response );
  std::cout << response << std::endl;

  std::vector<std::string> children;
  children.push_back("First Child");
  rws.GetModelChildren( name, children );
  std::cout << children[0] << std::endl;
  rws.GetTime();
  rws.Startup( true );
  while (1) {
    rws.Go();
    rws.Wait();
  }
} // end main()

