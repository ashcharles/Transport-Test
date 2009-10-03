#include "robotrpcserver.h"

//------------------------------------------------------------------------------
RobotRpcServer::RobotRpcServer ( ARobot * robot, int port ) : mServer( port )
{
  // try to get some basic devices
  mRobot = robot;
  mRobot->findDevice ( mDrivetrain, "drivetrain:0" );
  mRobot->findDevice ( mPowerPack, "powerpack:0" );
  mRobot->findDevice ( mRangeFinder, "ranger:0" );
  mRobot->findDevice ( mLights, "lights:0" );

  // setup handlers as appropriate
  if( mRangeFinder )
    mServer.addMethodHandler( new jsonrpc::Server::RPCMethod< RobotRpcServer >
                              (this, &RobotRpcServer::getRanges ), "getRanges" );
  // TODO: add other methods here

}
//------------------------------------------------------------------------------
RobotRpcServer::~RobotRpcServer()
{
}
//------------------------------------------------------------------------------
void RobotRpcServer::update()
{
  while( mServer.recv() );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getRanges( jsonrpc::variant params,
                                jsonrpc::object& results,
                                const std::string& ip,
                                int port )
{
  std::cout << "RobotRpcServer::getRanges()" << std::endl;
  //std::vector<float> data;
  //for( unsigned int i = 0; i < mRangeFinder->getNumSamples(); ++i )
//    data.push_back( mRangeFinder->mRangeData[i].range );
  results[ "range" ] = jsonrpc::toVariant( 12 );
}
//------------------------------------------------------------------------------
