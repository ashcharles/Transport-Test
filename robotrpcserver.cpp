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
  if( mRangeFinder ) {
    mServer.addMethodHandler( new jsonrpc::Server::RPCMethod< RobotRpcServer >
                              (this, &RobotRpcServer::getRangeFinder ),
                              "getRangeFinder" );
    mServer.addMethodHandler( new jsonrpc::Server::RPCMethod< RobotRpcServer >
                              (this, &RobotRpcServer::getRanges ), "getRanges" );
  }
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
  using namespace jsonrpc;
  array ranges;
  for( unsigned int i = 0; i < mRangeFinder->getNumSamples(); ++i ) {
    ranges.push_back( toVariant<double>( mRangeFinder->mRangeData[i].range ) );
  }
  results[ "range" ] = toVariant( ranges );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getRangeFinder( jsonrpc::variant params,
                                jsonrpc::object& results,
                                const std::string& ip,
                                int port )
{
  using namespace jsonrpc;

  results[ "numSamples" ] = toVariant<int>( mRangeFinder->getNumSamples() );
  double minRange = (mRangeFinder->getMinRange() == INFINITY ) ? 0.0 :
                      mRangeFinder->getMinRange();
  results[ "minRange" ] = toVariant<double>( minRange );
  results[ "maxRange" ] = toVariant<double>( mRangeFinder->getMaxRange() );
  results[ "beamConeAngle" ] = toVariant<double>( mRangeFinder->getBeamConeAngle() );

  array beamPose;
  for( unsigned int i = 0; i < mRangeFinder->getNumSamples(); ++i ) {
    Rapi::CPose2d pose( mRangeFinder->mRelativeBeamPose[i] );
    object poseObj;
    poseObj[ "x" ] = toVariant<double>( pose.mX );
    poseObj[ "y" ] = toVariant<double>( pose.mY );
    poseObj[ "yaw" ] = toVariant<double>( pose.mYaw );
    beamPose.push_back( toVariant<object>( poseObj ) );
  }
  results[ "beamPose" ] = toVariant<array>( beamPose );
}
//------------------------------------------------------------------------------
