#include "robotrpcserver.h"

using namespace jsonrpc;

//------------------------------------------------------------------------------
RobotRpcServer::RobotRpcServer ( Rapi::ARobot * robot, int port )
  : mServer ( port )
{
  // try to get some basic devices
  mRobot = robot;
  mRobot->findDevice ( mDrivetrain, "drivetrain:0" );
  mRobot->findDevice ( mPowerPack, "powerpack:0" );
  mRobot->findDevice ( mRangeFinder, "ranger:0" );

  // setup handlers as appropriate
  if ( mDrivetrain )
  {
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getDrivetrainDev ),
                               "getDrivetrainDev" );
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getDrivetrain ),
                               "getDrivetrain" );
  }
  if ( mPowerPack )
  {
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getPowerpackDev ),
                               "getPowerpackDev" );
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getPowerpack ),
                               "getPowerpack" );
  }
  if ( mRangeFinder )
  {
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getRangeFinderDev ),
                               "getRangeFinderDev" );
    mServer.addMethodHandler ( new Server::RPCMethod< RobotRpcServer >
                               ( this, &RobotRpcServer::getRanges ),
                               "getRanges" );
  }
}
//------------------------------------------------------------------------------
RobotRpcServer::~RobotRpcServer()
{
}
//------------------------------------------------------------------------------
void RobotRpcServer::update()
{
  while ( mServer.recv() );
}
//------------------------------------------------------------------------------
variant packVelocity ( Rapi::CVelocity2d velocity )
{
  object velocityObj;
  velocityObj[ "xDot" ] = toVariant<double> ( velocity.mXDot );
  velocityObj[ "yDot" ] = toVariant<double> ( velocity.mYDot );
  velocityObj[ "yawDot" ] = toVariant<double> ( velocity.mYawDot );
  return toVariant ( velocityObj );
}
//------------------------------------------------------------------------------
variant packPose ( Rapi::CPose2d pose )
{
  object poseObj;
  poseObj[ "x" ] = toVariant<double> ( pose.mX );
  poseObj[ "y" ] = toVariant<double> ( pose.mY );
  poseObj[ "yaw" ] = toVariant<double> ( pose.mYaw );
  return toVariant ( poseObj );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getDrivetrainDev ( variant params,
                                        object& results,
                                        const std::string& ip,
                                        int port )
{
  std::cout << "this method isn't needed" << std::endl;
}
//------------------------------------------------------------------------------
void RobotRpcServer::getPowerpackDev ( variant params,
                                       object& results,
                                       const std::string& ip,
                                       int port )
{
  std::cout << "this method isn't needed" << std::endl;
}
//------------------------------------------------------------------------------
void RobotRpcServer::getRangeFinderDev ( variant params,
        object& results,
        const std::string& ip,
        int port )
{
  results[ "numSamples" ] = toVariant<int> ( mRangeFinder->getNumSamples() );
  double minRange = ( mRangeFinder->getMinRange() == INFINITY ) ? 0.0 :
                    mRangeFinder->getMinRange();
  results[ "minRange" ] = toVariant<double> ( minRange );
  results[ "maxRange" ] = toVariant<double> ( mRangeFinder->getMaxRange() );
  results[ "beamConeAngle" ] = toVariant<double> ( mRangeFinder->getBeamConeAngle() );

  array beamPose;
  for ( unsigned int i = 0; i < mRangeFinder->getNumSamples(); ++i )
  {
    beamPose.push_back ( packPose ( mRangeFinder->mRelativeBeamPose[i] ) );
  }
  results[ "beamPose" ] = toVariant<array> ( beamPose );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getDrivetrain ( variant params,
                                     object& results,
                                     const std::string& ip,
                                     int port )
{
  results[ "isStalled" ] = toVariant<bool> ( mDrivetrain->isStalled() );
  results[ "stalledSince" ] = toVariant<double> ( mDrivetrain->stalledSince() );
  results[ "measVelocity" ] = packVelocity ( mDrivetrain->getVelocity() );
  results[ "cmdVelocity" ] = packVelocity ( mDrivetrain->getVelocityCmd() );
  results[ "odometry" ] = packPose ( mDrivetrain->getOdometry()->getPose() );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getPowerpack ( variant params,
                                    object& results,
                                    const std::string& ip,
                                    int port )
{
  results[ "voltage" ] = toVariant<double> ( mPowerPack->getVoltage() );
  results[ "remainingEnergy" ] = toVariant<double> ( mPowerPack->getBatteryLevel() );
  results[ "isCharging" ] = toVariant<bool> ( mPowerPack->isCharging() );
}
//------------------------------------------------------------------------------
void RobotRpcServer::getRanges ( variant params,
                                 object& results,
                                 const std::string& ip,
                                 int port )
{
  array ranges;
  for ( unsigned int i = 0; i < mRangeFinder->getNumSamples(); ++i )
  {
    ranges.push_back ( toVariant<double> ( mRangeFinder->mRangeData[i].range ) );
  }
  results[ "range" ] = toVariant ( ranges );
}
//------------------------------------------------------------------------------
