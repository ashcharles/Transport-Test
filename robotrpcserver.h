#ifndef ROBOTRPCSERVER_H
#define ROBOTRPCSERVER_H

#include "jsonrpc_server.h"
#include "RapiCore"

using namespace Rapi;
/**
  @author Ash Charles <jac27@sfu.ca>
*/
class RobotRpcServer
{
  public:
    RobotRpcServer ( ARobot * robot, int port );
    ~RobotRpcServer();
    void update( void );
    void getRanges( jsonrpc::variant params,
                    jsonrpc::object& results,
                    const std::string& ip,
                    int port );

  private:
    jsonrpc::TCPServer mServer;
    ARobot * mRobot;
    ADrivetrain2dof * mDrivetrain;
    APowerPack * mPowerPack;
    ARangeFinder * mRangeFinder;
    ALights * mLights;
};

#endif
