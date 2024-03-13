//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include "legged_controllers/utilities.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/robot_type", robot_name_);
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      leggedInterface_->getPinocchioInterface(), pinocchioMapping, leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());
  robotVisualizer_->setRobotName(robot_name_);
  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();

  std::vector<std::string> joint_names;
  loadData::loadStdVector(taskFile, "joint_names", joint_names, verbose);

  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_link");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                       leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);
  wbc_->setStanceMode(true);
  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  // Configuring the hardware interface
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

  // Reading relevant parameters
  RetrievingParameters();
  
  // loadEigenMatrix
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  startingTime_.fromSec(time.toSec() - 0.0001);
  const ros::Time shifted_time = time - startingTime_;
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

  mpcRunning_ = false;

  // Mode Subscribe
  ModeSubscribe();

  // Dynamic server
  serverPtr_ =
      std::make_unique<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>>(ros::NodeHandle("controller"));
  dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>::CallbackType f;
  f = boost::bind(&LeggedController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(f);
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  loopTimer_.startTimer();cpTimer_.startTimer();
  //std::cout<<"period:"<<period.toSec()<<std::endl;
  const ros::Time shifted_time = time - startingTime_;
  // State Estimate
  updateStateEstimation(shifted_time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);    //[质心动量、角动量、浮动基位置、姿态、关节角]  6+6+nj维

  // Evaluate the current policy
  vector_t optimizedState(stateDim_), optimizedInput(inputDim_);  //状态包括浮动基速度，浮动基位置，关节位置，输入包括关节速度和接触力

  size_t plannedMode = 0;   //0代表悬空，1、2：ssp，3：dsp，目前依赖于mpcmrtinterface规划
  bool mpc_updated_ = false;
  if (firstStartMpc_)   //MPC启动
  {
    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                     optimizedInput, plannedMode);
    currentObservation_.input = optimizedInput;
    mpc_updated_ = true;
  }
 // std::cout<<"optimizedState:"<<optimizedState.tail(jointDim_+6).transpose()<<std::endl;
 // std::cout<<"optimizedInput:"<<optimizedInput.transpose()<<std::endl;
  if (setWalkFlag_)
  {
    wbc_->setStanceMode(false);
  }
  else
  {
    optimizedState.setZero();
    optimizedInput.setZero();
    optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6);
    optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;
    plannedMode = 3;
    wbc_->setStanceMode(true);
  }
  
  if(robot_name_=="bipedv5"){ //平移关节的期望角度设成一致
    double lp = (optimizedState(14)+optimizedState(16))/2.0;
    double rp = (optimizedState(20)+optimizedState(22))/2.0;
    std::cout<<"MPC, lp 1:"<<optimizedState(14)<<", lp 2:"<<optimizedState(16)<<", lp:"<<lp<<std::endl;
   std::cout<<"MPC, rp 1:"<<optimizedState(20)<<", rp 2:"<<optimizedState(22)<<", rp:"<<rp<<std::endl;
    optimizedState(14) = lp;
    optimizedState(16) = lp;
    optimizedState(20) = rp;
    optimizedState(22) = rp;
    double lpv = (optimizedInput(14)+optimizedInput(16))/2.0;
    double rpv = (optimizedInput(20)+optimizedInput(22))/2.0;
   std::cout<<"MPC, lpv 1:"<<optimizedInput(14)<<", lpv 2:"<<optimizedInput(16)<<", lpv:"<<lpv<<std::endl;
   std::cout<<"MPC, rpv 1:"<<optimizedInput(20)<<", rpv 2:"<<optimizedInput(22)<<", rpv:"<<rpv<<std::endl;
    optimizedInput(14) = lpv;
    optimizedInput(16) = lpv;
    optimizedInput(20) = rpv;
    optimizedInput(22) = rpv;
  }


  const vector_t& mpc_planned_body_pos = optimizedState.segment(6, 6);
  const vector_t& mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
  const vector_t& mpc_planned_joint_vel = optimizedInput.segment(12, jointDim_);

  // WBC
  wbcTimer_.startTimer();
  //输入mpc期望轨迹和当前实际状态更新wbc
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  const vector_t& wbc_planned_torque = x.tail(jointDim_);     //wbc期望力矩
  //std::cout<<"wbc_planned_torque:"<<wbc_planned_torque.head(6).transpose()<<std::endl;
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointDim_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointDim_, wbc_->getContactForceSize());
  wbcTimer_.endTimer();

  posDes_ = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  velDes_ = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  scalar_t dt = period.toSec();
  posDes_ = posDes_ + 0.5 * wbc_planned_joint_acc * dt * dt;
  velDes_ = velDes_ + wbc_planned_joint_acc * dt;

  vector_t output_torque(jointDim_);
  //*********************** Set Joint Command: Normal Tracking *****************************//
  for (size_t j = 0; j < jointDim_; ++j)
  {
    //"Limit protection
    const auto& model = leggedInterface_->getPinocchioInterface().getModel();
    double lower_bound = model.lowerPositionLimit(6 + j);
    double upper_bound = model.upperPositionLimit(6 + j);
    if (!emergencyStopFlag_ && loadControllerFlag_ &&
        (hybridJointHandles_[j].getPosition() > upper_bound + 0.02 ||
         hybridJointHandles_[j].getPosition() < lower_bound - 0.02))      //关节位置接近极限时急停
    {
      emergencyStopFlag_ = true;
      std::cerr << "Reach Position Limit!!!!!!!!!!!!!!!!!!!!!!!! " << j << ":" << hybridJointHandles_[j].getPosition()
                << std::endl;
    }
    if (!loadControllerFlag_)   //没加载控制器时设置为默认关节位置
    {
      if(robot_name_=="hunter" || robot_name_ =="mit_humanoid"){
        if (j == 4 || j == 9)   //脚踝两个关节
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
        else
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position, 0);
      }else if(robot_name_ == "bipedv5"){
        if (j == 5 || j == 11)   //脚踝两个关节
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
        else if(j == 2|| j == 4 || j == 8 || j == 10) //平移关节
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_pri, kd_pri, 0);
        else
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position, 0);
      }else if(robot_name_ == "bipedv5_sim"){
        if (j == 4 || j == 9)   //脚踝两个关节
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
        else if(j == 3|| j == 8 ) //平移关节
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_pri, kd_pri, 0);
        else
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position, 0);
      }
    }
    else
    {
      contact_flag_t cmdContactFlag = modeNumber2StanceLeg(mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
      if(robot_name_=="hunter" || robot_name_ =="mit_humanoid"){
        if (j == 0 || j == 1 || j == 5 || j == 6) //髋关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 5)] ? kp_small_stance : kp_small_swing, kd_small,wbc_planned_torque(j));
        else if (j == 4 || j == 9)    //脚踝关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 5)] ? kp_small_stance : kp_small_swing, kd_feet,wbc_planned_torque(j));
        else    //其他关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j], cmdContactFlag[int(j / 5)] ? kp_big_stance : kp_big_swing, kd_big,wbc_planned_torque(j));
      }else if(robot_name_=="bipedv5"){
        if (j == 0 || j == 1 || j == 6 || j == 7) //髋关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 5)] ? kp_small_stance : kp_small_swing, kd_small,wbc_planned_torque(j));
        else if(j == 2|| j == 4){ //平移关节
         hybridJointHandles_[j].setCommand((posDes_[2]+posDes_[4])/2.0, (velDes_[2]+velDes_[4])/2.0, kp_pri_walk, kd_pri_walk,(wbc_planned_torque(2)+wbc_planned_torque(4))/2.0);
       // hybridJointHandles_[j].setCommand((posDes_[2]+posDes_[4])/2.0, (velDes_[2]+velDes_[4])/2.0, kp_pri_walk, kd_pri_walk,wbc_planned_torque(j));
          std::cout<<"wbc_planned_torque(j):"<<wbc_planned_torque(j)<<std::endl;
      } 
        else if( j == 8 || j == 10)
           hybridJointHandles_[j].setCommand((posDes_[8]+posDes_[10])/2.0, (velDes_[8]+velDes_[10])/2.0, kp_pri_walk, kd_pri_walk,(wbc_planned_torque(8)+wbc_planned_torque(10))/2.0);
         // hybridJointHandles_[j].setCommand((posDes_[8]+posDes_[10])/2.0, (velDes_[8]+velDes_[10])/2.0, kp_pri_walk, kd_pri_walk,wbc_planned_torque(j));
        else if (j == 5 || j == 11)    //脚踝关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_feet,wbc_planned_torque(j));
        else    //其他关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j], cmdContactFlag[int(j / 6)] ? kp_big_stance : kp_big_swing, kd_big,wbc_planned_torque(j));
      }else if(robot_name_=="bipedv5_sim"){
        if (j == 0 || j == 1 || j == 5 || j == 6) //髋关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 5)] ? kp_small_stance : kp_small_swing, kd_small,wbc_planned_torque(j));
        else if( j == 3 || j == 8)
           hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j], kp_pri_walk, kd_pri_walk,wbc_planned_torque(j));
        else if (j == 4 || j == 9)    //脚踝关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],cmdContactFlag[int(j / 5)] ? kp_small_stance : kp_small_swing, kd_feet,wbc_planned_torque(j));
        else    //其他关节
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j], cmdContactFlag[int(j / 5)] ? kp_big_stance : kp_big_swing, kd_big,wbc_planned_torque(j));
      }
    }
    if (emergencyStopFlag_)   //急停
    {
      hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0);
    }
    posDesOutput_(j) = hybridJointHandles_[j].getPositionDesired();
    velDesOutput_(j) = hybridJointHandles_[j].getVelocityDesired();

    output_torque(j) = hybridJointHandles_[j].getFeedforward() +
                       hybridJointHandles_[j].getKp() *
                           (hybridJointHandles_[j].getPositionDesired() - hybridJointHandles_[j].getPosition()) +
                       hybridJointHandles_[j].getKd() *
                           (hybridJointHandles_[j].getVelocityDesired() - hybridJointHandles_[j].getVelocity());
  }
  //*********************** Set Joint Command: Torque Tracking Test *****************************//

  CommandData command_data;
  vector_t planned_state = currentObservation_.state;
  vector_t planned_input = currentObservation_.input;
  planned_state.tail(jointDim_) = posDesOutput_;
  planned_input.tail(jointDim_) = velDesOutput_;
  command_data.mpcTargetTrajectories_.timeTrajectory.push_back(currentObservation_.time);
  command_data.mpcTargetTrajectories_.stateTrajectory.push_back(planned_state);
  command_data.mpcTargetTrajectories_.inputTrajectory.push_back(planned_input);
  command_data = mpc_updated_ ? mpcMrtInterface_->getCommand() : command_data;
  PrimalSolution primal_solution = mpc_updated_ ? mpcMrtInterface_->getPolicy() : PrimalSolution();

  // Visualization TO dO:更改发布tf变换中使用hunter的变量名
  robotVisualizer_->update(currentObservation_, primal_solution, command_data,
                           leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  cpTimer_.endTimer();
  loopTimer_.endTimer();
 // std::cout<<"compute time:"<<cpTimer_.getLastIntervalInMilliseconds()<<" ms , loop total time:"<<loopTimer_.getLastIntervalInMilliseconds()<<" ms , wbc time:"<<wbcTimer_.getLastIntervalInMilliseconds()<<" ms"<<std::endl;
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
      jointTor(hybridJointHandles_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t cmdContactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }
  //支撑腿默认按规划的切换，无接触传感器
  cmdContactFlag = modeNumber2StanceLeg(
      mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
  if (!firstStartMpc_)
  {
    for (size_t i = 0; i < 4; ++i)
    {
      cmdContactFlag[i] = true;
    }
  }
  stateEstimate_->updateCmdContact(cmdContactFlag);
  stateEstimate_->setStartStopTime4Legs(
      leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->threadSaftyGetStartStopTime(
          currentObservation_.time));




  for (size_t i = 0; i < 4; ++i)
  {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  //  std::cout<<"  i "<<i<<", "<<quat.coeffs()(i);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }

  for (size_t i = 0; i < 9; ++i)
  {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

   stateEstimate_->updateJointStates(jointPos, jointVel);
   stateEstimate_->updateContact(cmdContactFlag);    //采用规划接触情况代替真实情况
   stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance,
                             linearAccelCovariance);
   measuredRbdState_ = stateEstimate_->update(time, period);   //[浮动基姿态、位置、关节角度、浮动基角速度、线速度、关节速度] 2*(3+3+nj)维度
  //gt值
   gtState_->updateJointStates(jointPos, jointVel);
  measuredRbdState2_ = gtState_->update(time, period); 
  std::cout<<"state et: "<<measuredRbdState_(5)<<std::endl;
  std::cout<<"state gt: "<<measuredRbdState2_(5)<<std::endl;
  measuredRbdState_.head(5) = measuredRbdState2_.head(5);
  currentObservation_.time = time.toSec();    //[质心动量、角动量、浮动基位置、姿态、关节角]  6+6+nj维
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state.head(stateDim_) = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_); //浮动基速度转化为质心动量

  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  std::cout<<"new yaw:"<<currentObservation_.state(9)<<std::endl;
  currentObservation_.mode = stateEstimate_->getMode();
  currentObservation_.mode = gtState_->getMode();
  const auto& reference_manager = leggedInterface_->getSwitchedModelReferenceManagerPtr();
  reference_manager->getSwingTrajectoryPlanner()->setBodyVelWorld(stateEstimate_->getBodyVelWorld());
  reference_manager->setEstContactFlag(cmdContactFlag);

   stateEstimate_->setCmdTorque(jointTor);
   stateEstimate_->estContactForce(period);
  

}

LeggedController::~LeggedController()
{
  controllerRunning_ = false;
  mpcRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setRobotName(robot_name_);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc()
{
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);    //订阅参考轨迹
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {  //mpc线程
    while (controllerRunning_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpcRunning_)    // /load_controller话题接收消息后开始运行
              {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              //  std::cout<<"mpc time :"<<mpcTimer_.getLastIntervalInMilliseconds()<<" ms"<<std::endl;
                firstStartMpc_ = true;
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
  //订阅话题grount_truth/state
  gtState_ = std::make_shared<FromTopicStateEstimate>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  gtState_->loadSettings(taskFile, verbose);
}

void LeggedController::dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level)
{
  kp_position = config.kp_position;
  kd_position = config.kd_position;
  kp_big_stance = config.kp_big_stance;
  kp_big_swing = config.kp_big_swing;
  kp_small_stance = config.kp_small_stance;
  kp_small_swing = config.kp_small_swing;
  kd_small = config.kd_small;
  kd_big = config.kd_big;
  kd_feet = config.kd_feet;
  kp_pri = config.kp_pri;
  kd_pri = config.kd_pri;
  kp_pri_walk = config.kp_pri_walk;
  kd_pri_walk = config.kd_pri_walk;
}

void LeggedController::RetrievingParameters()
{
  stateDim_ = leggedInterface_->getCentroidalModelInfo().stateDim;
  inputDim_ = leggedInterface_->getCentroidalModelInfo().inputDim;
  jointDim_ = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  footDim_ = leggedInterface_->getCentroidalModelInfo().numThreeDofContacts;
  gencoordDim_ = leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;
  dofPerLeg_ = jointDim_ / 2;
  defalutJointPos_.resize(jointDim_);
}

void LeggedController::resetMPC()
{
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });
  mpcMrtInterface_->resetMpcNode(target_trajectories);
}
void LeggedController::ModeSubscribe()
{
  subSetWalk_ =
      ros::NodeHandle().subscribe<std_msgs::Float32>("/set_walk", 1, &LeggedController::setWalkCallback, this);
  subLoadcontroller_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/load_controller", 1,
                                                                      &LeggedController::loadControllerCallback, this);
  subEmgstop_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/emergency_stop", 1,
                                                               &LeggedController::EmergencyStopCallback, this);
}

void LeggedController::EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg)
{
  emergencyStopFlag_ = true;
  ROS_INFO("Successfully load the controller");
}

void LeggedController::setWalkCallback(const std_msgs::Float32::ConstPtr& msg)
{
  setWalkFlag_ = true;
  ROS_INFO("Set WALK Mode");
}

void LeggedController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;
  ROS_INFO("Successfully load the controller");
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)