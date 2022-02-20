// ----------------------------------------------------------------------------
// MouseReachLinearController.h
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef MOUSE_REACH_LINEAR_CONTROLLER_H
#define MOUSE_REACH_LINEAR_CONTROLLER_H
#include <ArduinoJson.h>
#include <JsonStream.h>
#include <Array.h>
#include <Vector.h>
#include <ConstantVariable.h>
#include <Functor.h>

#include <EventController.h>

#include <AudioApparatus.h>

#include <ModularServer.h>
#include <ModularDeviceBase.h>
#include <StepDirController.h>
#include <StepperController.h>
#include <StageController.h>

#include "MouseReachLinearController/Constants.h"


class MouseReachLinearController : public StageController
{
public:
  virtual void setup();
  virtual void update();

  mouse_reach_linear_controller::constants::AssayStatus getAssayStatus();
  StageController::PositionArray getBuzzPosition();
  StageController::PositionArray getLoadPosition();
  StageController::PositionArray getNextDeliverPosition();
  StageController::PositionArray getNextDispensePosition();
  long getPositionToneFrequency();
  long getPositionToneVolume();
  long getPositionToneDelay();
  long getPositionToneDuration();
  long getDispenseDelay();
  long getReturnDelay();
  long getBuzzPeriod();
  long getBuzzOnDuration();
  long getBuzzCount();
  long getWaitAtLoadDuration();
  long getTapPeriod();
  long getTapOnDuration();
  long getTapCount();

  void moveStageToBuzzPosition();
  void moveStageToLoadPosition();
  void waitAtLoad();
  void moveStageToNextDeliverPosition();
  void setDispenseVelocityLimit(size_t channel);
  void setDispenseVelocityLimits();
  void restoreVelocityLimits();
  void moveStageToNextDispensePosition();
  void playPositionTone();
  void setWaitToDispenseState();
  void waitToDispense();
  void setMoveToDispenseState();
  void waitToReturn();
  void setMoveToBuzzState();
  void setMoveToLoadState();
  void setWaitAtLoadState();
  void setMoveToNextDeliverPositionState();
  void buzz();
  void tap();

  void startAssay();
  void dispense();
  void abort();

private:
  modular_server::Pin pins_[mouse_reach_linear_controller::constants::PIN_COUNT_MAX];

  modular_server::Property properties_[mouse_reach_linear_controller::constants::PROPERTY_COUNT_MAX];
  modular_server::Parameter parameters_[mouse_reach_linear_controller::constants::PARAMETER_COUNT_MAX];
  modular_server::Function functions_[mouse_reach_linear_controller::constants::FUNCTION_COUNT_MAX];
  modular_server::Callback callbacks_[mouse_reach_linear_controller::constants::CALLBACK_COUNT_MAX];

  mouse_reach_linear_controller::constants::AssayStatus assay_status_;
  EventController<mouse_reach_linear_controller::constants::EVENT_COUNT_MAX> event_controller_;

  AudioApparatus<mouse_reach_linear_controller::constants::EVENT_COUNT_MAX> audio_apparatus_;

  StageController::PositionArray deliver_position_;

  // Handlers
  void setClientPropertyValuesHandler();
  void setDispenseVelocityLimitHandler(size_t channel);
  void getAssayStatusHandler();
  void playPositionToneHandler();
  void buzzHandler();
  void moveToDispenseHandler(int arg);
  void moveToBuzzHandler(int arg);
  void setMoveToLoadHandler(int arg);
  void setWaitAtLoadHandler(int arg);
  void setMoveToNextDeliverPositionHandler(int arg);
  void startAssayHandler(modular_server::Pin * pin_ptr);
  void dispenseHandler(modular_server::Pin * pin_ptr);
  void abortHandler(modular_server::Pin * pin_ptr);

};

#endif
