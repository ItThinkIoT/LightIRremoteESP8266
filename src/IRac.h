#ifndef IRAC_H_
#define IRAC_H_

// Copyright 2019 David Conran

#ifndef UNIT_TEST
#include <Arduino.h>
#else
#include <memory>
#endif
#include "IRremoteESP8266.h"
#include "ir_LG.h"
#include "ir_Rhoss.h"

// Constants
const int8_t kGpioUnused = -1;  ///< A placeholder for not using an actual GPIO.

// Class
/// A universal/common/generic interface for controling supported A/Cs.
class IRac {
 public:
  explicit IRac(const uint16_t pin, const bool inverted = false,
                const bool use_modulation = true);
  static bool isProtocolSupported(const decode_type_t protocol);
  static void initState(stdAc::state_t *state,
                        const decode_type_t vendor, const int16_t model,
                        const bool power, const stdAc::opmode_t mode,
                        const float degrees, const bool celsius,
                        const stdAc::fanspeed_t fan,
                        const stdAc::swingv_t swingv,
                        const stdAc::swingh_t swingh,
                        const bool quiet, const bool turbo, const bool econo,
                        const bool light, const bool filter, const bool clean,
                        const bool beep, const int16_t sleep,
                        const int16_t clock);
  static void initState(stdAc::state_t *state);
  void markAsSent(void);
  bool sendAc(void);
  bool sendAc(const stdAc::state_t desired, const stdAc::state_t *prev = NULL);
  bool sendAc(const decode_type_t vendor, const int16_t model,
              const bool power, const stdAc::opmode_t mode, const float degrees,
              const bool celsius, const stdAc::fanspeed_t fan,
              const stdAc::swingv_t swingv, const stdAc::swingh_t swingh,
              const bool quiet, const bool turbo, const bool econo,
              const bool light, const bool filter, const bool clean,
              const bool beep, const int16_t sleep = -1,
              const int16_t clock = -1);
  static bool cmpStates(const stdAc::state_t a, const stdAc::state_t b);
  static bool strToBool(const char *str, const bool def = false);
  static int16_t strToModel(const char *str, const int16_t def = -1);
  static stdAc::ac_command_t strToCommandType(const char *str,
      const stdAc::ac_command_t def = stdAc::ac_command_t::kControlCommand);
  static stdAc::opmode_t strToOpmode(
      const char *str, const stdAc::opmode_t def = stdAc::opmode_t::kAuto);
  static stdAc::fanspeed_t strToFanspeed(
      const char *str,
      const stdAc::fanspeed_t def = stdAc::fanspeed_t::kAuto);
  static stdAc::swingv_t strToSwingV(
      const char *str, const stdAc::swingv_t def = stdAc::swingv_t::kOff);
  static stdAc::swingh_t strToSwingH(
      const char *str, const stdAc::swingh_t def = stdAc::swingh_t::kOff);
  static String boolToString(const bool value);
  static String commandTypeToString(const stdAc::ac_command_t cmdType);
  static String opmodeToString(const stdAc::opmode_t mode,
                               const bool ha = false);
  static String fanspeedToString(const stdAc::fanspeed_t speed);
  static String swingvToString(const stdAc::swingv_t swingv);
  static String swinghToString(const stdAc::swingh_t swingh);
  stdAc::state_t getState(void);
  stdAc::state_t getStatePrev(void);
  bool hasStateChanged(void);
  stdAc::state_t next;  ///< The state we want the device to be in after we send
#ifdef UNIT_TEST
  /// @cond IGNORE
  /// UT-specific
  /// See @c OUTPUT_DECODE_RESULTS_FOR_UT macro description in IRac.cpp
  std::shared_ptr<IRrecv> _utReceiver = nullptr;
  std::unique_ptr<decode_results> _lastDecodeResults = nullptr;
  /// @endcond
#else

 private:
#endif  // UNIT_TEST
  uint16_t _pin;  ///< The GPIO to use to transmit messages from.
  bool _inverted;  ///< IR LED is lit when GPIO is LOW (true) or HIGH (false)?
  bool _modulation;  ///< Is frequency modulation to be used?
  stdAc::state_t _prev;  ///< The state we expect the device to currently be in.
#if SEND_LG
  void lg(IRLgAc *ac, const lg_ac_remote_model_t model,
          const bool on, const stdAc::opmode_t mode,
          const float degrees, const stdAc::fanspeed_t fan,
          const stdAc::swingv_t swingv, const stdAc::swingv_t swingv_prev,
          const stdAc::swingh_t swingh, const bool light);
#endif  // SEND_LG
#if SEND_RHOSS
  void rhoss(IRRhossAc *ac,
                const bool on, const stdAc::opmode_t mode, const float degrees,
                const stdAc::fanspeed_t fan, const stdAc::swingv_t swing);
#endif  // SEND_RHOSS
static stdAc::state_t cleanState(const stdAc::state_t state);
static stdAc::state_t handleToggles(const stdAc::state_t desired,
                                    const stdAc::state_t *prev = NULL);
};  // IRac class

/// Common functions for use with all A/Cs supported by the IRac class.
namespace IRAcUtils {
  String resultAcToString(const decode_results * const results);
  bool decodeToState(const decode_results *decode, stdAc::state_t *result,
                     const stdAc::state_t *prev = NULL);
}  // namespace IRAcUtils
#endif  // IRAC_H_
