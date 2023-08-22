// Copyright 2019 David Conran

// Provide a universal/standard interface for sending A/C nessages.
// It does not provide complete and maximum granular control but tries
// to offer most common functionality across all supported devices.

#include "IRac.h"
#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <string.h>
#ifndef ARDUINO
#include <string>
#endif
#include <cmath>
#if __cplusplus >= 201103L && defined(_GLIBCXX_USE_C99_MATH_TR1)
    using std::roundf;
#else
    using ::roundf;
#endif
#include "IRsend.h"
#include "IRremoteESP8266.h"
#include "IRtext.h"
#include "IRutils.h"
#include "ir_LG.h"
#include "ir_Rhoss.h"

// On the ESP8266 platform we need to use a special version of string handling
// functions to handle the strings stored in the flash address space.
#ifndef STRCASECMP
#if defined(ESP8266)
#define STRCASECMP(LHS, RHS) \
    strcasecmp_P(LHS, reinterpret_cast<const char*>(RHS))
#else  // ESP8266
#define STRCASECMP(LHS, RHS) strcasecmp(LHS, RHS)
#endif  // ESP8266
#endif  // STRCASECMP

#ifndef UNIT_TEST
#define OUTPUT_DECODE_RESULTS_FOR_UT(ac)
#else
/* NOTE: THIS IS NOT A DOXYGEN COMMENT (would require ENABLE_PREPROCESSING-YES)
/// If compiling for UT *and* a test receiver @c IRrecv is provided via the
/// @c _utReceived param, this injects an "output" gadget @c _lastDecodeResults
/// into the @c IRAc::sendAc method, so that the UT code may parse the "sent"
/// value and drive further assertions
///
/// @note The @c decode_results "returned" is a shallow copy (empty rawbuf),
///       mostly b/c the class does not have a custom/deep copy c-tor
///       and defining it would be an overkill for this purpose
/// @note For future maintainers: If @c IRAc class is ever refactored to use
///       polymorphism (static or dynamic)... this macro should be removed
///       and replaced with proper GMock injection.
*/
#define OUTPUT_DECODE_RESULTS_FOR_UT(ac)                        \
  {                                                             \
    if (_utReceiver) {                                          \
      _lastDecodeResults = nullptr;                             \
      (ac)._irsend.makeDecodeResult();                          \
      if (_utReceiver->decode(&(ac)._irsend.capture)) {         \
        _lastDecodeResults = std::unique_ptr<decode_results>(   \
          new decode_results((ac)._irsend.capture));            \
        _lastDecodeResults->rawbuf = nullptr;                   \
      }                                                         \
    }                                                           \
  }
#endif  // UNIT_TEST

/// Class constructor
/// @param[in] pin Gpio pin to use when transmitting IR messages.
/// @param[in] inverted true, gpio output defaults to high. false, to low.
/// @param[in] use_modulation true means use frequency modulation. false, don't.
IRac::IRac(const uint16_t pin, const bool inverted, const bool use_modulation) {
  _pin = pin;
  _inverted = inverted;
  _modulation = use_modulation;
  this->markAsSent();
}

/// Initialise the given state with the supplied settings.
/// @param[out] state A Ptr to where the settings will be stored.
/// @param[in] vendor The vendor/protocol type.
/// @param[in] model The A/C model if applicable.
/// @param[in] power The power setting.
/// @param[in] mode The operation mode setting.
/// @param[in] degrees The temperature setting in degrees.
/// @param[in] celsius Temperature units. True is Celsius, False is Fahrenheit.
/// @param[in] fan The speed setting for the fan.
/// @param[in] swingv The vertical swing setting.
/// @param[in] swingh The horizontal swing setting.
/// @param[in] quiet Run the device in quiet/silent mode.
/// @param[in] turbo Run the device in turbo/powerful mode.
/// @param[in] econo Run the device in economical mode.
/// @param[in] light Turn on the LED/Display mode.
/// @param[in] filter Turn on the (ion/pollen/etc) filter mode.
/// @param[in] clean Turn on the self-cleaning mode. e.g. Mould, dry filters etc
/// @param[in] beep Enable/Disable beeps when receiving IR messages.
/// @param[in] sleep Nr. of minutes for sleep mode.
///  -1 is Off, >= 0 is on. Some devices it is the nr. of mins to run for.
///  Others it may be the time to enter/exit sleep mode.
///  i.e. Time in Nr. of mins since midnight.
/// @param[in] clock The time in Nr. of mins since midnight. < 0 is ignore.
void IRac::initState(stdAc::state_t *state,
                     const decode_type_t vendor, const int16_t model,
                     const bool power, const stdAc::opmode_t mode,
                     const float degrees, const bool celsius,
                     const stdAc::fanspeed_t fan,
                     const stdAc::swingv_t swingv, const stdAc::swingh_t swingh,
                     const bool quiet, const bool turbo, const bool econo,
                     const bool light, const bool filter, const bool clean,
                     const bool beep, const int16_t sleep,
                     const int16_t clock) {
  state->protocol = vendor;
  state->model = model;
  state->power = power;
  state->mode = mode;
  state->degrees = degrees;
  state->celsius = celsius;
  state->fanspeed = fan;
  state->swingv = swingv;
  state->swingh = swingh;
  state->quiet = quiet;
  state->turbo = turbo;
  state->econo = econo;
  state->light = light;
  state->filter = filter;
  state->clean = clean;
  state->beep = beep;
  state->sleep = sleep;
  state->clock = clock;
}

/// Initialise the given state with the supplied settings.
/// @param[out] state A Ptr to where the settings will be stored.
/// @note Sets all the parameters to reasonable base/automatic defaults.
void IRac::initState(stdAc::state_t *state) {
  stdAc::state_t def;
  *state = def;
}

/// Get the current internal A/C climate state.
/// @return A Ptr to a state containing the current (to be sent) settings.
stdAc::state_t IRac::getState(void) { return next; }

/// Get the previous internal A/C climate state that should have already been
/// sent to the device. i.e. What the A/C unit should already be set to.
/// @return A Ptr to a state containing the previously sent settings.
stdAc::state_t IRac::getStatePrev(void) { return _prev; }

/// Is the given protocol supported by the IRac class?
/// @param[in] protocol The vendor/protocol type.
/// @return true if the protocol is supported by this class, otherwise false.
bool IRac::isProtocolSupported(const decode_type_t protocol) {
  switch (protocol) {
#if SEND_AIRTON
    case decode_type_t::AIRTON:
#endif  // SEND_AIRTON
#if SEND_AIRWELL
    case decode_type_t::AIRWELL:
#endif  // SEND_AIRWELL
#if SEND_AMCOR
    case decode_type_t::AMCOR:
#endif
#if SEND_ARGO
    case decode_type_t::ARGO:
#endif
#if SEND_BOSCH144
    case decode_type_t::BOSCH144:
#endif
#if SEND_CARRIER_AC64
    case decode_type_t::CARRIER_AC64:
#endif  // SEND_CARRIER_AC64
#if SEND_COOLIX
    case decode_type_t::COOLIX:
#endif
#if SEND_CORONA_AC
    case decode_type_t::CORONA_AC:
#endif
#if SEND_DAIKIN
    case decode_type_t::DAIKIN:
#endif
#if SEND_DAIKIN128
    case decode_type_t::DAIKIN128:
#endif
#if SEND_DAIKIN152
    case decode_type_t::DAIKIN152:
#endif
#if SEND_DAIKIN160
    case decode_type_t::DAIKIN160:
#endif
#if SEND_DAIKIN176
    case decode_type_t::DAIKIN176:
#endif
#if SEND_DAIKIN2
    case decode_type_t::DAIKIN2:
#endif
#if SEND_DAIKIN216
    case decode_type_t::DAIKIN216:
#endif
#if SEND_DAIKIN64
    case decode_type_t::DAIKIN64:
#endif
#if SEND_DELONGHI_AC
    case decode_type_t::DELONGHI_AC:
#endif
#if SEND_ECOCLIM
    case decode_type_t::ECOCLIM:
#endif
#if SEND_ELECTRA_AC
    case decode_type_t::ELECTRA_AC:
#endif
#if SEND_FUJITSU_AC
    case decode_type_t::FUJITSU_AC:
#endif
#if SEND_GOODWEATHER
    case decode_type_t::GOODWEATHER:
#endif
#if SEND_GREE
    case decode_type_t::GREE:
#endif
#if SEND_HAIER_AC
    case decode_type_t::HAIER_AC:
#endif
#if SEND_HAIER_AC160
    case decode_type_t::HAIER_AC160:
#endif  // SEND_HAIER_AC160
#if SEND_HAIER_AC176
    case decode_type_t::HAIER_AC176:
#endif  // SEND_HAIER_AC176
#if SEND_HAIER_AC_YRW02
    case decode_type_t::HAIER_AC_YRW02:
#endif
#if SEND_HITACHI_AC
    case decode_type_t::HITACHI_AC:
#endif
#if SEND_HITACHI_AC1
    case decode_type_t::HITACHI_AC1:
#endif
#if SEND_HITACHI_AC264
    case decode_type_t::HITACHI_AC264:
#endif
#if SEND_HITACHI_AC296
    case decode_type_t::HITACHI_AC296:
#endif
#if SEND_HITACHI_AC344
    case decode_type_t::HITACHI_AC344:
#endif
#if SEND_HITACHI_AC424
    case decode_type_t::HITACHI_AC424:
#endif
#if SEND_KELON
    case decode_type_t::KELON:
#endif
#if SEND_KELVINATOR
    case decode_type_t::KELVINATOR:
#endif
#if SEND_LG
    case decode_type_t::LG:
    case decode_type_t::LG2:
#endif
#if SEND_MIDEA
    case decode_type_t::MIDEA:
#endif  // SEND_MIDEA
#if SEND_MIRAGE
    case decode_type_t::MIRAGE:
#endif  // SEND_MIRAGE
#if SEND_MITSUBISHI_AC
    case decode_type_t::MITSUBISHI_AC:
#endif
#if SEND_MITSUBISHI112
    case decode_type_t::MITSUBISHI112:
#endif
#if SEND_MITSUBISHI136
    case decode_type_t::MITSUBISHI136:
#endif
#if SEND_MITSUBISHIHEAVY
    case decode_type_t::MITSUBISHI_HEAVY_88:
    case decode_type_t::MITSUBISHI_HEAVY_152:
#endif
#if SEND_NEOCLIMA
    case decode_type_t::NEOCLIMA:
#endif
#if SEND_PANASONIC_AC
    case decode_type_t::PANASONIC_AC:
#endif
#if SEND_PANASONIC_AC32
    case decode_type_t::PANASONIC_AC32:
#endif
#if SEND_RHOSS
    case decode_type_t::RHOSS:
#endif
#if SEND_SAMSUNG_AC
    case decode_type_t::SAMSUNG_AC:
#endif
#if SEND_SANYO_AC
    case decode_type_t::SANYO_AC:
#endif
#if SEND_SANYO_AC88
    case decode_type_t::SANYO_AC88:
#endif
#if SEND_SHARP_AC
    case decode_type_t::SHARP_AC:
#endif
#if SEND_TCL112AC
    case decode_type_t::TCL112AC:
#endif
#if SEND_TECHNIBEL_AC
    case decode_type_t::TECHNIBEL_AC:
#endif
#if SEND_TECO
    case decode_type_t::TECO:
#endif
#if SEND_TEKNOPOINT
    case decode_type_t::TEKNOPOINT:
#endif  // SEND_TEKNOPOINT
#if SEND_TOSHIBA_AC
    case decode_type_t::TOSHIBA_AC:
#endif
#if SEND_TRANSCOLD
    case decode_type_t::TRANSCOLD:
#endif
#if SEND_TROTEC
    case decode_type_t::TROTEC:
#endif
#if SEND_TROTEC_3550
    case decode_type_t::TROTEC_3550:
#endif  // SEND_TROTEC_3550
#if SEND_TRUMA
    case decode_type_t::TRUMA:
#endif  // SEND_TRUMA
#if SEND_VESTEL_AC
    case decode_type_t::VESTEL_AC:
#endif
#if SEND_VOLTAS
    case decode_type_t::VOLTAS:
#endif
#if SEND_YORK
    case decode_type_t::YORK:
#endif
#if SEND_WHIRLPOOL_AC
    case decode_type_t::WHIRLPOOL_AC:
#endif
      return true;
    default:
      return false;
  }
}

#if SEND_LG
/// Send a LG A/C message with the supplied settings.
/// @param[in, out] ac A Ptr to an IRLgAc object to use.
/// @param[in] model The A/C model to use.
/// @param[in] on The power setting.
/// @param[in] mode The operation mode setting.
/// @param[in] degrees The temperature setting in degrees.
/// @param[in] fan The speed setting for the fan.
/// @param[in] swingv The vertical swing setting.
/// @param[in] swingv_prev The previous vertical swing setting.
/// @param[in] swingh The horizontal swing setting.
/// @param[in] light Turn on the LED/Display mode.
void IRac::lg(IRLgAc *ac, const lg_ac_remote_model_t model,
              const bool on, const stdAc::opmode_t mode,
              const float degrees, const stdAc::fanspeed_t fan,
              const stdAc::swingv_t swingv, const stdAc::swingv_t swingv_prev,
              const stdAc::swingh_t swingh, const bool light) {
  ac->begin();
  ac->setModel(model);
  ac->setPower(on);
  ac->setMode(ac->convertMode(mode));
  ac->setTemp(degrees);
  ac->setFan(ac->convertFan(fan));
  ac->setSwingV(ac->convertSwingV(swingv_prev));
  ac->updateSwingPrev();
  ac->setSwingV(ac->convertSwingV(swingv));
  const uint8_t pos = ac->convertVaneSwingV(swingv);
  for (uint8_t vane = 0; vane < kLgAcSwingVMaxVanes; vane++)
    ac->setVaneSwingV(vane, pos);
  // Toggle the swingv for LG6711A20083V models if we need to.
  // i.e. Off to Not-Off, send a toggle. Not-Off to Off, send a toggle.
  if ((model == lg_ac_remote_model_t::LG6711A20083V) &&
      ((swingv == stdAc::swingv_t::kOff) !=
       (swingv_prev == stdAc::swingv_t::kOff)))
    ac->setSwingV(kLgAcSwingVToggle);
  ac->setSwingH(swingh != stdAc::swingh_t::kOff);
  // No Quiet setting available.
  // No Turbo setting available.
  ac->setLight(light);
  // No Filter setting available.
  // No Clean setting available.
  // No Beep setting available.
  // No Sleep setting available.
  // No Clock setting available.
  ac->send();
}
#endif  // SEND_LG

#if SEND_RHOSS
/// Send an Rhoss A/C message with the supplied settings.
/// @param[in, out] ac A Ptr to an IRRhossAc object to use.
/// @param[in] on The power setting.
/// @param[in] mode The operation mode setting.
/// @param[in] degrees The temperature setting in degrees.
/// @param[in] fan The speed setting for the fan.
/// @param[in] swing The swing setting.
void IRac::rhoss(IRRhossAc *ac,
                const bool on, const stdAc::opmode_t mode, const float degrees,
                const stdAc::fanspeed_t fan, const stdAc::swingv_t swing) {
  ac->begin();
  ac->setPower(on);
  ac->setMode(ac->convertMode(mode));
  ac->setSwing(swing != stdAc::swingv_t::kOff);
  ac->setTemp(degrees);
  ac->setFan(ac->convertFan(fan));
  // No Quiet setting available.
  // No Light setting available.
  // No Filter setting available.
  // No Turbo setting available.
  // No Economy setting available.
  // No Clean setting available.
  // No Beep setting available.
  // No Sleep setting available.
  ac->send();
}
#endif  // SEND_RHOSS

/// Create a new state base on the provided state that has been suitably fixed.
/// @note This is for use with Home Assistant, which requires mode to be off if
///   the power is off.
/// @param[in] state The state_t structure describing the desired a/c state.
/// @return A stdAc::state_t with the needed settings.
stdAc::state_t IRac::cleanState(const stdAc::state_t state) {
  stdAc::state_t result = state;
  // A hack for Home Assistant, it appears to need/want an Off opmode.
  // So enforce the power is off if the mode is also off.
  if (state.mode == stdAc::opmode_t::kOff) result.power = false;
  return result;
}

/// Create a new state base on desired & previous states but handle
/// any state changes for options that need to be toggled.
/// @param[in] desired The state_t structure describing the desired a/c state.
/// @param[in] prev A Ptr to the previous state_t structure.
/// @return A stdAc::state_t with the needed settings.
stdAc::state_t IRac::handleToggles(const stdAc::state_t desired,
                                   const stdAc::state_t *prev) {
  stdAc::state_t result = desired;
  // If we've been given a previous state AND the it's the same A/C basically.
  if (prev != NULL && desired.protocol == prev->protocol &&
      desired.model == prev->model) {
    // Check if we have to handle toggle settings for specific A/C protocols.
    switch (desired.protocol) {
      case decode_type_t::COOLIX:
      case decode_type_t::TRANSCOLD:
        if ((desired.swingv == stdAc::swingv_t::kOff) ^
            (prev->swingv == stdAc::swingv_t::kOff))  // It changed, so toggle.
          result.swingv = stdAc::swingv_t::kAuto;
        else
          result.swingv = stdAc::swingv_t::kOff;  // No change, so no toggle.
        result.turbo = desired.turbo ^ prev->turbo;
        result.light = desired.light ^ prev->light;
        result.clean = desired.clean ^ prev->clean;
        result.sleep = ((desired.sleep >= 0) ^ (prev->sleep >= 0)) ? 0 : -1;
        break;
      case decode_type_t::DAIKIN128:
        result.power = desired.power ^ prev->power;
        result.light = desired.light ^ prev->light;
        break;
      case decode_type_t::ELECTRA_AC:
        result.light = desired.light ^ prev->light;
        break;
      case decode_type_t::FUJITSU_AC:
        result.turbo = desired.turbo ^ prev->turbo;
        result.econo = desired.econo ^ prev->econo;
        break;
      case decode_type_t::MIDEA:
        result.turbo = desired.turbo ^ prev->turbo;
        result.econo = desired.econo ^ prev->econo;
        result.light = desired.light ^ prev->light;
        result.clean = desired.clean ^ prev->clean;
        // FALL THRU
      case decode_type_t::CORONA_AC:
      case decode_type_t::HITACHI_AC344:
      case decode_type_t::HITACHI_AC424:
        if ((desired.swingv == stdAc::swingv_t::kOff) ^
            (prev->swingv == stdAc::swingv_t::kOff))  // It changed, so toggle.
          result.swingv = stdAc::swingv_t::kAuto;
        else
          result.swingv = stdAc::swingv_t::kOff;  // No change, so no toggle.
        break;
      case decode_type_t::SHARP_AC:
        result.light = desired.light ^ prev->light;
        if ((desired.swingv == stdAc::swingv_t::kOff) ^
            (prev->swingv == stdAc::swingv_t::kOff))  // It changed, so toggle.
          result.swingv = stdAc::swingv_t::kAuto;
        else
          result.swingv = stdAc::swingv_t::kOff;  // No change, so no toggle.
        break;
      case decode_type_t::KELON:
        if ((desired.swingv == stdAc::swingv_t::kOff) ^
            (prev->swingv == stdAc::swingv_t::kOff))  // It changed, so toggle.
          result.swingv = stdAc::swingv_t::kAuto;
        else
          result.swingv = stdAc::swingv_t::kOff;  // No change, so no toggle.
        // FALL-THRU
      case decode_type_t::AIRWELL:
      case decode_type_t::DAIKIN64:
      case decode_type_t::PANASONIC_AC32:
      case decode_type_t::WHIRLPOOL_AC:
        result.power = desired.power ^ prev->power;
        break;
      case decode_type_t::MIRAGE:
        if (desired.model == mirage_ac_remote_model_t::KKG29AC1)
          result.light = desired.light ^ prev->light;
        result.clean = desired.clean ^ prev->clean;
        break;
      case decode_type_t::PANASONIC_AC:
        // CKP models use a power mode toggle.
        if (desired.model == panasonic_ac_remote_model_t::kPanasonicCkp)
          result.power = desired.power ^ prev->power;
        break;
      case decode_type_t::SAMSUNG_AC:
        result.beep = desired.beep ^ prev->beep;
        result.clean = desired.clean ^ prev->clean;
        break;
      default:
        {};
    }
  }
  return result;
}

/// Send A/C message for a given device using common A/C settings.
/// @param[in] vendor The vendor/protocol type.
/// @param[in] model The A/C model if applicable.
/// @param[in] power The power setting.
/// @param[in] mode The operation mode setting.
/// @note Changing mode from "Off" to something else does NOT turn on a device.
/// You need to use `power` for that.
/// @param[in] degrees The temperature setting in degrees.
/// @param[in] celsius Temperature units. True is Celsius, False is Fahrenheit.
/// @param[in] fan The speed setting for the fan.
/// @note The following are all "if supported" by the underlying A/C classes.
/// @param[in] swingv The vertical swing setting.
/// @param[in] swingh The horizontal swing setting.
/// @param[in] quiet Run the device in quiet/silent mode.
/// @param[in] turbo Run the device in turbo/powerful mode.
/// @param[in] econo Run the device in economical mode.
/// @param[in] light Turn on the LED/Display mode.
/// @param[in] filter Turn on the (ion/pollen/etc) filter mode.
/// @param[in] clean Turn on the self-cleaning mode. e.g. Mould, dry filters etc
/// @param[in] beep Enable/Disable beeps when receiving IR messages.
/// @param[in] sleep Nr. of minutes for sleep mode.
///  -1 is Off, >= 0 is on. Some devices it is the nr. of mins to run for.
///  Others it may be the time to enter/exit sleep mode.
///  i.e. Time in Nr. of mins since midnight.
/// @param[in] clock The time in Nr. of mins since midnight. < 0 is ignore.
/// @return True, if accepted/converted/attempted etc. False, if unsupported.
bool IRac::sendAc(const decode_type_t vendor, const int16_t model,
                  const bool power, const stdAc::opmode_t mode,
                  const float degrees, const bool celsius,
                  const stdAc::fanspeed_t fan,
                  const stdAc::swingv_t swingv, const stdAc::swingh_t swingh,
                  const bool quiet, const bool turbo, const bool econo,
                  const bool light, const bool filter, const bool clean,
                  const bool beep, const int16_t sleep, const int16_t clock) {
  stdAc::state_t to_send;
  initState(&to_send, vendor, model, power, mode, degrees, celsius, fan, swingv,
            swingh, quiet, turbo, econo, light, filter, clean, beep, sleep,
            clock);
  return this->sendAc(to_send, &to_send);
}

/// Send A/C message for a given device using state_t structures.
/// @param[in] desired The state_t structure describing the desired new ac state
/// @param[in] prev A Ptr to the state_t structure containing the previous state
/// @note Changing mode from "Off" to something else does NOT turn on a device.
/// You need to use `power` for that.
/// @return True, if accepted/converted/attempted etc. False, if unsupported.
bool IRac::sendAc(const stdAc::state_t desired, const stdAc::state_t *prev) {
  // Convert the temp from Fahrenheit to Celsius if we are not in Celsius mode.
  float degC __attribute__((unused)) =
      desired.celsius ? desired.degrees : fahrenheitToCelsius(desired.degrees);
  // Convert the sensorTemp from Fahrenheit to Celsius if we are not in Celsius
  // mode.
  float sensorTempC __attribute__((unused)) =
      desired.sensorTemperature ? desired.sensorTemperature
          : fahrenheitToCelsius(desired.sensorTemperature);
  // special `state_t` that is required to be sent based on that.
  stdAc::state_t send = this->handleToggles(this->cleanState(desired), prev);
  // Some protocols expect a previous state for power.
  // Construct a pointer-safe previous power state incase prev is NULL/NULLPTR.
#if (SEND_HITACHI_AC1 || SEND_SAMSUNG_AC || SEND_SHARP_AC)
  const bool prev_power = (prev != NULL) ? prev->power : !send.power;
  const int16_t prev_sleep = (prev != NULL) ? prev->sleep : -1;
#endif  // (SEND_HITACHI_AC1 || SEND_SAMSUNG_AC || SEND_SHARP_AC)
#if (SEND_LG || SEND_SHARP_AC)
  const stdAc::swingv_t prev_swingv = (prev != NULL) ? prev->swingv
                                                     : stdAc::swingv_t::kOff;
#endif  // (SEND_LG || SEND_SHARP_AC)
#if (SEND_HAIER_AC160)
  const bool prev_light = (prev != NULL) ? prev->light : !send.light;
#endif  // (SEND_HAIER_AC160)
#if SEND_MIDEA
  const bool prev_quiet = (prev != NULL) ? prev->quiet : !send.quiet;
#endif  // SEND_MIDEA
  // Per vendor settings & setup.
  switch (send.protocol) {
#if SEND_LG
    case LG:
    case LG2:
    {
      IRLgAc ac(_pin, _inverted, _modulation);
      lg(&ac, (lg_ac_remote_model_t)send.model, send.power, send.mode,
         send.degrees, send.fanspeed, send.swingv, prev_swingv, send.swingh,
         send.light);
      break;
    }
#endif  // SEND_LG
#if SEND_RHOSS
    case RHOSS:
    {
      IRRhossAc ac(_pin, _inverted, _modulation);
      rhoss(&ac, send.power, send.mode, degC, send.fanspeed, send.swingv);
      break;
    }
#endif  // SEND_RHOSS
    default:
      return false;  // Fail, didn't match anything.
  }
  return true;  // Success.
}  // NOLINT(readability/fn_size)

/// Update the previous state to the current one.
void IRac::markAsSent(void) {
  _prev = next;
}

/// Send an A/C message based soley on our internal state.
/// @return True, if accepted/converted/attempted. False, if unsupported.
bool IRac::sendAc(void) {
  bool success = this->sendAc(next, &_prev);
  if (success) this->markAsSent();
  return success;
}

/// Compare two AirCon states.
/// @note The comparison excludes the clock.
/// @param a A state_t to be compared.
/// @param b A state_t to be compared.
/// @return True if they differ, False if they don't.
bool IRac::cmpStates(const stdAc::state_t a, const stdAc::state_t b) {
  return a.protocol != b.protocol || a.model != b.model || a.power != b.power ||
      a.mode != b.mode || a.degrees != b.degrees || a.celsius != b.celsius ||
      a.fanspeed != b.fanspeed || a.swingv != b.swingv ||
      a.swingh != b.swingh || a.quiet != b.quiet || a.turbo != b.turbo ||
      a.econo != b.econo || a.light != b.light || a.filter != b.filter ||
      a.clean != b.clean || a.beep != b.beep || a.sleep != b.sleep ||
      a.command != b.command || a.sensorTemperature != b.sensorTemperature ||
      a.iFeel != b.iFeel;
}

/// Check if the internal state has changed from what was previously sent.
/// @note The comparison excludes the clock.
/// @return True if it has changed, False if not.
bool IRac::hasStateChanged(void) { return cmpStates(next, _prev); }

/// Convert the supplied str into the appropriate enum.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
stdAc::ac_command_t IRac::strToCommandType(const char *str,
                                           const stdAc::ac_command_t def) {
  if (!STRCASECMP(str, kControlCommandStr))
    return stdAc::ac_command_t::kControlCommand;
  else if (!STRCASECMP(str, kIFeelReportStr) ||
           !STRCASECMP(str, kIFeelStr))
    return stdAc::ac_command_t::kSensorTempReport;
  else if (!STRCASECMP(str, kSetTimerCommandStr) ||
           !STRCASECMP(str, kTimerStr))
    return stdAc::ac_command_t::kTimerCommand;
  else if (!STRCASECMP(str, kConfigCommandStr))
    return stdAc::ac_command_t::kConfigCommand;
  else
    return def;
}

/// Convert the supplied str into the appropriate enum.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
stdAc::opmode_t IRac::strToOpmode(const char *str,
                                  const stdAc::opmode_t def) {
  if (!STRCASECMP(str, kAutoStr) ||
      !STRCASECMP(str, kAutomaticStr))
    return stdAc::opmode_t::kAuto;
  else if (!STRCASECMP(str, kOffStr) ||
           !STRCASECMP(str, kStopStr))
    return stdAc::opmode_t::kOff;
  else if (!STRCASECMP(str, kCoolStr) ||
           !STRCASECMP(str, kCoolingStr))
    return stdAc::opmode_t::kCool;
  else if (!STRCASECMP(str, kHeatStr) ||
           !STRCASECMP(str, kHeatingStr))
    return stdAc::opmode_t::kHeat;
  else if (!STRCASECMP(str, kDryStr) ||
           !STRCASECMP(str, kDryingStr) ||
           !STRCASECMP(str, kDehumidifyStr))
    return stdAc::opmode_t::kDry;
  else if (!STRCASECMP(str, kFanStr) ||
          // The following Fans strings with "only" are required to help with
          // HomeAssistant & Google Home Climate integration.
          // For compatibility only.
          // Ref: https://www.home-assistant.io/integrations/google_assistant/#climate-operation-modes
           !STRCASECMP(str, kFanOnlyStr) ||
           !STRCASECMP(str, kFan_OnlyStr) ||
           !STRCASECMP(str, kFanOnlyWithSpaceStr) ||
           !STRCASECMP(str, kFanOnlyNoSpaceStr))
    return stdAc::opmode_t::kFan;
  else
    return def;
}

/// Convert the supplied str into the appropriate enum.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
stdAc::fanspeed_t IRac::strToFanspeed(const char *str,
                                      const stdAc::fanspeed_t def) {
  if (!STRCASECMP(str, kAutoStr) ||
      !STRCASECMP(str, kAutomaticStr))
    return stdAc::fanspeed_t::kAuto;
  else if (!STRCASECMP(str, kMinStr) ||
           !STRCASECMP(str, kMinimumStr) ||
           !STRCASECMP(str, kLowestStr))
    return stdAc::fanspeed_t::kMin;
  else if (!STRCASECMP(str, kLowStr) ||
           !STRCASECMP(str, kLoStr))
    return stdAc::fanspeed_t::kLow;
  else if (!STRCASECMP(str, kMedStr) ||
           !STRCASECMP(str, kMediumStr) ||
           !STRCASECMP(str, kMidStr))
    return stdAc::fanspeed_t::kMedium;
  else if (!STRCASECMP(str, kHighStr) ||
           !STRCASECMP(str, kHiStr))
    return stdAc::fanspeed_t::kHigh;
  else if (!STRCASECMP(str, kMaxStr) ||
           !STRCASECMP(str, kMaximumStr) ||
           !STRCASECMP(str, kHighestStr))
    return stdAc::fanspeed_t::kMax;
  else if (!STRCASECMP(str, kMedHighStr))
    return stdAc::fanspeed_t::kMediumHigh;
  else
    return def;
}

/// Convert the supplied str into the appropriate enum.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
stdAc::swingv_t IRac::strToSwingV(const char *str,
                                  const stdAc::swingv_t def) {
  if (!STRCASECMP(str, kAutoStr) ||
      !STRCASECMP(str, kAutomaticStr) ||
      !STRCASECMP(str, kOnStr) ||
      !STRCASECMP(str, kSwingStr))
    return stdAc::swingv_t::kAuto;
  else if (!STRCASECMP(str, kOffStr) ||
           !STRCASECMP(str, kStopStr))
    return stdAc::swingv_t::kOff;
  else if (!STRCASECMP(str, kMinStr) ||
           !STRCASECMP(str, kMinimumStr) ||
           !STRCASECMP(str, kLowestStr) ||
           !STRCASECMP(str, kBottomStr) ||
           !STRCASECMP(str, kDownStr))
    return stdAc::swingv_t::kLowest;
  else if (!STRCASECMP(str, kLowStr))
    return stdAc::swingv_t::kLow;
  else if (!STRCASECMP(str, kMidStr) ||
           !STRCASECMP(str, kMiddleStr) ||
           !STRCASECMP(str, kMedStr) ||
           !STRCASECMP(str, kMediumStr) ||
           !STRCASECMP(str, kCentreStr))
    return stdAc::swingv_t::kMiddle;
  else if (!STRCASECMP(str, kUpperMiddleStr))
    return stdAc::swingv_t::kUpperMiddle;
  else if (!STRCASECMP(str, kHighStr) ||
           !STRCASECMP(str, kHiStr))
    return stdAc::swingv_t::kHigh;
  else if (!STRCASECMP(str, kHighestStr) ||
           !STRCASECMP(str, kMaxStr) ||
           !STRCASECMP(str, kMaximumStr) ||
           !STRCASECMP(str, kTopStr) ||
           !STRCASECMP(str, kUpStr))
    return stdAc::swingv_t::kHighest;
  else
    return def;
}

/// Convert the supplied str into the appropriate enum.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
stdAc::swingh_t IRac::strToSwingH(const char *str,
                                  const stdAc::swingh_t def) {
  if (!STRCASECMP(str, kAutoStr) ||
      !STRCASECMP(str, kAutomaticStr) ||
      !STRCASECMP(str, kOnStr) || !STRCASECMP(str, kSwingStr))
    return stdAc::swingh_t::kAuto;
  else if (!STRCASECMP(str, kOffStr) ||
           !STRCASECMP(str, kStopStr))
    return stdAc::swingh_t::kOff;
  else if (!STRCASECMP(str, kLeftMaxNoSpaceStr) ||              // "LeftMax"
           !STRCASECMP(str, kLeftMaxStr) ||                     // "Left Max"
           !STRCASECMP(str, kMaxLeftNoSpaceStr) ||              // "MaxLeft"
           !STRCASECMP(str, kMaxLeftStr))                       // "Max Left"
    return stdAc::swingh_t::kLeftMax;
  else if (!STRCASECMP(str, kLeftStr))
    return stdAc::swingh_t::kLeft;
  else if (!STRCASECMP(str, kMidStr) ||
           !STRCASECMP(str, kMiddleStr) ||
           !STRCASECMP(str, kMedStr) ||
           !STRCASECMP(str, kMediumStr) ||
           !STRCASECMP(str, kCentreStr))
    return stdAc::swingh_t::kMiddle;
  else if (!STRCASECMP(str, kRightStr))
    return stdAc::swingh_t::kRight;
  else if (!STRCASECMP(str, kRightMaxNoSpaceStr) ||              // "RightMax"
           !STRCASECMP(str, kRightMaxStr) ||                     // "Right Max"
           !STRCASECMP(str, kMaxRightNoSpaceStr) ||              // "MaxRight"
           !STRCASECMP(str, kMaxRightStr))                       // "Max Right"
    return stdAc::swingh_t::kRightMax;
  else if (!STRCASECMP(str, kWideStr))
    return stdAc::swingh_t::kWide;
  else
    return def;
}

/// Convert the supplied str into the appropriate enum.
/// @note Assumes str is the model code or an integer >= 1.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The enum to return if no conversion was possible.
/// @return The equivalent enum.
/// @note After adding a new model you should update modelToStr() too.
int16_t IRac::strToModel(const char *str, const int16_t def) {
  // Gree
  if (!STRCASECMP(str, kYaw1fStr)) {
    return gree_ac_remote_model_t::YAW1F;
  } else if (!STRCASECMP(str, kYbofbStr)) {
    return gree_ac_remote_model_t::YBOFB;
  } else if (!STRCASECMP(str, kYx1fsfStr)) {
    return gree_ac_remote_model_t::YX1FSF;
  // Haier models
  } else if (!STRCASECMP(str, kV9014557AStr)) {
    return haier_ac176_remote_model_t::V9014557_A;
  } else if (!STRCASECMP(str, kV9014557BStr)) {
    return haier_ac176_remote_model_t::V9014557_B;
  // HitachiAc1 models
  } else if (!STRCASECMP(str, kRlt0541htaaStr)) {
    return hitachi_ac1_remote_model_t::R_LT0541_HTA_A;
  } else if (!STRCASECMP(str, kRlt0541htabStr)) {
    return hitachi_ac1_remote_model_t::R_LT0541_HTA_B;
  // Fujitsu A/C models
  } else if (!STRCASECMP(str, kArrah2eStr)) {
    return fujitsu_ac_remote_model_t::ARRAH2E;
  } else if (!STRCASECMP(str, kArdb1Str)) {
    return fujitsu_ac_remote_model_t::ARDB1;
  } else if (!STRCASECMP(str, kArreb1eStr)) {
    return fujitsu_ac_remote_model_t::ARREB1E;
  } else if (!STRCASECMP(str, kArjw2Str)) {
    return fujitsu_ac_remote_model_t::ARJW2;
  } else if (!STRCASECMP(str, kArry4Str)) {
    return fujitsu_ac_remote_model_t::ARRY4;
  } else if (!STRCASECMP(str, kArrew4eStr)) {
    return fujitsu_ac_remote_model_t::ARREW4E;
  // LG A/C models
  } else if (!STRCASECMP(str, kGe6711ar2853mStr)) {
    return lg_ac_remote_model_t::GE6711AR2853M;
  } else if (!STRCASECMP(str, kAkb75215403Str)) {
    return lg_ac_remote_model_t::AKB75215403;
  } else if (!STRCASECMP(str, kAkb74955603Str)) {
    return lg_ac_remote_model_t::AKB74955603;
  } else if (!STRCASECMP(str, kAkb73757604Str)) {
    return lg_ac_remote_model_t::AKB73757604;
  } else if (!STRCASECMP(str, kLg6711a20083vStr)) {
    return lg_ac_remote_model_t::LG6711A20083V;
  // Panasonic A/C families
  } else if (!STRCASECMP(str, kLkeStr) ||
             !STRCASECMP(str, kPanasonicLkeStr)) {
    return panasonic_ac_remote_model_t::kPanasonicLke;
  } else if (!STRCASECMP(str, kNkeStr) ||
             !STRCASECMP(str, kPanasonicNkeStr)) {
    return panasonic_ac_remote_model_t::kPanasonicNke;
  } else if (!STRCASECMP(str, kDkeStr) ||
             !STRCASECMP(str, kPanasonicDkeStr) ||
             !STRCASECMP(str, kPkrStr) ||
             !STRCASECMP(str, kPanasonicPkrStr)) {
    return panasonic_ac_remote_model_t::kPanasonicDke;
  } else if (!STRCASECMP(str, kJkeStr) ||
             !STRCASECMP(str, kPanasonicJkeStr)) {
    return panasonic_ac_remote_model_t::kPanasonicJke;
  } else if (!STRCASECMP(str, kCkpStr) ||
             !STRCASECMP(str, kPanasonicCkpStr)) {
    return panasonic_ac_remote_model_t::kPanasonicCkp;
  } else if (!STRCASECMP(str, kRkrStr) ||
             !STRCASECMP(str, kPanasonicRkrStr)) {
    return panasonic_ac_remote_model_t::kPanasonicRkr;
  // Sharp A/C Models
  } else if (!STRCASECMP(str, kA907Str)) {
    return sharp_ac_remote_model_t::A907;
  } else if (!STRCASECMP(str, kA705Str)) {
    return sharp_ac_remote_model_t::A705;
  } else if (!STRCASECMP(str, kA903Str)) {
    return sharp_ac_remote_model_t::A903;
  // TCL A/C Models
  } else if (!STRCASECMP(str, kTac09chsdStr)) {
    return tcl_ac_remote_model_t::TAC09CHSD;
  } else if (!STRCASECMP(str, kGz055be1Str)) {
    return tcl_ac_remote_model_t::GZ055BE1;
  // Voltas A/C models
  } else if (!STRCASECMP(str, k122lzfStr)) {
    return voltas_ac_remote_model_t::kVoltas122LZF;
  // Whirlpool A/C models
  } else if (!STRCASECMP(str, kDg11j13aStr) ||
             !STRCASECMP(str, kDg11j104Str)) {
    return whirlpool_ac_remote_model_t::DG11J13A;
  } else if (!STRCASECMP(str, kDg11j191Str)) {
    return whirlpool_ac_remote_model_t::DG11J191;
  // Argo A/C models
  } else if (!STRCASECMP(str, kArgoWrem2Str)) {
    return argo_ac_remote_model_t::SAC_WREM2;
  } else if (!STRCASECMP(str, kArgoWrem3Str)) {
    return argo_ac_remote_model_t::SAC_WREM3;
  } else {
    int16_t number = atoi(str);
    if (number > 0)
      return number;
    else
      return def;
  }
}

/// Convert the supplied str into the appropriate boolean value.
/// @param[in] str A Ptr to a C-style string to be converted.
/// @param[in] def The boolean value to return if no conversion was possible.
/// @return The equivalent boolean value.
bool IRac::strToBool(const char *str, const bool def) {
  if (!STRCASECMP(str, kOnStr) ||
      !STRCASECMP(str, k1Str) ||
      !STRCASECMP(str, kYesStr) ||
      !STRCASECMP(str, kTrueStr))
    return true;
  else if (!STRCASECMP(str, kOffStr) ||
           !STRCASECMP(str, k0Str) ||
           !STRCASECMP(str, kNoStr) ||
           !STRCASECMP(str, kFalseStr))
    return false;
  else
    return def;
}

/// Convert the supplied boolean into the appropriate String.
/// @param[in] value The boolean value to be converted.
/// @return The equivalent String for the locale.
String IRac::boolToString(const bool value) {
  return value ? kOnStr : kOffStr;
}

/// Convert the supplied operation mode into the appropriate String.
/// @param[in] cmdType The enum to be converted.
/// @return The equivalent String for the locale.
String IRac::commandTypeToString(const stdAc::ac_command_t cmdType) {
  switch (cmdType) {
    case stdAc::ac_command_t::kControlCommand:    return kControlCommandStr;
    case stdAc::ac_command_t::kSensorTempReport: return kIFeelReportStr;
    case stdAc::ac_command_t::kTimerCommand:      return kSetTimerCommandStr;
    case stdAc::ac_command_t::kConfigCommand:     return kConfigCommandStr;
    default:                                      return kUnknownStr;
  }
}

/// Convert the supplied operation mode into the appropriate String.
/// @param[in] mode The enum to be converted.
/// @param[in] ha A flag to indicate we want GoogleHome/HomeAssistant output.
/// @return The equivalent String for the locale.
String IRac::opmodeToString(const stdAc::opmode_t mode, const bool ha) {
  switch (mode) {
    case stdAc::opmode_t::kOff:  return kOffStr;
    case stdAc::opmode_t::kAuto: return kAutoStr;
    case stdAc::opmode_t::kCool: return kCoolStr;
    case stdAc::opmode_t::kHeat: return kHeatStr;
    case stdAc::opmode_t::kDry:  return kDryStr;
    case stdAc::opmode_t::kFan:  return ha ? kFan_OnlyStr : kFanStr;
    default:                     return kUnknownStr;
  }
}

/// Convert the supplied fan speed enum into the appropriate String.
/// @param[in] speed The enum to be converted.
/// @return The equivalent String for the locale.
String IRac::fanspeedToString(const stdAc::fanspeed_t speed) {
  switch (speed) {
    case stdAc::fanspeed_t::kAuto:       return kAutoStr;
    case stdAc::fanspeed_t::kMax:        return kMaxStr;
    case stdAc::fanspeed_t::kHigh:       return kHighStr;
    case stdAc::fanspeed_t::kMedium:     return kMediumStr;
    case stdAc::fanspeed_t::kMediumHigh: return kMedHighStr;
    case stdAc::fanspeed_t::kLow:        return kLowStr;
    case stdAc::fanspeed_t::kMin:        return kMinStr;
    default:                             return kUnknownStr;
  }
}

/// Convert the supplied enum into the appropriate String.
/// @param[in] swingv The enum to be converted.
/// @return The equivalent String for the locale.
String IRac::swingvToString(const stdAc::swingv_t swingv) {
  switch (swingv) {
    case stdAc::swingv_t::kOff:          return kOffStr;
    case stdAc::swingv_t::kAuto:         return kAutoStr;
    case stdAc::swingv_t::kHighest:      return kHighestStr;
    case stdAc::swingv_t::kHigh:         return kHighStr;
    case stdAc::swingv_t::kMiddle:       return kMiddleStr;
    case stdAc::swingv_t::kUpperMiddle:  return kUpperMiddleStr;
    case stdAc::swingv_t::kLow:          return kLowStr;
    case stdAc::swingv_t::kLowest:       return kLowestStr;
    default:                             return kUnknownStr;
  }
}

/// Convert the supplied enum into the appropriate String.
/// @param[in] swingh The enum to be converted.
/// @return The equivalent String for the locale.
String IRac::swinghToString(const stdAc::swingh_t swingh) {
  switch (swingh) {
    case stdAc::swingh_t::kOff:      return kOffStr;
    case stdAc::swingh_t::kAuto:     return kAutoStr;
    case stdAc::swingh_t::kLeftMax:  return kLeftMaxStr;
    case stdAc::swingh_t::kLeft:     return kLeftStr;
    case stdAc::swingh_t::kMiddle:   return kMiddleStr;
    case stdAc::swingh_t::kRight:    return kRightStr;
    case stdAc::swingh_t::kRightMax: return kRightMaxStr;
    case stdAc::swingh_t::kWide:     return kWideStr;
    default:                         return kUnknownStr;
  }
}

namespace IRAcUtils {
  /// Display the human readable state of an A/C message if we can.
  /// @param[in] result A Ptr to the captured `decode_results` that contains an
  ///   A/C mesg.
  /// @return A string with the human description of the A/C message.
  ///   An empty string if we can't.
  String resultAcToString(const decode_results * const result) {
    switch (result->decode_type) {
#if DECODE_LG
      case decode_type_t::LG:
      case decode_type_t::LG2: {
        IRLgAc ac(kGpioUnused);
        ac.setRaw(result->value, result->decode_type);  // Use value, not state.
        return ac.isValidLgAc() ? ac.toString() : "";
      }
#endif  // DECODE_LG
#if DECODE_RHOSS
    case decode_type_t::RHOSS: {
      IRRhossAc ac(kGpioUnused);
      ac.setRaw(result->state);
      return ac.toString();
    }
#endif  // DECODE_RHOSS
      default:
        return "";
    }
  }

  /// Convert a valid IR A/C remote message that we understand enough into a
  /// Common A/C state.
  /// @param[in] decode A PTR to a successful raw IR decode object.
  /// @param[in] result A PTR to a state structure to store the result in.
  /// @param[in] prev A PTR to a state structure which has the prev. state.
  /// @return A boolean indicating success or failure.
  bool decodeToState(const decode_results *decode, stdAc::state_t *result,
                     const stdAc::state_t *prev
/// @cond IGNORE
// *prev flagged as "unused" due to potential compiler warning when some
// protocols that use it are disabled. It really is used.
                                                __attribute__((unused))
/// @endcond
                    ) {
    if (decode == NULL || result == NULL) return false;  // Safety check.
    switch (decode->decode_type) {
#if DECODE_LG
      case decode_type_t::LG:
      case decode_type_t::LG2: {
        IRLgAc ac(kGpioUnused);
        ac.setRaw(decode->value, decode->decode_type);  // Use value, not state.
        if (!ac.isValidLgAc()) return false;
        *result = ac.toCommon(prev);
        break;
      }
#endif  // DECODE_LG
#if DECODE_RHOSS
      case decode_type_t::RHOSS: {
        IRRhossAc ac(kGpioUnused);
        ac.setRaw(decode->state);
        *result = ac.toCommon();
        break;
      }
#endif  // DECODE_RHOSS
      default:
        return false;
    }
    return true;
  }
}  // namespace IRAcUtils
