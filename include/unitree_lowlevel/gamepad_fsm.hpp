#pragma once

#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "unitree_lowlevel/gamepad.hpp"

namespace unitree {
namespace common {

// ===================================================================
// GamepadFSM — generic string-keyed state machine driven by gamepad
//
// Transition triggers use the format:
//   "button"               — on_press of a single button
//   "modifier + button"    — modifier held + on_press of button
//   "A + B + X"            — A held + B held + X on_press (last = trigger)
//
// The last token in a "+" chain is the *trigger* (checked via on_press).
// All preceding tokens are *modifiers* (checked via pressed).
//
// Example YAML:
//   transitions:
//     velocity: L2 + up
//     mimic:    R1 + down
//     hop:      X
//     combo:    L1 + R1 + A        # L1 held + R1 held + A on_press
// ===================================================================

class GamepadFSM {
public:
  // -------- types --------
  struct Transition {
    std::string target;                  // destination state name
    std::vector<std::string> modifiers;  // buttons that must be held
    std::string trigger;                 // button checked via on_press
  };

  using TransitionCallback = std::function<void(const std::string& from,
                                                const std::string& to)>;

  // -------- construction / setup --------
  GamepadFSM() = default;

  /// Load FSM from a YAML node with structure:
  ///   default: <state_name>
  ///   states:
  ///     state_a:
  ///       transitions:
  ///         state_b: L2+down
  ///         state_c: X
  ///     state_b:
  ///       transitions: ...
  void loadFromYAML(const YAML::Node& fsmNode) {
    if (!fsmNode["default"]) {
      throw std::runtime_error("[GamepadFSM] missing 'default' key");
    }
    default_state_ = fsmNode["default"].as<std::string>();

    const auto& states = fsmNode["states"];
    if (!states || !states.IsMap()) {
      throw std::runtime_error("[GamepadFSM] 'states' must be a map");
    }

    transitions_.clear();
    for (auto it = states.begin(); it != states.end(); ++it) {
      const std::string sname = it->first.as<std::string>();
      const auto& entry = it->second;

      std::vector<Transition> trs;
      if (entry["transitions"] && entry["transitions"].IsMap()) {
        for (auto ti = entry["transitions"].begin();
             ti != entry["transitions"].end(); ++ti) {
          const std::string target = ti->first.as<std::string>();
          const std::string trigger_str = ti->second.as<std::string>();
          trs.push_back(parseTrigger(target, trigger_str));
        }
      }
      transitions_[sname] = std::move(trs);
    }

    // Validate default exists in states
    if (transitions_.find(default_state_) == transitions_.end()) {
      throw std::runtime_error("[GamepadFSM] default state '" +
                               default_state_ + "' not found in states");
    }

    active_state_ = default_state_;
  }

  /// Manually add a single transition.
  void addTransition(const std::string& from_state,
                     const std::string& target,
                     const std::string& trigger_str) {
    transitions_[from_state].push_back(parseTrigger(target, trigger_str));
  }

  /// Set the active state (e.g. on external reset).
  void setState(const std::string& name) { active_state_ = name; }

  /// Set callback invoked on every transition.
  void setOnTransition(TransitionCallback cb) { on_transition_ = std::move(cb); }

  // -------- runtime --------

  /// Check gamepad and perform transition if triggered.
  /// Returns the new state name (unchanged if no transition fired).
  const std::string& update(const Gamepad& gp) {
    auto it = transitions_.find(active_state_);
    if (it == transitions_.end()) return active_state_;

    for (const auto& tr : it->second) {
      // All modifiers must be held
      bool mods_ok = true;
      for (const auto& mod : tr.modifiers) {
        if (!buttonPressed(gp, mod)) { mods_ok = false; break; }
      }
      if (!mods_ok) continue;

      // Trigger must have on_press this tick
      if (buttonOnPress(gp, tr.trigger)) {
        const std::string prev = active_state_;
        active_state_ = tr.target;
        if (on_transition_) on_transition_(prev, active_state_);
        return active_state_;
      }
    }
    return active_state_;
  }

  // -------- accessors --------
  const std::string& activeState() const { return active_state_; }
  const std::string& defaultState() const { return default_state_; }
  const std::unordered_map<std::string, std::vector<Transition>>&
  allTransitions() const { return transitions_; }

  // -------- static helpers (public so others can reuse) --------

  /// Parse trigger string like "L2+down", "R1+X", "A+B+X", or just "X".
  static Transition parseTrigger(const std::string& target,
                                 const std::string& trigger_str) {
    Transition tr;
    tr.target = target;

    // Split by '+'
    std::vector<std::string> tokens;
    std::istringstream ss(trigger_str);
    std::string tok;
    while (std::getline(ss, tok, '+')) {
      // trim whitespace
      auto b = tok.find_first_not_of(" \t");
      auto e = tok.find_last_not_of(" \t");
      if (b != std::string::npos)
        tokens.push_back(tok.substr(b, e - b + 1));
    }

    if (tokens.empty()) {
      throw std::runtime_error(
          "[GamepadFSM] empty trigger string for target '" + target + "'");
    }

    // Last token = trigger (on_press), all preceding = modifiers (pressed)
    tr.trigger = tokens.back();
    for (size_t i = 0; i + 1 < tokens.size(); ++i) {
      tr.modifiers.push_back(tokens[i]);
    }

    return tr;
  }

  /// Check if a named button is currently held.
  static bool buttonPressed(const Gamepad& gp, const std::string& btn) {
    if (btn == "A")      return gp.A.pressed;
    if (btn == "B")      return gp.B.pressed;
    if (btn == "X")      return gp.X.pressed;
    if (btn == "Y")      return gp.Y.pressed;
    if (btn == "up")     return gp.up.pressed;
    if (btn == "down")   return gp.down.pressed;
    if (btn == "left")   return gp.left.pressed;
    if (btn == "right")  return gp.right.pressed;
    if (btn == "L1")     return gp.L1.pressed;
    if (btn == "L2")     return gp.L2.pressed;
    if (btn == "R1")     return gp.R1.pressed;
    if (btn == "R2")     return gp.R2.pressed;
    if (btn == "start")  return gp.start.pressed;
    if (btn == "select") return gp.select.pressed;
    if (btn == "F1")     return gp.F1.pressed;
    if (btn == "F2")     return gp.F2.pressed;
    std::cerr << "[GamepadFSM] Unknown button: " << btn << std::endl;
    return false;
  }

  /// Check if a named button was just pressed this tick.
  static bool buttonOnPress(const Gamepad& gp, const std::string& btn) {
    if (btn == "A")      return gp.A.on_press;
    if (btn == "B")      return gp.B.on_press;
    if (btn == "X")      return gp.X.on_press;
    if (btn == "Y")      return gp.Y.on_press;
    if (btn == "up")     return gp.up.on_press;
    if (btn == "down")   return gp.down.on_press;
    if (btn == "left")   return gp.left.on_press;
    if (btn == "right")  return gp.right.on_press;
    if (btn == "L1")     return gp.L1.on_press;
    if (btn == "L2")     return gp.L2.on_press;
    if (btn == "R1")     return gp.R1.on_press;
    if (btn == "R2")     return gp.R2.on_press;
    if (btn == "start")  return gp.start.on_press;
    if (btn == "select") return gp.select.on_press;
    if (btn == "F1")     return gp.F1.on_press;
    if (btn == "F2")     return gp.F2.on_press;
    std::cerr << "[GamepadFSM] Unknown button: " << btn << std::endl;
    return false;
  }

private:
  std::string default_state_;
  std::string active_state_;
  std::unordered_map<std::string, std::vector<Transition>> transitions_;
  TransitionCallback on_transition_;
};

}  // namespace common
}  // namespace unitree
