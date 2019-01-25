#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

namespace state_machine
{

enum SystemState {
    start,
    exploring,
    planning,
    gate_valiating,
    gate_tracking,
    stop
};

class StateMachine
{
  public:
    // Current system state
    SystemState _state;

    // Set/Get method for state
    void SetState(SystemState state) { _state = state; };
    const SystemState& GetState() const { return _state; };

    // Action
    void Reset();
    void TryStateChange(SystemState target_state);

    // Set initial state as start
    StateMachine() { _state = SystemState::start; };
};

} // namespace state_machine

#endif