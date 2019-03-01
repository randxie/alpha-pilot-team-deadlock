#include <gtest/gtest.h>
#include "state_machine.h"

TEST(TEST_STATE_CHANGE, test_state_change) {
    state_machine::StateMachine state_m;
    ASSERT_EQ(state_m.GetState(), state_machine::SystemState::start);

    state_m.SetState(state_machine::SystemState::exploring);
    ASSERT_EQ(state_m.GetState(), state_machine::SystemState::exploring);
}