from quad_msgs.msg import (
    BodyForceEstimate,
    BodyPlan,
    BodyState,
    ContactMode,
    FootContact,
    FootPlanDiscrete,
    FootState,
    GRFArray,
    LegCommand,
    LegCommandArray,
    LegContactMode,
    LocalPlan,
    MotorCommand,
    MultiFootPlanContinuous,
    MultiFootPlanDiscrete,
    MultiFootState,
    RobotPlan,
    RobotPlanConstraints,
    RobotPlanDiagnostics,
    RobotState,
)
from quad_msgs.srv import PlanWithConstraints


def test_all_generated_messages_construct():
    message_types = [
        BodyForceEstimate,
        BodyPlan,
        BodyState,
        ContactMode,
        FootContact,
        FootPlanDiscrete,
        FootState,
        GRFArray,
        LegCommand,
        LegCommandArray,
        LegContactMode,
        LocalPlan,
        MotorCommand,
        MultiFootPlanContinuous,
        MultiFootPlanDiscrete,
        MultiFootState,
        RobotPlan,
        RobotPlanConstraints,
        RobotPlanDiagnostics,
        RobotState,
    ]

    for message_type in message_types:
        assert message_type() is not None


def test_nested_plan_messages_accept_expected_fields():
    state = RobotState()
    state.body.pose.position.x = 1.0
    state.joints.position = [0.1] * 12
    state.feet.feet = [FootState() for _ in range(4)]

    grf = GRFArray()
    grf.vectors = []
    grf.points = []
    grf.contact_states = [True, False, True, False]

    plan = RobotPlan()
    plan.states.append(state)
    plan.grfs.append(grf)
    plan.plan_indices.append(3)
    plan.primitive_ids.append(1)
    plan.diagnostics.horizon_length = 26

    assert plan.states[0].body.pose.position.x == 1.0
    assert len(plan.states[0].joints.position) == 12
    assert len(plan.states[0].feet.feet) == 4
    assert list(plan.grfs[0].contact_states) == [True, False, True, False]
    assert list(plan.plan_indices) == [3]
    assert list(plan.primitive_ids) == [1]
    assert plan.diagnostics.horizon_length == 26


def test_plan_with_constraints_service_constructs_request_and_response():
    request = PlanWithConstraints.Request()
    request.constraints.pos_x = [1.0, 2.0]
    request.constraints.pos_y = [0.0, 0.5]
    request.constraints.pos_z = [0.3, 0.3]
    request.constraints.yaw = [0.0, 0.1]
    request.constraints.t_start = [0.0, 0.1]
    request.constraints.t_end = [0.1, 0.2]
    request.warm_start = True

    response = PlanWithConstraints.Response()
    response.success = True
    response.plan.plan_indices = [0, 1]

    assert request.warm_start is True
    assert list(request.constraints.pos_x) == [1.0, 2.0]
    assert response.success is True
    assert list(response.plan.plan_indices) == [0, 1]
