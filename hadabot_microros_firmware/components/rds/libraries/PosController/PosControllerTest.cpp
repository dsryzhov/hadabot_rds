class TestMotionController : public IMotionController {
public:
    TestMotionController() {
        linear_velocity = 0;
        angular_velocity = 0;
    }

     void updateMotion(float _linear_velocity, float _angular_velocity) override {

     };

     bool checkMotion(float _linear_velocity, float _angular_velocity) {
         return (_linear_velocity = linear_velocity && angular_velocity == _angular_velocity);

     }

    float linear_velocity;
    float angular_velocity;
}

void test_pos_controller_MOVE_FORWARD() {
    TestMotionController motion_controller;
    PosController controller;
    controller.init(&motion_controller);

    Position goal_position(10, 0.1, 0);
    Position current_position(0, 0, 0);
    Twist current_twist(0, 0);

    controller.setGoalPosition(goal_position);
    controller.setCurrentPosition(current_position, current_twist);
    controller.setParams(0.45, 7.85, 0.1, 0.3, 0.3);

    controller.updateMotion();
    MotionType motion_type = contorller.getMotionType();
    assert(motion_type == MOVE_FORWARD);
    assert(motion_controller->checkMotion(DESIRED_LINEAR_VELOCITY, 0));
}

void test_pos_controller_ROTATE_TO_GOAL_POS_CCW() {
    TestMotionController motion_controller;
    PosController controller;
    controller.init(&motion_controller);

    Position goal_position(10, 10, 0);
    Position current_position(0, 0, 0);
    Twist current_twist(0, 0);

    controller.setGoalPosition(goal_position);
    controller.setCurrentPosition(current_position, current_twist);
    controller.updateMotion();

    MotionType motion_type = controller.getMotionType();
    assert(motion_type == ROTATE_CCW);
    assert(motion_controller->checkMotion(0, DESIRED_ANGULAR_VELOCITY));
}

void test_pos_controller_ROTATE_TO_GOAL_POS_CW() {
    TestMotionController motion_controller;
    PosController controller;
    controller.init(&motion_controller);

    Position goal_position(10, -10, 0);
    Position current_position(0, 0, 0);
    Twist current_twist(0, 0);

    controller.setGoalPosition(goal_position);
    controller.setCurrentPosition(current_position, current_twist);
    controller.updateMotion();

    MotionType motion_type = controller.getMotionType();
    assert(motion_type == ROTATE_CW);
    assert(motion_controller->checkMotion(0, -DESIRED_ANGULAR_VELOCITY));
}



void pos_controller_tests() {
    test_pos_controller_ROTATE_TO_GOAL_POS_CCW();
    test_pos_controller_ROTATE_TO_GOAL_POS_CW();
    test_pos_controller_ROTATE_TO_GOAL_ORIENTATION_CCW();
    test_pos_controller_ROTATE_TO_GOAL_ORIENTATION_CW();
    test_pos_controller_MOVE_FORWARD();
    test_pos_controller_STOP();
}

