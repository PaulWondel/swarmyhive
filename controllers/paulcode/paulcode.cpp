#include <webots/Supervisor.hpp>

#define TIME_STEP 32

using namespace webots;

int main() {
  Supervisor *supervisor = new Supervisor();

  // do this once only
  Node *robot_node = supervisor->getFromDef("botJr");
  Field *trans_field = robot_node->getField("translation");

  while (supervisor->step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *values = trans_field->getSFVec3f();
    std::cout << "BOT120 is at position: " << values[0] << ' '
              << values[1] << ' ' << values[2] << std::endl;
  }

  delete supervisor;
  return 0;
}