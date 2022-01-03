# arm_action_executor
package to take a string of commands for a robot arm and execute them sequentially. listens to /arm_cmd topic and calls /arm_movement action with a Pose

can be used with mqtt bridge to receive commands and publish robot state to mqtt broker
