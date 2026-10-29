# EasyFly launch lifecycle

Use `easyfly_launch.composition.ComposableNodeContainer` in project launch files.
The container process, plugin descriptions, conditions, scoped parameters and
remappings remain the standard ROS interfaces. Only loading is changed.

Humble's threaded loader can call a destroyed client after launch shutdown,
including scheduling the next component after cancelling the previous wait.
This loader polls asynchronously on the same event loop as adapter shutdown.
It checks shutdown before each ROS operation, sends each load request once,
never starts a subsequent load after cancellation, and leaves adapter shutdown
to destroy outstanding clients. A rejected load or live transport error still
fails launch. There is no catch-all exception suppression or load retry.

The tests use real ROS services and LaunchService to cover missing services,
shutdown while a response is pending, completion, rejection and frozen scoped
parameters. They do not start Gazebo, PX4 or issue flight commands.

The component loader owns one dedicated ROS adapter per launch context. Its spin
thread shuts down the executor before the adapter joins that thread and destroys
the node and context. This also releases Humble signal guards explicitly, instead
of leaving their destructor to run after context shutdown. Other ROS adapters are
not modified. The real-service tests check guard release and context shutdown.
