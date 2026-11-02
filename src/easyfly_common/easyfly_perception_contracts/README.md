# Perception contracts

V2-native validation, UUID/session/epoch admission, int64 identity, bounded producer-owned int32 aliases and one-way numeric compatibility views. Pixel velocity units never imply world velocity. A V1 numeric view cannot manufacture V2 evidence.

`WorldInput` admits a unique actual DDS writer, verifies camera/time/reference/geometry and retains the native envelope through its consumer callback. A source change invalidates related work before replacement data is accepted. Alias history is bounded and never evicted to permit identity reuse.

ROS Humble's ordinary rclpy callback omits publisher GID. `NativeSubscription` therefore uses a bounded read-only C++ intake, serializes the original message plus verified GID, then deserializes it on the Python node's steady timer. Metadata queues hold at most four 4 MiB samples; original-image intake holds only the latest sample, at most 8 MiB. It starts no algorithm, publishes no ROS messages and owns no control interface. Node destruction closes the intake context and thread. Build dependency: pybind11 headers; runtime dependencies: ROS messages/rclcpp/rclpy.

Native streams and services are separate from V1. All producers/consumers default to explicit compatibility configuration `perception_version=1`; use version 2 consistently across a pipeline. V2 producer compatibility publishing defaults off. Source changes terminate active target bindings; existing cancellation/ownership release protocols remain in their owning nodes.

See the [perception contract](../../algorithms/docs/extension-contracts.md) for versioned fields, identity and compatibility rules.

`backend_status.hpp` and the optional Python `backend_status.BackendMonitor` provide read-only lifecycle diagnostics on `/platform/backends/<role>`, using `diagnostic_msgs/DiagnosticArray`. The intake also accepts DiagnosticArray and verifies the actual DDS writer. These reports distinguish effective selection, creation, provider, input readiness and expiry; they grant no task or flight permission. The Python monitor uses the common backend registry. The complete versioned field contract and scope are in [integration guide](../../algorithms/docs/integration-guide.md).
