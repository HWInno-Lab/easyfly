# Backend registry

Static metadata and strict configuration validation. This package starts no algorithms and grants no runtime readiness or flight admission.

- `easyfly_backend_registry.Registry`: IDs, defaults, role-specific schemas, layer provenance, reviewed legacy importers, input requirements and entrypoint checks.
- `easyfly_backend_registry/metadata.hpp`: generated C++ lookup and validators for the existing strong SORT/A* parameter structures. Its exported CMake target has no ROS or Python runtime dependency.
- `backend_registry list|validate|check-source`: read-only CLI.

`data/backends.json` is authoritative for registered implementations; UI labels/platform schemas remain data in this package. CMake generates the installed UI options from this registration. Registering a name does not implement its creator or launch route. Adapter tests enumerate registered C++ implementations; Python checks validate referenced packages/launch files/exported declarations.

Python validation requires `python3-jsonschema` (Draft 2020-12). Grid roles `global_grid` and `combined_grid` have separate defaults. The full tracking YAML importer preserves the historical loader fallback values when fields were absent; the shipped profile defaults remain 0.15 / 45 / 3 / 100 / 1.5.

See the [integration guide](../../algorithms/docs/integration-guide.md) for factory registration, configuration routing and acceptance.

Python and CMake share one UI generation path shared by Python and CMake. Computation creators can attach a normalizer to `Factory::add`; `resolve` reports the same validated parameters consumed by `create`. Integer JSON values are canonicalized for the C++ binding, including its int32 SORT limits. Runtime observation remains outside this metadata package. See the [integration guide](../../algorithms/docs/integration-guide.md).
