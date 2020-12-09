## Changelog
- > The following attributes aren’t available: machine, respawn, respawn_delay, clear_params under node
- nodelet -> ros2 component
## Best Practice
- there is no `condition` in `ComposableNode`, but you can add that in `ComposableNodeContainer` or use `LoadComposableNode` action with condition.
- `LaunchConfiguration` cannot be converted into str directly. so if you want to concatenate it to others you can try:
  ```
  some_param=['some string', LaunchConfiguration('arg')]
  ```
- `ComposableNodeContainer` **must** specify a name and namespace.
- `extra_arguments` in `ComposableNode` is used to specify the arguments when container load it. so like `parameters`, it will take a single string as a file path to open. if you want to add `--no-daemon`, you cannot push this string into `extra_arguments` directly.