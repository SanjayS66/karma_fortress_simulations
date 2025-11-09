# Simplified Command Flow - Direct Remapping

## Why This Is Better

Instead of using multiple relay nodes, we directly remap Nav2's velocity output to the controller's input topic using the nav2_params.yaml configuration.

## New Command Flow

```
┌─ Teleop (if used) ─────────────────────────┐
│                                             │
│  teleop_twist_keyboard publishes Twist    │
│  to /cmd_vel (unstamped)                  │
│              ↓                             │
│  [cmd_vel_relay]                          │
│  (simple bridge)                          │
│              ↓                             │
│  /diff_cont/cmd_vel_unstamped             │
│         (to controller)                   │
└─────────────────────────────────────────────┘

┌─ Nav2 Navigation ──────────────────────────┐
│                                             │
│  velocity_smoother publishes Twist        │
│  (normally to /cmd_vel_nav)               │
│              ↓                             │
│  [Remapping in nav2_params.yaml]          │
│  cmd_vel → /diff_cont/cmd_vel_unstamped  │
│              ↓                             │
│  /diff_cont/cmd_vel_unstamped             │
│         (to controller)                   │
└─────────────────────────────────────────────┘

         ↓↓↓ SAME TOPIC ↓↓↓

[diff_drive_controller]
  (subscribes to /diff_cont/cmd_vel_unstamped)
        ↓
  Gazebo bridge
        ↓
  Wheels spin! 🎯
```

## Configuration

### 1. `config/nav2_params.yaml` - Added Remapping

```yaml
velocity_smoother:
  ros__parameters:
    use_sim_time: True
    # ... other params ...
    remappings:
      cmd_vel: "/diff_cont/cmd_vel_unstamped"
```

This tells Nav2's velocity_smoother to publish directly to `/diff_cont/cmd_vel_unstamped` instead of `/cmd_vel_nav`.

### 2. `rcup_garden/cmd_vel_relay.py` - Still Used for Teleop

```python
# Subscribe to /cmd_vel (from teleop_twist_keyboard)
self.subscription = self.create_subscription(Twist, '/cmd_vel', ...)

# Publish to /diff_cont/cmd_vel_unstamped (what controller expects)
self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', ...)
```

### 3. `launch/nav2.launch.py` - Removed Intermediate Relay

- ✅ Removed `cmd_vel_nav_relay` node (no longer needed)
- ✅ Removed relay from launch description
- ✅ Simplified to just: gazebo → rviz → delayed_localization → delayed_navigation

### 4. `config/diff_drive_controllers.yaml` - Already Correct

```yaml
diff_cont:
  ros__parameters:
    use_stamped_vel: false  # ← Expects unstamped messages
    # Messages arrive at /diff_cont/cmd_vel_unstamped
```

## Key Settings

| Setting | Value | Meaning |
|---------|-------|---------|
| `use_stamped_vel` | `false` | Expects geometry_msgs/Twist (not geometry_msgs/TwistStamped) |
| `cmd_vel_timeout` | 0.5 | Stops wheels if no command in 500ms |

## Result

✅ **Simpler**: No intermediate relay node needed for Nav2  
✅ **Direct**: Nav2 publishes directly to controller input  
✅ **Same behavior**: Both teleop and Nav2 use same topic  
✅ **Fewer topics**: Cleaner rostopic list  
✅ **Faster**: Less message copying overhead  

## Test It

```bash
# Rebuild (picks up nav2_params changes)
colcon build --symlink-install

# Launch
ros2 launch rcup_garden nav2.launch.py

# In RViz:
# 1. Set initial pose (2D Pose Estimate)
# 2. Set goal (Nav2 Goal)
# 3. Watch bot move to goal! 🎯
```

## No Changes Needed

The `cmd_vel_nav_relay.py` file can remain in the package but won't be used or launched.
The `cmd_vel_relay.py` is still used for teleop bridging and stays as is.
