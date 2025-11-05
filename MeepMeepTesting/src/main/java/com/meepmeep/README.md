# MeepMeep Visualization

MeepMeep is a visualization tool for RoadRunner path planning. It allows you to visualize and test trajectories before running them on your robot.

## Setup

MeepMeep has been added to your project as a separate module. The main visualization class is `MeepMeepTesting.java`.

## Running MeepMeep

1. **In Android Studio:**
   - Right-click on `MeepMeepTesting.java`
   - Select "Run 'MeepMeepTesting.main()'"
   - A window will open showing the trajectory visualization

2. **Alternative method:**
   - Go to Run → Edit Configurations
   - Click "+" and select "Application"
   - Set the module to `MeepMeepTesting`
   - Set the main class to `com.meepmeep.MeepMeepTesting`
   - Run the configuration

## Customizing Trajectories

Edit the trajectory in `MeepMeepTesting.java`. The example shows a square pattern, but you can modify it to match your autonomous paths.

Common RoadRunner actions:
- `lineToX(double x)` - Drive to a specific x coordinate
- `lineToY(double y)` - Drive to a specific y coordinate
- `lineToLinearHeading(Vector2d end, double heading)` - Drive to a position with a specific heading
- `turn(double angle)` - Turn in place by a specific angle (in radians)
- `strafeLeft(double distance)` - Strafe left
- `strafeRight(double distance)` - Strafe right
- `forward(double distance)` - Drive forward
- `back(double distance)` - Drive backward

## Tuning Constants

Make sure to update `DriveConstants.java` in this module with your robot's actual specifications:
- `MAX_VEL` - Maximum translational velocity (in/s)
- `MAX_ACCEL` - Maximum translational acceleration (in/s²)
- `MAX_ANG_VEL` - Maximum angular velocity (rad/s)
- `MAX_ANG_ACCEL` - Maximum angular acceleration (rad/s²)
- `TRACK_WIDTH` - Distance between wheels (in)

**Important:** Keep these values in sync with the DriveConstants in the TeamCode module for consistency.

These values should match your robot's actual capabilities for accurate visualization.

## Field Backgrounds

You can change the field background in `MeepMeepTesting.java`:
- `FIELD_FREIGHT_FRENZY`
- `FIELD_POWERPLAY_OFFICIAL`
- `FIELD_FREIGHTFRENZY_ADI_DARK`
- And more...

For more information, visit: https://learnroadrunner.com/tool/meepmeep

