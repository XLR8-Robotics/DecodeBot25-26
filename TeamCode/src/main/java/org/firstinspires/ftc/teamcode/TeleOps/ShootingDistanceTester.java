package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Shooting Distance Tester", group = "TeleOp")
public class ShootingDistanceTester extends LinearOpMode {

    private Robot robot;
    
    // Manual control variables
    private double currentShooterRPM = 2000.0; // Starting RPM
    private double currentHoodPosition = 0.5; // Starting position (mid-range)
    
    // Increment values for fine-tuning
    private static final double RPM_INCREMENT = 50.0;
    private static final double HOOD_INCREMENT = 0.01;
    
    // Button state tracking for toggle behavior
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousCircle = false;
    
    // Lift Servo State
    private boolean isLiftUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot(hardwareMap);

        // Force shooter blocker to be open (unblocked) initially
        robot.turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Dpad Up/Down: RPM (+/- " + RPM_INCREMENT + ")");
        telemetry.addData("Controls", "Dpad Right/Left: Hood Position (+/- 0.01)");
        telemetry.addData("Controls", "Hold Cross: Run Shooter");
        telemetry.addData("Controls", "R2/L2: Intake In/Out");
        telemetry.addData("Controls", "Circle: Toggle Lift Servo");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Manual Control Logic ---
            
            // 1. Adjust Shooter RPM Target (Dpad Up/Down)
            if (gamepad1.dpad_up && !previousDpadUp) {
                currentShooterRPM += RPM_INCREMENT;
            } else if (gamepad1.dpad_down && !previousDpadDown) {
                currentShooterRPM -= RPM_INCREMENT;
            }
            
            // Clamp RPM between 0.0 and MAX
            currentShooterRPM = Math.max(0.0, Math.min(Constants.AutoShootingConfig.MAX_SHOOTER_RPM, currentShooterRPM));
            
            // 2. Adjust Hood Position Target (Dpad Right/Left)
            if (gamepad1.dpad_right && !previousDpadRight) {
                currentHoodPosition += HOOD_INCREMENT;
            } else if (gamepad1.dpad_left && !previousDpadLeft) {
                currentHoodPosition -= HOOD_INCREMENT;
            }
            
            // Clamp hood position between 0.0 and 1.0
            currentHoodPosition = Math.max(0.0, Math.min(1.0, currentHoodPosition));

            // 3. Apply Hood Position
            robot.shooter.setHoodPosition(currentHoodPosition);
            
            // 4. Run Shooter Motor (Hold Cross)
            if (gamepad1.cross) {
                robot.shooter.setRPM(currentShooterRPM);
            } else {
                robot.shooter.setRPM(0);
            }

            // 5. Intake Control (R2 = In, L2 = Out)
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.intake.setPower(intakePower * Constants.IntakeConfig.INTAKE_SPEED);

            // 6. Lift Servo Toggle (Circle)
            if (gamepad1.circle && !previousCircle) {
                isLiftUp = !isLiftUp;
                if (isLiftUp) {
                    robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
                } else {
                    robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
                }
            }

            // Ensure Shooter Blocker is ALWAYS open
            robot.turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
            
            // Update previous button states
            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;
            previousDpadLeft = gamepad1.dpad_left;
            previousDpadRight = gamepad1.dpad_right;
            previousCircle = gamepad1.circle;

            // 7. Drive Control
            double forward = -gamepad1.left_stick_y; 
            double strafe = gamepad1.left_stick_x;  
            double turn = gamepad1.right_stick_x;   
            robot.basicDriveTrain.drive(forward, strafe, turn);

            // Display Telemetry
            displayTelemetry();
        }
    }

    /**
     * Displays comprehensive telemetry data organized by category.
     */
    private void displayTelemetry() {
        telemetry.addData("=== SHOOTER CONTROLS ===", "");
        telemetry.addData("Target RPM", "%.0f", currentShooterRPM);
        telemetry.addData("Target Hood Pos", "%.2f", currentHoodPosition);
        telemetry.addLine();
        
        displayMotorAndServoTelemetry();
        displaySensorTelemetry();
        telemetry.update();
    }

    /**
     * Displays motor power and servo position telemetry.
     */
    private void displayMotorAndServoTelemetry() {
        telemetry.addData("=== MOTORS & SERVOS ===", "");
        telemetry.addData("Shooter RPM (Actual)", "%.0f", robot.shooter.getCurrentRPM());
        telemetry.addData("Shooter Power (Applied)", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position (Actual)", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Intake Power", "%.2f", robot.intake.getMotorPower());
        telemetry.addData("Lift Servo", isLiftUp ? "UP" : "DOWN");
        telemetry.addData("Blocker", "OPEN");
    }

    /**
     * Displays sensor readings and limit switch states.
     */
    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        
        // Limelight info is useful for distance testing
        if (robot.limelight != null) {
             telemetry.addData("Limelight Has Target", robot.limelight.hasTarget());
             if (robot.limelight.hasTarget()) {
                 // Assuming getDistance() or similar exists, otherwise just raw data
                 telemetry.addData("Limelight TY", "%.2f", robot.limelight.getTy()); 
             }
        }
    }
}
