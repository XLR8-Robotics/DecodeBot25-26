package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Master tuning menu that provides information about all available tuning OpModes.
 * This OpMode serves as a guide and reference for the complete tuning suite.
 * 
 * Available Tuning OpModes:
 * 1. 🎯 Shooting Tuning - Distance-based shooting parameters (RPM/Power + Hood)
 * 2. 🎯 Turret Aiming Tuning - PID gains for turret aiming system
 * 3. 🚀 Launch Sequence Tuning - Timing parameters for launch sequence
 * 4. ⚙️ Shooter RPM Tuning - Velocity PID gains for RPM control
 */
@TeleOp(name = "📋 Tuning Menu & Guide", group = "Tuning")
public class TuningMenuOpMode extends LinearOpMode {
    
    private ElapsedTime displayTimer = new ElapsedTime();
    private int currentPage = 0;
    private final int totalPages = 5; // Overview + 4 tuning modes
    
    @Override
    public void runOpMode() {
        displayTimer.reset();
        
        telemetry.addData("📋", "TUNING SYSTEM GUIDE");
        telemetry.addData("Status", "Ready - Select from available tuning OpModes");
        telemetry.addData("Info", "This guide explains each tuning system");
        telemetry.addLine();
        telemetry.addData("Controls", "DPAD UP/DOWN to navigate pages");
        telemetry.addData("", "START to exit and select an OpMode");
        telemetry.update();

        waitForStart();
        
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        while (opModeIsActive()) {
            // Handle page navigation
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            
            if (currentDpadUp && !previousDpadUp) {
                currentPage = (currentPage - 1 + totalPages) % totalPages;
                displayTimer.reset();
            }
            
            if (currentDpadDown && !previousDpadDown) {
                currentPage = (currentPage + 1) % totalPages;
                displayTimer.reset();
            }
            
            previousDpadUp = currentDpadUp;
            previousDpadDown = currentDpadDown;
            
            // Display current page
            displayCurrentPage();
            
            telemetry.update();
            sleep(100); // Prevent excessive updates
        }
    }
    
    private void displayCurrentPage() {
        telemetry.clear();
        
        switch (currentPage) {
            case 0:
                displayOverviewPage();
                break;
            case 1:
                displayShootingTuningPage();
                break;
            case 2:
                displayTurretAimingTuningPage();
                break;
            case 3:
                displayLaunchSequenceTuningPage();
                break;
            case 4:
                displayShooterRPMTuningPage();
                break;
        }
        
        // Navigation footer
        telemetry.addLine();
        telemetry.addData("📖", String.format("Page %d/%d", currentPage + 1, totalPages));
        telemetry.addData("Navigation", "DPAD ↑↓ | START to exit");
    }
    
    private void displayOverviewPage() {
        telemetry.addData("📋", "TUNING SYSTEM OVERVIEW");
        telemetry.addLine();
        
        telemetry.addData("🎯 Purpose", "Real-time parameter tuning for competitive performance");
        telemetry.addLine();
        
        telemetry.addData("📊", "AVAILABLE TUNING OPMODES:");
        telemetry.addData("1️⃣", "🎯 Shooting Tuning");
        telemetry.addData("", "   Distance-based RPM/Power & Hood positions");
        telemetry.addData("2️⃣", "🎯 Turret Aiming Tuning");
        telemetry.addData("", "   PID gains for accurate target tracking");
        telemetry.addData("3️⃣", "🚀 Launch Sequence Tuning");
        telemetry.addData("", "   Timing parameters for automated sequences");
        telemetry.addData("4️⃣", "⚙️ Shooter RPM Tuning");
        telemetry.addData("", "   Velocity PID for consistent motor speed");
        telemetry.addLine();
        
        telemetry.addData("💾", "WORKFLOW:");
        telemetry.addData("Step 1", "Select appropriate tuning OpMode");
        telemetry.addData("Step 2", "Adjust parameters in real-time");
        telemetry.addData("Step 3", "Test and observe performance");
        telemetry.addData("Step 4", "Save tuned values to file");
        telemetry.addData("Step 5", "Copy generated code to Constants.java");
    }
    
    private void displayShootingTuningPage() {
        telemetry.addData("🎯", "SHOOTING TUNING");
        telemetry.addLine();
        
        telemetry.addData("📋 Purpose", "Tune distance-based shooting parameters");
        telemetry.addData("🎯 Target", "Consistent shots at all distances (12\" to 120\")");
        telemetry.addLine();
        
        telemetry.addData("⚙️", "TUNABLE PARAMETERS:");
        telemetry.addData("• RPM Values", "Motor speed for each distance breakpoint");
        telemetry.addData("• Hood Positions", "Servo positions for trajectory arc");
        telemetry.addData("• Control Mode", "Toggle between RPM and Power control");
        telemetry.addLine();
        
        telemetry.addData("🎮", "KEY CONTROLS:");
        telemetry.addData("DPAD ↑↓", "Select distance breakpoint (9 total)");
        telemetry.addData("Left Stick", "Adjust RPM (±20 RPM per unit)");
        telemetry.addData("Right Stick", "Adjust hood (±0.005 per unit)");
        telemetry.addData("A Button", "Test shoot with current parameters");
        telemetry.addData("Y Button", "Toggle RPM/Power control mode");
        telemetry.addLine();
        
        telemetry.addData("📊", "TUNING PROCESS:");
        telemetry.addData("1", "Drive to known distance from target");
        telemetry.addData("2", "Select corresponding breakpoint");
        telemetry.addData("3", "Adjust RPM and hood until shots hit");
        telemetry.addData("4", "Repeat for all 9 distance breakpoints");
        telemetry.addData("5", "Save and copy to Constants.java");
    }
    
    private void displayTurretAimingTuningPage() {
        telemetry.addData("🎯", "TURRET AIMING TUNING");
        telemetry.addLine();
        
        telemetry.addData("📋 Purpose", "Tune PID gains for turret aiming accuracy");
        telemetry.addData("🎯 Target", "Fast, accurate, stable target tracking");
        telemetry.addLine();
        
        telemetry.addData("⚙️", "TUNABLE PARAMETERS:");
        telemetry.addData("• KP (Proportional)", "Response speed (0.001-1.0)");
        telemetry.addData("• KI (Integral)", "Steady-state error elimination");
        telemetry.addData("• KD (Derivative)", "Overshoot and oscillation control");
        telemetry.addData("• Oscillation Settings", "Search behavior when no target");
        telemetry.addLine();
        
        telemetry.addData("🎮", "KEY CONTROLS:");
        telemetry.addData("DPAD ←→", "Select PID parameter to tune");
        telemetry.addData("Left Stick", "Adjust selected parameter");
        telemetry.addData("A Button", "Toggle auto-aiming on/off");
        telemetry.addData("Y Button", "Switch between Red/Blue targets");
        telemetry.addLine();
        
        telemetry.addData("📊", "TUNING TIPS:");
        telemetry.addData("• Start with KP", "Increase until fast but stable");
        telemetry.addData("• Add KI sparingly", "Only if steady-state error exists");
        telemetry.addData("• Use KD carefully", "Reduces overshoot but can cause noise");
        telemetry.addData("• Test at various", "Distances and angles to AprilTag");
        
        telemetry.addData("🔍", "Performance tracking shows settling time and accuracy");
    }
    
    private void displayLaunchSequenceTuningPage() {
        telemetry.addData("🚀", "LAUNCH SEQUENCE TUNING");
        telemetry.addLine();
        
        telemetry.addData("📋 Purpose", "Tune timing parameters for launch sequence");
        telemetry.addData("🎯 Target", "Reliable, consistent automated launching");
        telemetry.addLine();
        
        telemetry.addData("⚙️", "TUNABLE PARAMETERS:");
        telemetry.addData("• Shooter Spin-Up", "Time for shooter to reach speed (0.5-10s)");
        telemetry.addData("• Intake Reverse", "Reverse time when cancelled (0.2-5s)");
        telemetry.addData("• Lift Hold", "Time to hold lift servo position (0.1-3s)");
        telemetry.addData("• Button Timeouts", "Single/triple press detection windows");
        telemetry.addLine();
        
        telemetry.addData("🎮", "KEY CONTROLS:");
        telemetry.addData("DPAD ↑↓", "Select timing parameter (5 total)");
        telemetry.addData("Left Stick", "Adjust timing (various increments)");
        telemetry.addData("A Button", "Start test sequence");
        telemetry.addData("Cross (×)", "Standard launch controls (single/triple)");
        telemetry.addLine();
        
        telemetry.addData("📊", "SEQUENCE TRACKING:");
        telemetry.addData("• State Timing", "How long spent in each state");
        telemetry.addData("• Transition Log", "Complete sequence history");
        telemetry.addData("• Performance", "Success rate and consistency");
        
        telemetry.addData("💡", "Test with actual game pieces for realistic timing");
    }
    
    private void displayShooterRPMTuningPage() {
        telemetry.addData("⚙️", "SHOOTER RPM TUNING");
        telemetry.addLine();
        
        telemetry.addData("📋 Purpose", "Tune velocity PID for consistent RPM control");
        telemetry.addData("🎯 Target", "Accurate, stable motor speed regardless of load");
        telemetry.addLine();
        
        telemetry.addData("⚙️", "TUNABLE PARAMETERS:");
        telemetry.addData("• KP (Proportional)", "Response to RPM error (1-50)");
        telemetry.addData("• KI (Integral)", "Eliminates steady-state error (0-5)");
        telemetry.addData("• KD (Derivative)", "Dampens oscillation (0-5)");
        telemetry.addData("• KF (Feed-Forward)", "Motor-specific gain (~12-15)");
        telemetry.addData("• Ticks/Rev", "Motor encoder specification");
        telemetry.addLine();
        
        telemetry.addData("🎮", "KEY CONTROLS:");
        telemetry.addData("DPAD ←→", "Select PID parameter to tune");
        telemetry.addData("Left Stick", "Adjust selected parameter");
        telemetry.addData("Right Stick", "Adjust target RPM (50 RPM steps)");
        telemetry.addData("A Button", "Toggle shooter on/off");
        telemetry.addData("Y Button", "Cycle through preset test RPMs");
        telemetry.addLine();
        
        telemetry.addData("📊", "PERFORMANCE METRICS:");
        telemetry.addData("• RPM Error", "Difference from target");
        telemetry.addData("• Settling Time", "Time to reach stable speed");
        telemetry.addData("• Stability", "Consistency of RPM control");
        
        telemetry.addData("💡", "Start with KF (~12), then tune KP for responsiveness");
    }
}
