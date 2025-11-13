package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Utility class for saving and managing tuned parameters.
 * Generates code that can be copy-pasted into Constants.java for easy integration.
 */
public class TuningUtils {
    
    private static final String TUNING_DIRECTORY = "/sdcard/FIRST/tuning/";
    
    /**
     * Saves shooting parameters to a file with generated Constants.java code.
     */
    public void saveShootingParameters(String filename, double[] distances, double[] rpmValues, 
                                     double[] hoodValues, boolean usingRPM) {
        try {
            FileWriter writer = new FileWriter(TUNING_DIRECTORY + filename);
            
            // Header with timestamp and metadata
            writer.write(generateHeader("SHOOTING PARAMETERS", usingRPM ? "RPM Control" : "Power Control"));
            
            // Generate Constants.java compatible code
            writer.write("// Copy this code to Constants.AutoShootingConfig:\n\n");
            
            // Distance breakpoints
            writer.write("public static final double[] DISTANCE_BREAKPOINTS = {\n");
            for (int i = 0; i < distances.length; i++) {
                writer.write(String.format("    %.1f%s   // %s shots\n", 
                    distances[i], 
                    (i < distances.length - 1) ? "," : "",
                    getDistanceDescription(distances[i])));
            }
            writer.write("};\n\n");
            
            // RPM/Power values
            if (usingRPM) {
                writer.write("public static final double[] SHOOTER_RPM_VALUES = {\n");
                for (int i = 0; i < rpmValues.length; i++) {
                    writer.write(String.format("    %.0f%s   // %.1f\" - %s\n", 
                        rpmValues[i], 
                        (i < rpmValues.length - 1) ? "," : "",
                        distances[i],
                        getDistanceDescription(distances[i])));
                }
                writer.write("};\n\n");
            } else {
                writer.write("public static final double[] SHOOTER_POWER_VALUES = {\n");
                for (int i = 0; i < rpmValues.length; i++) {
                    double powerValue = rpmValues[i] / 5000.0; // Convert RPM to power approximation
                    writer.write(String.format("    %.2f%s   // %.1f\" - %s\n", 
                        powerValue, 
                        (i < rpmValues.length - 1) ? "," : "",
                        distances[i],
                        getDistanceDescription(distances[i])));
                }
                writer.write("};\n\n");
            }
            
            // Hood position values
            writer.write("public static final double[] HOOD_POSITION_VALUES = {\n");
            for (int i = 0; i < hoodValues.length; i++) {
                writer.write(String.format("    %.3f%s   // %.1f\" - %s\n", 
                    hoodValues[i], 
                    (i < hoodValues.length - 1) ? "," : "",
                    distances[i],
                    getDistanceDescription(distances[i])));
            }
            writer.write("};\n\n");
            
            // Additional configuration
            writer.write("// Control mode setting\n");
            writer.write(String.format("public static final boolean USE_RPM_CONTROL = %s;\n\n", usingRPM));
            
            // Summary data
            writer.write(generateSummaryData("SHOOTING", distances.length));
            
            writer.close();
            
        } catch (IOException e) {
            // Silently fail - telemetry will show if save was successful
        }
    }
    
    /**
     * Saves turret aiming PID parameters to a file.
     */
    public void saveTurretAimingParameters(String filename, double kp, double ki, double kd, 
                                         double oscillationSpeed, double oscillationPeriod) {
        try {
            FileWriter writer = new FileWriter(TUNING_DIRECTORY + filename);
            
            writer.write(generateHeader("TURRET AIMING PARAMETERS", "PID Control"));
            
            writer.write("// Copy this code to Constants.TurretAimingConfig:\n\n");
            
            writer.write(String.format("public static final double AIMING_KP = %.4f;\n", kp));
            writer.write(String.format("public static final double AIMING_KI = %.4f;\n", ki));
            writer.write(String.format("public static final double AIMING_KD = %.4f;\n", kd));
            writer.write("\n");
            writer.write(String.format("public static final double OSCILLATION_SPEED = %.2f;\n", oscillationSpeed));
            writer.write(String.format("public static final double OSCILLATION_PERIOD_MS = %.1f;\n", oscillationPeriod));
            
            // Tuning notes
            writer.write("\n// TUNING NOTES:\n");
            writer.write("// KP: Increase for faster response, decrease if oscillating\n");
            writer.write("// KI: Increase to eliminate steady-state error\n");
            writer.write("// KD: Increase to reduce overshoot and oscillation\n");
            
            writer.write(generateSummaryData("TURRET_AIMING", 5));
            
            writer.close();
            
        } catch (IOException e) {
            // Silently fail
        }
    }
    
    /**
     * Saves launch sequence timing parameters to a file.
     */
    public void saveLaunchSequenceParameters(String filename, long spinUpTime, long reverseTime, 
                                           long liftHoldTime, long singlePressTimeout, long triplePressTimeout) {
        try {
            FileWriter writer = new FileWriter(TUNING_DIRECTORY + filename);
            
            writer.write(generateHeader("LAUNCH SEQUENCE PARAMETERS", "Timing Control"));
            
            writer.write("// Copy this code to Constants.LaunchSequenceConfig:\n\n");
            
            writer.write(String.format("public static final long SHOOTER_SPIN_UP_TIME_MS = %dL; // %.1f seconds\n", 
                spinUpTime, spinUpTime / 1000.0));
            writer.write(String.format("public static final long INTAKE_REVERSE_TIME_MS = %dL; // %.1f seconds\n", 
                reverseTime, reverseTime / 1000.0));
            writer.write(String.format("public static final long LIFT_SERVO_HOLD_TIME_MS = %dL; // %.1f seconds\n", 
                liftHoldTime, liftHoldTime / 1000.0));
            writer.write(String.format("public static final long SINGLE_PRESS_TIMEOUT_MS = %dL; // %.1f seconds\n", 
                singlePressTimeout, singlePressTimeout / 1000.0));
            writer.write(String.format("public static final long TRIPLE_PRESS_TIMEOUT_MS = %dL; // %.1f seconds\n", 
                triplePressTimeout, triplePressTimeout / 1000.0));
            
            // Timing notes
            writer.write("\n// TIMING NOTES:\n");
            writer.write("// SPIN_UP_TIME: Time for shooter to reach target speed\n");
            writer.write("// REVERSE_TIME: Time to reverse intake when cancelled\n");
            writer.write("// LIFT_HOLD_TIME: Time to hold lift servo in position\n");
            writer.write("// PRESS_TIMEOUTS: Button press detection windows\n");
            
            writer.write(generateSummaryData("LAUNCH_SEQUENCE", 5));
            
            writer.close();
            
        } catch (IOException e) {
            // Silently fail
        }
    }
    
    /**
     * Saves shooter RPM control PID parameters to a file.
     */
    public void saveShooterRPMParameters(String filename, double velocityKp, double velocityKi, 
                                       double velocityKd, double velocityKf, double ticksPerRev) {
        try {
            FileWriter writer = new FileWriter(TUNING_DIRECTORY + filename);
            
            writer.write(generateHeader("SHOOTER RPM CONTROL PARAMETERS", "Velocity PID"));
            
            writer.write("// Copy this code to Constants.AutoShootingConfig:\n\n");
            
            writer.write(String.format("public static final double VELOCITY_KP = %.2f;\n", velocityKp));
            writer.write(String.format("public static final double VELOCITY_KI = %.2f;\n", velocityKi));
            writer.write(String.format("public static final double VELOCITY_KD = %.2f;\n", velocityKd));
            writer.write(String.format("public static final double VELOCITY_KF = %.2f;\n", velocityKf));
            writer.write("\n");
            writer.write(String.format("public static final double SHOOTER_MOTOR_TICKS_PER_REV = %.1f;\n", ticksPerRev));
            
            // RPM PID tuning notes
            writer.write("\n// RPM PID TUNING NOTES:\n");
            writer.write("// KP: Proportional gain - increase for faster response\n");
            writer.write("// KI: Integral gain - increase to eliminate steady-state error\n");
            writer.write("// KD: Derivative gain - increase to reduce overshoot\n");
            writer.write("// KF: Feed-forward gain - motor-specific, usually 12-15 for goBILDA motors\n");
            writer.write("// TICKS_PER_REV: Motor encoder specification (e.g., 537.7 for goBILDA 5202)\n");
            
            writer.write(generateSummaryData("SHOOTER_RPM", 5));
            
            writer.close();
            
        } catch (IOException e) {
            // Silently fail
        }
    }
    
    // =================================================================================
    // HELPER METHODS
    // =================================================================================
    
    private String generateHeader(String systemName, String controlType) {
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
        String timestamp = dateFormat.format(new Date());
        
        StringBuilder header = new StringBuilder();
        header.append("// ").append("=".repeat(70)).append("\n");
        header.append("// ").append(systemName).append(" - TUNED PARAMETERS\n");
        header.append("// ").append("=".repeat(70)).append("\n");
        header.append("// Generated: ").append(timestamp).append("\n");
        header.append("// Control Type: ").append(controlType).append("\n");
        header.append("// Robot: DECODE Season 2025-2026\n");
        header.append("// \n");
        header.append("// INSTRUCTIONS:\n");
        header.append("// 1. Copy the code blocks below\n");
        header.append("// 2. Replace the corresponding sections in Constants.java\n");
        header.append("// 3. Test the new values in your regular OpModes\n");
        header.append("// 4. Make further adjustments if needed\n");
        header.append("// ").append("=".repeat(70)).append("\n\n");
        
        return header.toString();
    }
    
    private String generateSummaryData(String systemType, int parameterCount) {
        StringBuilder summary = new StringBuilder();
        summary.append("\n\n// ").append("=".repeat(50)).append("\n");
        summary.append("// TUNING SUMMARY\n");
        summary.append("// ").append("=".repeat(50)).append("\n");
        summary.append("// System: ").append(systemType).append("\n");
        summary.append("// Parameters Tuned: ").append(parameterCount).append("\n");
        summary.append("// Tuning Session: ").append(new SimpleDateFormat("MMM dd, yyyy", Locale.getDefault()).format(new Date())).append("\n");
        summary.append("// \n");
        summary.append("// Remember to:\n");
        summary.append("// - Test these values in competition conditions\n");
        summary.append("// - Document any field-specific adjustments needed\n");
        summary.append("// - Keep backup copies of working configurations\n");
        summary.append("// ").append("=".repeat(50)).append("\n");
        
        return summary.toString();
    }
    
    private String getDistanceDescription(double distance) {
        if (distance <= 18) return "Very close";
        if (distance <= 30) return "Close";
        if (distance <= 45) return "Medium-close";
        if (distance <= 60) return "Medium";
        if (distance <= 75) return "Medium-far";
        if (distance <= 90) return "Far";
        if (distance <= 105) return "Very far";
        return "Maximum range";
    }
}
