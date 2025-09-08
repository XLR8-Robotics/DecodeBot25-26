package org.firstinspires.ftc.teamcode.Autos.commands;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Simple autonomous command interface.
 */
public interface AutoCommand {
    /** Called once at command start. */
    void init(Robot robot);

    /** Called repeatedly until isFinished() is true. */
    void execute(Robot robot);

    /** True when the command has completed. */
    boolean isFinished(Robot robot);

    /** Called once after finishing (or when interrupted). */
    default void end(Robot robot) {}
}


