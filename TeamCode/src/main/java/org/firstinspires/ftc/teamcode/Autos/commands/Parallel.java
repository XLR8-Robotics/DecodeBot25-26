package org.firstinspires.ftc.teamcode.Autos.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Runs commands in parallel; finishes when all finish.
 */
public class Parallel implements AutoCommand {
    private final List<AutoCommand> commands = new ArrayList<>();
    private boolean initialized;

    public Parallel(AutoCommand... cmds) {
        commands.addAll(Arrays.asList(cmds));
    }

    @Override
    public void init(Robot robot) {
        for (AutoCommand c : commands) c.init(robot);
        initialized = true;
    }

    @Override
    public void execute(Robot robot) {
        if (!initialized) init(robot);
        for (AutoCommand c : commands) c.execute(robot);
    }

    @Override
    public boolean isFinished(Robot robot) {
        for (AutoCommand c : commands) if (!c.isFinished(robot)) return false;
        return true;
    }
}


