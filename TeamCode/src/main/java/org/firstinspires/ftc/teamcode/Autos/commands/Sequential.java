package org.firstinspires.ftc.teamcode.Autos.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Runs commands one after another.
 */
public class Sequential implements AutoCommand {
    private final List<AutoCommand> commands = new ArrayList<>();
    private int index;

    public Sequential(AutoCommand... cmds) {
        commands.addAll(Arrays.asList(cmds));
    }

    @Override
    public void init(Robot robot) {
        index = 0;
        if (!commands.isEmpty()) commands.get(0).init(robot);
    }

    @Override
    public void execute(Robot robot) {
        if (index >= commands.size()) return;
        AutoCommand current = commands.get(index);
        current.execute(robot);
        if (current.isFinished(robot)) {
            current.end(robot);
            index++;
            if (index < commands.size()) commands.get(index).init(robot);
        }
    }

    @Override
    public boolean isFinished(Robot robot) {
        return index >= commands.size();
    }
}


