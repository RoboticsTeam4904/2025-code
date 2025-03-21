package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * Gets a new command from a factory every time it is initialized.
 */
public class CreateOnInitialize extends Command {

    private final Supplier<Command> commandDealer;
    private Command currentActiveCommand = null;

    public CreateOnInitialize(String name, Supplier<Command> commandDealer) {
        this.commandDealer = commandDealer;
        setName(name);
    }

    /**
     * Command that takes in a command factory and runs a new command from that
     * factory each time it is scheduled. This avoids the problems of command
     * factory code running on "construct" rather than on "initialize."
     *
     * @param commandDealer The Command factory
     */
    public CreateOnInitialize(Supplier<Command> commandDealer) {
        this("CreateOnInitialize", commandDealer);
    }

    @Override
    public void initialize() {
        currentActiveCommand = commandDealer.get();
        currentActiveCommand.initialize();

        this.getRequirements().clear();
        this.addRequirements(currentActiveCommand.getRequirements());

        setName("CreateOnInitialize: " + currentActiveCommand.getName());
    }

    @Override
    public void execute() {
        currentActiveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return currentActiveCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        currentActiveCommand.end(interrupted);
        currentActiveCommand = null;
    }
}
