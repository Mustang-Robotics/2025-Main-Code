package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class FollowThenScore extends SequentialCommandGroup{
    public FollowThenScore(Command path, Intake intake, LED led){
        addCommands(
            new LEDSolidRed(led),
            path,
            new IntakeSetSpeed(intake, .25),
            new WaitCommand(.6),
            new IntakeSetSpeed(intake, 0),
            new LEDSolidGreen(led)
        );
    }
    
}
