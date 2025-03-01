package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GripCoral extends SequentialCommandGroup {
    public GripCoral(Intake intake, Arm arm) {
        addCommands(
            new ChangeAngle(arm, Constants.ElevatorConstants.kArmTravel),
            new IntakeSpeed(intake, .3),
            new WaitCommand(.065),
            new IntakeSpeed(intake, 0)
        );
    }
}
