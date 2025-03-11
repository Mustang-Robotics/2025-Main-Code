package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class RemoveAlgae extends SequentialCommandGroup {
    public RemoveAlgae(DriveSubsystem drive, Arm arm, Intake intake){
        addCommands(
            new ChangeAngle(arm, ElevatorConstants.kAlgae),
            new IntakeSetSpeed(intake, .4),
            new AlgaeDrive(drive, .15),
            new WaitCommand(.75),
            new AlgaeDrive(drive, 0),
            new IntakeSetSpeed(intake, 0)
        );
    }
}
