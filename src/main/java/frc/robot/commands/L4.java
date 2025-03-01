package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class L4 extends SequentialCommandGroup {
    public L4(Elevator elevator, Arm arm) {
        addCommands(
            new ChangeAngle(arm, ElevatorConstants.kArmTravel),
            new ChangeHeight(elevator, ElevatorConstants.kL4Height),
            new ChangeAngle(arm, ElevatorConstants.kL4Arm)
        );
    }
}
