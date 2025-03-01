package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class L3 extends SequentialCommandGroup {
    public L3(Elevator elevator, Arm arm) {
        addCommands(
            new ChangeAngle(arm, ElevatorConstants.kArmTravel),
            new ChangeHeight(elevator, ElevatorConstants.kL3Height),
            new ChangeAngle(arm, ElevatorConstants.kL2_3Arm)
        );
    }
}