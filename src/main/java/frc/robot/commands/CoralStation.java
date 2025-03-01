package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;


public class CoralStation extends SequentialCommandGroup {
    public CoralStation(Elevator elevator, Arm arm) {
        addCommands(
            new ChangeAngle(arm, ElevatorConstants.kArmTravel),
            new ChangeHeight(elevator, ElevatorConstants.kCorralStation),
            new ChangeAngle(arm, ElevatorConstants.kCorralStationArm)
        );
    }
}