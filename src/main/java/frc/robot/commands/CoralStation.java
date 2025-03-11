package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
<<<<<<< HEAD
import frc.robot.Constants.ElevatorHeights;
=======
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.Constants.ElevatorConstants;

>>>>>>> a2f230b (Basically Done)

public class CoralStation extends SequentialCommandGroup {
    public CoralStation(Elevator elevator, Arm arm, Intake intake, LED led) {
        addCommands(
<<<<<<< HEAD
            new ChangeAngle(arm, 50),
            new ChangeHeight(elevator, ElevatorHeights.kCorralStation),
            new ChangeAngle(arm, 20)
=======
            new LEDSolidBlue(led),
            new ChangeAngle(arm, ElevatorConstants.kArmTravel),
            new ChangeHeight(elevator, ElevatorConstants.kCorralStation),
            new ChangeAngle(arm, ElevatorConstants.kCorralStationArm),
            new IntakeCurrentStop(intake, led, 0.2),
            new LEDSolidGreen(led),
            new IntakeSetSpeed(intake, -.15),
            new WaitCommand(.15),
            new IntakeSetSpeed(intake, 0)
            
>>>>>>> a2f230b (Basically Done)
        );
    }
}