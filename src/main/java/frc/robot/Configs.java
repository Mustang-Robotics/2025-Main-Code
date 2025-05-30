package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Intake {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        static {
                intakeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .inverted(true);
        }
    }

    public static final class Elevator {
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        static {
                elevatorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .inverted(true)
                .voltageCompensation(12);

                elevatorConfig
                .encoder
                .positionConversionFactor(100)
                .velocityConversionFactor(100);

                elevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.0035)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(500000)
                .maxAcceleration(500000)
                .allowedClosedLoopError(10);

        }
    }
    public static final class Arm {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {
                armConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .inverted(true)
                .voltageCompensation(12);

                armConfig
                .absoluteEncoder
                .positionConversionFactor(360)
                .velocityConversionFactor(360);

                armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(.0005)
                .outputRange(-.5, .5)
                .positionWrappingEnabled(true)
                .maxMotion
                .maxVelocity(1000000)
                .maxAcceleration(500000)
                .allowedClosedLoopError(2);

        }
    }

    public static final class Climb {
        public static final SparkMaxConfig climbConfig = new SparkMaxConfig();

        static {
                climbConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .inverted(true);
        }
    }
}
