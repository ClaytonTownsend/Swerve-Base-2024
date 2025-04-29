package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    // Create Swerve Modules
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveCanId,
        DriveConstants.frontLeftTurnCanId,
        DriveConstants.frontLeftCANcoderID,
        DriveConstants.kFrontLeftChassisAngularOffset
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveCanId,
        DriveConstants.frontRightTurnCanId,
        DriveConstants.frontRightCANcoderID,
        DriveConstants.kFrontRightChassisAngularOffset
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveCanId,
        DriveConstants.backLeftTurnCanId,
        DriveConstants.backLeftCANcoderID,
        DriveConstants.kBackLeftChassisAngularOffset
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveCanId,
        DriveConstants.backRightTurnCanId,
        DriveConstants.backRightCANcoderID,
        DriveConstants.kBackRightChassisAngularOffset
    );

    // Create an array of swerve modules
    private final SwerveModule[] swerveModules = new SwerveModule[] {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    };

    private final Pigeon2 pigeon2 = new Pigeon2(DriveConstants.pigeonCanId, "CANivore");

    // Create a new SwerveDriveKinematics object with the locations of the swerve modules
    // The locations are defined in meters relative to the center of the robot\
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    // Initialize the odometry
    SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[] {
                    new SwerveModulePosition(), // Front-Left
                    new SwerveModulePosition(), //Front-Right
                    new SwerveModulePosition(), // Back-Left
                    new SwerveModulePosition() // Back-Right
            }, Pose2d.kZero);

    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions()
    {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
            new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
            new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
            new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
        };
    }

    @Override
    public void periodic()
    {
        // Update the relitive econders to be the same as the CANcoder
        swerveModules[0].updateEncoderState();
        swerveModules[1].updateEncoderState();
        swerveModules[2].updateEncoderState();
        swerveModules[3].updateEncoderState();

        // Update the odometry every run.
        odometry.update(
                Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()), // returns current gyro reading as a Rotation2d
                getCurrentSwerveModulePositions() // Get the current swerve module positions
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return kinematics.toChassisSpeeds(
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        );
    }

    public void driveRelative(ChassisSpeeds speeds)
    {
        // Extract the individual speed components from the ChassisSpeeds object
        boolean fieldRelative = false; // The robot is robot orentend
        double xSpeedDelivered = speeds.vxMetersPerSecond;         // Forward/backward speed
        double ySpeedDelivered = speeds.vyMetersPerSecond;         // Left/right strafe speed
        double rotDelivered = speeds.omegaRadiansPerSecond;        // Rotational speed (angular velocity)

        // Convert chassis speeds to swerve module states (individual wheel speeds and angles)
        // If driving field-relative, convert using current gyro heading (from pigeon2); otherwise use robot-relative
        var swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        );

        // Normalize wheel speeds if any exceed the max allowed, to preserve direction while staying within limits
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeedMetersPerSec);

        // Send the desired state (speed + angle) to each swerve module
        swerveModules[0].setDesiredState(swerveModuleStates[0]);
        swerveModules[1].setDesiredState(swerveModuleStates[1]);
        swerveModules[2].setDesiredState(swerveModuleStates[2]);
        swerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
    * Resets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            },
            pose);
    }

    /**
    * Method to drive the robot using joystick info.
    *
    * @param xSpeed        Speed of the robot in the x direction (forward).
    * @param ySpeed        Speed of the robot in the y direction (sideways).
    * @param rot           Angular rate of the robot.
    * @param fieldRelative Whether the provided x and y speeds are relative to the
    *                      field.
    */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
    {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.maxSpeedMetersPerSec;
        double ySpeedDelivered = ySpeed * DriveConstants.maxSpeedMetersPerSec;
        double rotDelivered = rot * DriveConstants.maxSpeedMetersPerSec;

        // Convert chassis speeds to swerve module states (individual wheel speeds and angles)
        // If driving field-relative, convert using current gyro heading (from pigeon2); otherwise use robot-relative
        var swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        // Normalize wheel speeds if any exceed the max allowed, to preserve direction while staying within limits
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeedMetersPerSec);

        // Send the desired state (speed + angle) to each swerve module
        swerveModules[0].setDesiredState(swerveModuleStates[0]);
        swerveModules[1].setDesiredState(swerveModuleStates[1]);
        swerveModules[2].setDesiredState(swerveModuleStates[2]);
        swerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    public void setX()
    {
        swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        swerveModules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        swerveModules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
    * Sets the swerve ModuleStates.
    *
    * @param desiredStates The desired SwerveModule states.
    */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Limits the wheel speeds so none exceed the maximum speed of the robot.
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, 
            DriveConstants.maxSpeedMetersPerSec
        );

        // Applies the corrected (or unmodified) desired state to each swerve module.
        swerveModules[0].setDesiredState(desiredStates[0]); // Front Left
        swerveModules[1].setDesiredState(desiredStates[1]); // Front Right
        swerveModules[2].setDesiredState(desiredStates[2]); // Back Left
        swerveModules[3].setDesiredState(desiredStates[3]); // Back Right
    }

    // Resets the drive encoders to currently read a position of 0.
    public void resetEncoders() {
        swerveModules[0].resetEncoders();
        swerveModules[1].resetEncoders();
        swerveModules[2].resetEncoders();
        swerveModules[3].resetEncoders();
    }  

    // Zeros the heading of the robot.
    public void zeroHeading() {
        pigeon2.reset();
    }

    /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180
    */
    public double getHeading() {
        return Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()).getDegrees();
    }

    /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
    public double getTurnRate() {
        return pigeon2.getYaw().getValueAsDouble() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }
}