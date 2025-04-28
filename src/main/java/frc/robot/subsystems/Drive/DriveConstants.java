package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 3.62; // Circumference of wheel * Max Wheel RPS
    public static final double odometryFrequency = 100.0; // Hz

    // The horizontal distance between the centers of the left and right wheels (side-to-side)  
    public static final double trackWidth = Units.inchesToMeters(24.75);

    // The distance between the centers of the front and back wheels (front-to-back)
    public static final double wheelBase = Units.inchesToMeters(24.75);

    // Calculate the distance from the center of the robot to any one of the wheels (drive base radius).
    // This uses the Pythagorean theorem (hypotenuse of half the track width and half the wheelbase),
    // because the wheels are offset both sideways and forward/backward from the center.
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    // Define the positions of each swerve module relative to the center of the robot (robot's origin).
    // Each Translation2d is (x, y) where:
    //  - Positive x = right, negative x = left
    //  - Positive y = forward, negative y = backward
    // Positions are based on half the trackWidth and half the wheelBase.
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        
        // Front Left Module: right (positive x), forward (positive y)
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),

        // Front Right Module: right (positive x), backward (negative y)
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),

        // Back Left Module: left (negative x), forward (positive y)
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),

        // Back Right Module: left (negative x), backward (negative y)
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.888);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(2.362);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(-0.066);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.207);

    // Device CAN IDs
    public static final int pigeonCanId = 13;

    public static final int frontLeftDriveCanId = 1;
    public static final int backLeftDriveCanId = 7;
    public static final int frontRightDriveCanId = 4;
    public static final int backRightDriveCanId = 10;

    public static final int frontLeftTurnCanId = 2;
    public static final int backLeftTurnCanId = 8;
    public static final int frontRightTurnCanId = 5;
    public static final int backRightTurnCanId = 11;

    public static final int frontLeftCANcoderID = 3;
    public static final int backLeftCANcoderID = 9;
    public static final int frontRightCANcoderID = 6;
    public static final int backRightCANcoderID = 12;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = 0.1016 / 2;
    public static final double driveMotorReduction = 6.75;
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 150 / 7.0;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final boolean driveEncoderInverted = false;
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    
    // Turn encoder configuration
    public static final boolean turnEncoderInverted = false;
    public static final double turnEncoderPositionFactor =
        2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double turnEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double kDrivingP = 0.0;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;

    // Turn PID configuration
    public static final double kTurningP = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningFF = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians


}
