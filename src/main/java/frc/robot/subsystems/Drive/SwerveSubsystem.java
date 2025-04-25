package frc.robot.subsystems.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem {

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry   odometry;
    Pigeon2               gyro;
    SwerveModule[]        swerveModules;

    public void swerveDrive()
    {
        // Initialize the swerve modules
        swerveModules = new SwerveModule[4];

        // Create a new SwerveDriveKinematics object with the locations of the swerve modules
        // The locations are defined in meters relative to the center of the robot\
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Units.inchesToMeters(12.4375), Units.inchesToMeters(12.4375)), // Front left
                new Translation2d(Units.inchesToMeters(12.4375), Units.inchesToMeters(-12.4375)), // Front right
                new Translation2d(Units.inchesToMeters(-12.4375), Units.inchesToMeters(12.4375)), // Back left
                new Translation2d(Units.inchesToMeters(-12.4375), Units.inchesToMeters(-12.4375)) // Back right
        );

        // Initialize the gyroscope
        gyro = new Pigeon2();

        // Initialize the odometry
        odometry = new SwerveDriveOdometry(
                kinematics,
                gyro.getAngle(), // returns current gyro reading as a Rotation2d
                new SwerveModulePosition[] {
                        new SwerveModulePosition(), // Front-Left
                        new SwerveModulePosition(), //Front-Right
                        new SwerveModulePosition(), // Back-Left
                        new SwerveModulePosition() // Back-Right
                },
                // Initial position of the robot
                new Pose2d(0, 0, new Rotation2d()) // X = 0, Y = 0, Heading = 0
        );
    }

    public void drive()
    {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        ChassisSpeeds testChassisSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));

        // Get swerve module states from the chassis speeds
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testChassisSpeeds);

        // Set the swerve module states to the swerve modules
        swerveModules[0].setState(swerveModuleStates[0]); // Front-Left
        swerveModules[1].setState(swerveModuleStates[1]); // Front-Right
        swerveModules[2].setState(swerveModuleStates[2]); // Back-Left
        swerveModules[3].setState(swerveModuleStates[3]); // Back-Right
    }

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
        // Update the odometry every run.
        odometry.update(
                gyro.getAngle(), // returns current gyro reading as a Rotation2d
                getCurrentSwerveModulePositions() // Get the current swerve module positions
        );
    }
}