package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {

    private SparkMax driveMotor;
    private SparkMax steerMotor;
    private CANcoder absoluteEncoder;
    private SparkClosedLoopController drivePIDController;
    private SparkClosedLoopController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID);

        // Get the PID Controllers
        drivePIDController = driveMotor.getClosedLoopController();
        turningPIDController = steerMotor.getClosedLoopController();

        // Get the Encoders
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        // Make a new SparkMaxConfig for the drive and steer motors
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        // Set the motors cofigs to default
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

        // Set the CANcoder configs
        CANcoderConfiguration CANcoderConfigs = new CANcoderConfiguration();
        absoluteEncoder.getConfigurator().apply(CANcoderConfigs);

        // Set the magnet sensor configs
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        CANcoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);

        // Steering Motor Configuration
        steerConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        steerConfig.encoder
            // Apply position and velocity conversion factors for the turning encoder. We
            // want these in radians and radians per second to use with WPILib's swerve
            // APIs.
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .setPositionPIDWrappingEnabled(true)
            .setPositionPIDWrappingMinInput(0)
            .setPositionPIDWrappingMaxInput(90);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set the PID gains for the turning motor. Note these are example gains, and you
            // may need to tune them for your own robot!
            .setP(ModuleConstants.kTurningP)
            .setI(ModuleConstants.kTurningI)
            .setD(ModuleConstants.kTurningD)
            .setFF(ModuleConstants.kTurningFF);


        // Drive Motor Configuration
        driveConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            // Apply position and velocity conversion factors for the driving encoder. The
            // native units for position and velocity are rotations and RPM, respectively,
            // but we want meters and meters per second to use with WPILib's swerve APIs. 
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set the PID gains for the turning motor. Note these are example gains, and you
            // may need to tune them for your own robot!
            .setP(ModuleConstants.kDrivingP)
            .setI(ModuleConstants.kDrivingI)
            .setD(ModuleConstants.kDrivingD)
            .setFF(ModuleConstants.kDrivingFF);

        // Save the Spark Max configs. if a Spark Max browns out during
        // operation, it will maintain the above configurations.
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder.setPosition(0);
        steerEncoder.setPosition(steerEncoder.getPosition() * 360);
    }

    public double getDistance()
    {
        return driveEncoder.getPosition();
    }

    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }

    public void setState(SwerveModuleState state)
    {

    }
}