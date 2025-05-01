package frc.robot.subsystems.drive;

import static frc.robot.util.SparkUtil.*;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private SparkMax driveMotor;
    private SparkMax steerMotor;
    private CANcoder absoluteEncoder;
    private SparkClosedLoopController drivingPIDController;
    private SparkClosedLoopController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID, double chassisAngularOffset)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID, "CANivore");

        // Get the PID Controllers
        drivingPIDController = driveMotor.getClosedLoopController();
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
            .inverted(DriveConstants.turnEncoderInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
            .voltageCompensation(12.0);
        steerConfig.encoder
            // Apply position and velocity conversion factors for the turning encoder. We
            // want these in radians and radians per second to use with WPILib's swerve
            // APIs.
            .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
            .uvwAverageDepth(2);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
            // Set the PID gains for the turning motor
            .pidf(
                DriveConstants.kTurningP,  // Proportional gain - how strongly the controller reacts to error
                DriveConstants.kTurningI,  // Integral gain - helps eliminate steady-state error over time
                DriveConstants.kTurningD,  // Derivative gain - dampens the system based on the rate of error change
                DriveConstants.kTurningFF  // Feedforward - anticipates needed output based on desired motion
             );
        steerConfig.signals
        // Always have the primary encoder's position reporting enabled.
        .primaryEncoderPositionAlwaysOn(true)
        
        // Set how often (in milliseconds) the encoder position is updated.
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        
        // Always have the primary encoder's velocity reporting enabled.
        .primaryEncoderVelocityAlwaysOn(true)
        
        // Set how often (in milliseconds) the encoder velocity is updated.
        .primaryEncoderVelocityPeriodMs(20)
        
        // Set how often the applied motor output (commanded output %) is updated.
        .appliedOutputPeriodMs(20)
        
        // Set how often the bus voltage (voltage supplied to motor) is updated.
        .busVoltagePeriodMs(20)
        
        // Set how often the output current (current drawn by motor) is updated.
        .outputCurrentPeriodMs(20);
         
         // Attempt to configure the turning motor controller (turnSpark) up to 5 times,
         // applying the turning configuration settings.
         // Uses safe reset mode and persists the parameters across reboots.
         tryUntilOk(
            steerMotor,
             5, // Number of retry attempts
             () -> steerMotor.configure(
                 steerConfig,                   // Configuration object for turn motor
                 ResetMode.kResetSafeParameters, // Only reset parameters that are safe to reset
                 PersistMode.kPersistParameters  // Save the settings to flash memory
             )
         );
         
         // Attempt to set the turning encoder's position based on the absolute encoder reading,
         // converting it from rotations (0–1) to radians (0–2π), with up to 5 retry attempts.
         tryUntilOk(
            steerMotor,
             5, // Number of retry attempts
             () -> steerEncoder.setPosition(
                absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI // Converts rotations to radians
             )
         );




        // Drive Motor Configuration
        driveConfig
            .inverted(DriveConstants.driveEncoderInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
            .voltageCompensation(12.0);
        driveConfig.encoder
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2)
            // Apply position and velocity conversion factors for the driving encoder. The
            // native units for position and velocity are rotations and RPM, respectively,
            // but we want meters and meters per second to use with WPILib's swerve APIs. 
            .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set the PID gains for the driving motor
            .pidf(
                    DriveConstants.kDrivingP,  // Proportional gain - how strongly the controller reacts to error
                    DriveConstants.kDrivingI,  // Integral gain - helps eliminate steady-state error over time
                    DriveConstants.kDrivingD,  // Derivative gain - dampens the system based on the rate of error change
                    DriveConstants.kDrivingFF  // Feedforward - anticipates needed output based on desired motion
                 );
        driveConfig.signals
            // Ensure the primary encoder's position is always being measured.
            .primaryEncoderPositionAlwaysOn(true)

            // Set how often (in milliseconds) to update the encoder's position reading.
            // Calculated based on the odometry update frequency (ex: 100 Hz → 10ms).
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))

            // Ensure the primary encoder's velocity is always being measured.
            .primaryEncoderVelocityAlwaysOn(true)

            // Set how often (in milliseconds) to update the encoder's velocity reading.
            // 20ms is a standard update rate (~50Hz).
            .primaryEncoderVelocityPeriodMs(20)

            // Set how often (in milliseconds) to update the applied motor output reading.
            .appliedOutputPeriodMs(20)

            // Set how often (in milliseconds) to update the bus voltage reading (motor voltage).
            .busVoltagePeriodMs(20)

            // Set how often (in milliseconds) to update the output current reading (current draw).
            .outputCurrentPeriodMs(20);
        // Attempt to configure the drive motor controller (driveSpark) up to 5 times,
        // to ensure that the configuration is correctly applied even if there are CAN communication hiccups.
        tryUntilOk(
            driveMotor,
            5, // Number of retries allowed
            () -> driveMotor.configure(
                driveConfig,                   // Configuration settings for the motor
                ResetMode.kResetSafeParameters, // Reset only safe parameters (won't fully factory reset)
                PersistMode.kPersistParameters  // Save the configuration so it persists after reboot
            )
        );
        // Attempt to reset (zero) the drive encoder's position up to 5 times,
        // making sure the encoder reads 0.0 at startup.
        tryUntilOk(
            driveMotor,
            5, // Number of retries allowed
            () -> driveEncoder.setPosition(0.0) // Set encoder position to 0
        );

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(steerEncoder.getPosition());
    }

    /**
    Get the distance in meters.
    */
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }

    /**
    Get the angle.
    */
    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(steerEncoder.getPosition() - m_chassisAngularOffset);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(driveEncoder.getVelocity(),
            new Rotation2d(steerEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state)
    {
        turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

    public void updateEncoderState(){
        steerEncoder.setPosition(Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble()).getRadians());
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(steerEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(steerEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turningPIDController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
}