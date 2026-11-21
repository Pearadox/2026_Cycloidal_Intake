package frc.robot.subsystems;
 
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import frc.robot.Constants.DriveConstants;
 
 
public class Drive extends SubsystemBase {  
 
 
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(DriveConstants.ROBOT_LENGTH / 2, DriveConstants.ROBOT_WIDTH / 2);
 
 
  private final SwerveModule frontLeft = new SwerveModule(7, 8, 12, 0.0, false);
  private final SwerveModule frontRight = new SwerveModule(3, 4, 11, 0.0, true);
  private final SwerveModule backLeft = new SwerveModule(5, 6, 13, 0.0, false);
  private final SwerveModule backRight = new SwerveModule(1, 2, 14, 0.0, true);
 
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.GYRO_ID); 
 
  
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.ROBOT_LENGTH / 2, DriveConstants.ROBOT_WIDTH / 2),    // front left
      new Translation2d(DriveConstants.ROBOT_LENGTH / 2, -DriveConstants.ROBOT_WIDTH / 2),   // front right
      new Translation2d(-DriveConstants.ROBOT_LENGTH / 2, DriveConstants.ROBOT_WIDTH / 2),   // back left
      new Translation2d(-DriveConstants.ROBOT_LENGTH / 2, -DriveConstants.ROBOT_WIDTH / 2)  // back right
      ); 
 
  
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, getGyroAngle(), getWheelPositions(), new Pose2d());
 
  
  @Override
  public void periodic() {
    odometry.update(getGyroAngle(), getWheelPositions());
  }
       
  private void setupSimpleMotor(TalonFX motor) {
    TalonFXConfiguration settings = new TalonFXConfiguration();
    settings.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    settings.CurrentLimits.StatorCurrentLimit = 40;
    settings.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(settings); }
       
  
  private SwerveModulePosition[] getWheelPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }
      
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }
 
 
  
  private void drive(double forwardSpeed, double sidewaysSpeed, double turnSpeed) {
    Rotation2d facing = getPose().getRotation();
    if (onRedAlliance()) {
      facing = facing.plus(Rotation2d.fromDegrees(180)); // red driver stands on the other side
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, turnSpeed, facing);
 
   
    SwerveModuleState[] wheels = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(wheels, DriveConstants.MAX_SPEED); 
    frontLeft.setState(wheels[0]);
    frontRight.setState(wheels[1]);
    backLeft.setState(wheels[2]);
    backRight.setState(wheels[3]);
  }
     
  
  private boolean onRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
 
  public Command lockWheels() {
    return run(() -> {
      frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      backLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      backRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    });
  }
 
  public void resetHeading() {
    Rotation2d forward = Rotation2d.fromDegrees(onRedAlliance() ? 180 : 0);
    odometry.resetPosition(getGyroAngle(), getWheelPositions(), new Pose2d(getPose().getTranslation(), forward));
  }
 
  
  private Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }
    
    
  private static class SwerveModule {
   
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final VelocityVoltage driveRequest = new VelocityVoltage(0);
    private final PositionVoltage steerRequest = new PositionVoltage(0);
 
    SwerveModule(int driveId, int steerId, int encoderId, double encoderOffset, boolean driveFlipped) {
      driveMotor = new TalonFX(driveId);
      steerMotor = new TalonFX(steerId);
      CANcoder encoder = new CANcoder(encoderId);
 
      
      CANcoderConfiguration encoderSettings = new CANcoderConfiguration();
      encoderSettings.MagnetSensor.MagnetOffset = encoderOffset;
      encoderSettings.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      encoder.getConfigurator().apply(encoderSettings);
 
      // Drive motor settings
      TalonFXConfiguration driveSettings = new TalonFXConfiguration();
      driveSettings.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      if (driveFlipped) {
        driveSettings.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      } else {
        driveSettings.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      }
      driveSettings.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEARING;   
      driveSettings.Slot0.kV = 0.12 * DriveConstants.DRIVE_GEARING;                   
      driveSettings.Slot0.kP = 0.1;                                    // TODO tune this
      driveSettings.CurrentLimits.StatorCurrentLimit = 80;
      driveSettings.CurrentLimits.StatorCurrentLimitEnable = true;
      driveMotor.getConfigurator().apply(driveSettings);
 
      // Steer motor settings. 
      TalonFXConfiguration steerSettings = new TalonFXConfiguration();
      steerSettings.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      steerSettings.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
      steerSettings.Feedback.FeedbackRemoteSensorID = encoderId;
      steerSettings.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      steerSettings.Feedback.RotorToSensorRatio = DriveConstants.STEER_GEARING;
      steerSettings.Feedback.SensorToMechanismRatio = 1.0;
      steerSettings.ClosedLoopGeneral.ContinuousWrap = true;          
      
      steerSettings.Slot0.kP = 100;
      steerSettings.Slot0.kD = 0.5;
      steerSettings.CurrentLimits.StatorCurrentLimit = 40;
      steerSettings.CurrentLimits.StatorCurrentLimitEnable = true;
      steerMotor.getConfigurator().apply(steerSettings);
    }
 
    Rotation2d getAngle() {
      return Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());
    }
 
    SwerveModulePosition getPosition() {
      double metersDriven = driveMotor.getPosition().getValueAsDouble() * DriveConstants.WHEEL_DISTANCE_PER_TURN;
      return new SwerveModulePosition(metersDriven, getAngle());
    }
 
    void setState(SwerveModuleState wanted) {
      wanted.optimize(getAngle()); 
      driveMotor.setControl(driveRequest.withVelocity(wanted.speedMetersPerSecond / DriveConstants.WHEEL_DISTANCE_PER_TURN));
      steerMotor.setControl(steerRequest.withPosition(wanted.angle.getRotations()));
    }
  }
 
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_SPEED);
 
   
    frontLeft.setState(setpointStates[0]);
    frontRight.setState(setpointStates[1]);
    backLeft.setState(setpointStates[2]);
    backRight.setState(setpointStates[3]);
  } 
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_SPEED; 
  }
 
  
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }
}