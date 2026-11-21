package frc.robot.subsystems;
 
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
 
  private final TalonFX PIVOT_LEFT = new TalonFX(IntakeConstants.PIVOT_LEFT_ID);
  //private final TalonFX PIVOT_RIGHT = new TalonFX(PIVOT_RIGHT_ID);
  private final TalonFX ROLLER_LEFT = new TalonFX(IntakeConstants.ROLLER_LEFT_ID);
  private final TalonFX ROLLER_RIGHT = new TalonFX(IntakeConstants.ROLLER_RIGHT_ID);
 
  private final MotionMagicDutyCycle pivotRequest = new MotionMagicDutyCycle(0);
  private final VoltageOut rollerRequest = new VoltageOut(0);
 
  public Intake() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO; 
    pivotConfig.Slot0.kP = 0.4; // tODO: tune this
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 300.0;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations((IntakeConstants.DEPLOYED_DEGS + 5) * IntakeConstants.PIVOT_GEAR_RATIO);
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations((IntakeConstants.STOWED_DEGS - 5) * IntakeConstants.PIVOT_GEAR_RATIO);
 
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PIVOT_LEFT.getConfigurator().apply(pivotConfig);
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // mirror
    //PIVOT_RIGHT.getConfigurator().apply(pivotConfig);
 
   
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
 
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ROLLER_LEFT.getConfigurator().apply(rollerConfig);
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ROLLER_RIGHT.getConfigurator().apply(rollerConfig);
 
   
    PIVOT_LEFT.setPosition(Units.degreesToRotations(IntakeConstants.STOWED_DEGS));
    //PIVOT_RIGHT.setPosition(Units.degreesToRotations(STOWED_DEGS));
  }
 
  private void setPivot(double degrees) {
    double rotations = Units.degreesToRotations(degrees * IntakeConstants.PIVOT_GEAR_RATIO);
    PIVOT_LEFT.setControl(pivotRequest.withPosition(rotations));
    //PIVOT_RIGHT.setControl(pivotRequest.withPosition(rotations));
  }
 
  private void setRollers(double volts) {
    ROLLER_LEFT.setControl(rollerRequest.withOutput(volts));
    ROLLER_RIGHT.setControl(rollerRequest.withOutput(volts));
  }
  
  public void intake() {
      
        setPivot(IntakeConstants.DEPLOYED_DEGS);
        System.out.println("intake intake");
          setRollers(IntakeConstants.ROLLER_VOLTS);
        
  };

  public void intakeStop() {
          setRollers(0);
          setPivot(IntakeConstants.STOWED_DEGS);
          System.out.println("intake intakeE");
        
  };
 

  public void outtake() {
    System.out.println("intake outtake");
    setRollers(-IntakeConstants.ROLLER_VOLTS);
  
  }
 
  
  public void deploy() {
    System.out.println("intake deployed");
    setPivot(IntakeConstants.DEPLOYED_DEGS);
  }
 
  
  public void stow() {
     setPivot(IntakeConstants.STOWED_DEGS);
  }
}
 