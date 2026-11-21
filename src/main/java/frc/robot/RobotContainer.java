// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Drive;
//import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Launcher;
//import frc.robot.subsystems.Spindexer;
//import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

  // The robot's subsystems and commands are defined here...
 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public class RobotContainer {
  // Subsystems
  //private final Spindexer spindexer = new Spindexer();
  // private final Intake Intake = new Intake();   // add this (lowercase variable)
  // private final 

  // Subsystems
  


private final Drive Drive = new Drive();
private final Intake Intake = new Intake();
  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
 
  public RobotContainer() {
    configureBindings();

  }

  private double getRobotSpeedMultiplier() {
    return 1.0;
  }
 
  private void configureBindings() {
    // Hold right bumper to spin the spindexer, left bumper to back it out (unjam)

   driverController
        .leftBumper()
        .whileTrue(new InstantCommand(() -> Intake.intake()))
        .onFalse(new InstantCommand(() -> Intake.intakeStop()));

   driverController
        .povUp()
        .whileTrue(new InstantCommand(() -> Intake.stow()));
    driverController
        .povDown()
        .whileTrue(new InstantCommand(() -> Intake.deploy()));

        
  driverController
        .povLeft()
        .whileTrue(new RunCommand(() -> Intake.outtake()));
    
    Drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            Drive,
            () -> -driverController.getLeftY() * getRobotSpeedMultiplier(), // CHANGED: driverController
            () -> -driverController.getLeftX() * getRobotSpeedMultiplier(),
            () -> -driverController.getRightX()));
          
  }
 
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
