// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.A;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private Arm arm;
  private final CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private MoveArm moveArm;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    A a = new A();
    SmartDashboard.putData(a);
    arm = new Arm();
    moveArm = new MoveArm(arm, 0, controller);
    SmartDashboard.putData("A", arm);
    SmartDashboard.putData(moveArm);
    SmartDashboard.putData("Motor", arm.motor1);
    arm.setDefaultCommand(moveArm);
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    InstantCommand command1 = new InstantCommand();
    InstantCommand command2 = new InstantCommand();
    InstantCommand command3 = new InstantCommand();


    ParallelDeadlineGroup command = new ParallelDeadlineGroup(command1, command2, command3);

    // An example command will be run in autonomous
    return command;
  }
}
