// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmUtils;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  Arm arm;
  double spAngle;
  double spLength;
  double section1CurrentAngle;
  double section2CurrentAngle;

  PIDController scetion1Pid;
  PIDController scetion2Pid;
  CommandXboxController controller;
  double requesteAngle;
  double requesteLength = 0.01;
  double speedOutput;

  boolean pressButton;
  double sec1Kp = Constants.ArmConstants.SECTION_1_KP;
  double sed1Ki = Constants.ArmConstants.SECTION_1_KI;
  double sec1Kd = Constants.ArmConstants.SECTION_1_KD;

  double sec2Kp = Constants.ArmConstants.SECTION_2_KP;
  double sed2Ki = Constants.ArmConstants.SECTION_2_KI;
  double sec2Kd = Constants.ArmConstants.SECTION_2_KD;
  
  public MoveArm(Arm arm, double sp, CommandXboxController controller) {
    this.arm = arm;
    this.spAngle = sp;
    scetion1Pid = new PIDController(Constants.ArmConstants.SECTION_1_KP, Constants.ArmConstants.SECTION_1_KI, Constants.ArmConstants.SECTION_1_KD);
    scetion1Pid.setTolerance(Constants.ArmConstants.SECTION_1_PID_TOLERANCE);
    scetion2Pid = new PIDController(Constants.ArmConstants.SECTION_2_KP, Constants.ArmConstants.SECTION_2_KI, Constants.ArmConstants.SECTION_2_KD);
    scetion2Pid.setTolerance(Constants.ArmConstants.SECTION_2_PID_TOLERANCE);
    this.controller = controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requesteAngle = 0;
    section1CurrentAngle = arm.getAngleM1();
    section2CurrentAngle = arm.getAngleM1();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(controller.getLeftY())>0.5){
      requesteAngle += 0.5 * Math.signum(controller.getLeftY());
    } 
    if(controller.getRightY() > 0.5 && requesteLength < Constants.ArmConstants.MAX_LENGTH - 0.05){
      requesteLength += 0.05;
    } else if(controller.getRightY() < -0.5 && requesteLength > 0.05){
      requesteLength -= 0.05;
    } 

    if(controller.b().getAsBoolean()){}
    if(controller.a().getAsBoolean()){spAngle = requesteAngle; spLength = requesteLength; updatePidSp();}
    section1CurrentAngle = arm.getAngleM1();
    section2CurrentAngle = arm.getAngleM2();
    arm.motor1.setSpeed(scetion1Pid.calculate(section1CurrentAngle));
    arm.motor2.setSpeed(scetion2Pid.calculate(section2CurrentAngle));
  }

  private void updatePidSp(){
    Pair<Double,Double> angles = ArmUtils.ArmInverseKinematics(spAngle, spLength, arm.getAngleM1(), arm.getAngleM2());
    scetion1Pid.setSetpoint(angles.getFirst());
    scetion2Pid.setSetpoint(angles.getSecond());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("REQUESTED ANGLE", ()->{return requesteAngle;}, null);
      builder.addDoubleProperty("REQUESTED LENGTH", ()->{return requesteLength;}, null);

    builder.addDoubleProperty("SEC1 SET POINT", ()->{return scetion1Pid.getSetpoint();}, null);
    builder.addDoubleProperty("SEC1 PID OUTPUT", ()->{return scetion1Pid.calculate(section1CurrentAngle);}, null);
    builder.addDoubleProperty("SEC1 PID ERROR", ()->{return scetion1Pid.getPositionError();}, null);
    builder.addDoubleProperty("SEC1 KP", ()->{return scetion1Pid.getP();}, (p)->{scetion1Pid.setP(p);});
    builder.addDoubleProperty("SEC1 KI", ()->{return scetion1Pid.getI();}, (i)->{scetion1Pid.setI(i);});
    builder.addDoubleProperty("SEC1 KD", ()->{return scetion1Pid.getD();}, (d)->{scetion1Pid.setD(d);});

    builder.addDoubleProperty("SEC2 SET POINT", ()->{return scetion2Pid.getSetpoint();}, null);
    builder.addDoubleProperty("SEC2 PID OUTPUT", ()->{return scetion2Pid.calculate(section2CurrentAngle);}, null);
    builder.addDoubleProperty("SEC2 PID ERROR", ()->{return scetion2Pid.getPositionError();}, null);
    builder.addDoubleProperty("SEC2 KP", ()->{return scetion2Pid.getP();}, (p)->{scetion2Pid.setP(p);});
    builder.addDoubleProperty("SEC2 KI", ()->{return scetion2Pid.getI();}, (i)->{scetion2Pid.setI(i);});
    builder.addDoubleProperty("SEC2 KD", ()->{return scetion2Pid.getD();}, (d)->{scetion2Pid.setD(d);});

  }

}
