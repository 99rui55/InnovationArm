// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.SimulatorMotor;
import frc.robot.utils.TrajectoryGenerator;

import java.util.Arrays;

import javax.swing.plaf.nimbus.State;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  /** Creates a new Parallelogram. */
  public final SimulatorMotor motor1;
  public final SimulatorMotor motor2;
  public final Field2d armDisplay;
  
  


  public Arm() {
    ArmUtils.initializeTable();
    motor1 = new SimulatorMotor();
    motor2 = new SimulatorMotor();
    armDisplay = new Field2d();
    SmartDashboard.putData(armDisplay);
    initializeArmDisplay();
    
    
  }

  public void setSpeedM1(double speed){
    motor1.setSpeed((speed * Constants.ArmConstants.TICK_PER_DEGREE));
  }

  public void setAngleM1(double angle){motor1.setAngle(angle);}

  public double getAngleM1(){
    return motor1.getDegrees();
  }

  public void setSpeedM2(double speed){
    motor2.setSpeed((speed * Constants.ArmConstants.TICK_PER_DEGREE));
  }

  public void setAngleM2(double angle){motor2.setAngle(angle);}

  public double getAngleM2(){
    return motor2.getDegrees();
  }

  

  @Override
  public void periodic() {
    updateArmDisplay();
  }

  private void initializeArmDisplay(){
    double arm1Length = Constants.ArmConstants.SECTION_1_LENGTH * Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER;
    double arm2Length = Constants.ArmConstants.SECTION_2_LENGTH * Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER;

    PathConstraints constraints = new PathConstraints(4, 10);
    TrajectoryGenerator arm1 = new TrajectoryGenerator(Alliance.Blue);
    arm1.add(new Pose2d(new Translation2d(8.2,0), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));
    arm1.add(new Pose2d(new Translation2d(arm1Length + 8.2,0), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));
    
    armDisplay.getObject("Base").setTrajectory(PathPlanner.loadPath("BaseStand1", constraints));
    armDisplay.getObject("Arm1").setTrajectory(PathPlanner.generatePath(constraints,Arrays.asList(arm1.generate())));

    TrajectoryGenerator arm2 = new TrajectoryGenerator(Alliance.Blue);
    arm2.add(new Pose2d(new Translation2d(arm1Length + 8.2,0), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));
    arm2.add(new Pose2d(new Translation2d(arm1Length + 8.2 + arm2Length,0), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));
    
    armDisplay.getObject("Arm2").setTrajectory(PathPlanner.generatePath(constraints,Arrays.asList(arm2.generate())));
  }

  private void updateArmDisplay(){
    double section1Angle = getAngleM1();
    PathConstraints constraints = new PathConstraints(4, 10);
    TrajectoryGenerator arm1 = new TrajectoryGenerator(Alliance.Red);
    double section1EndY = Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER * Constants.ArmConstants.SECTION_1_LENGTH * Math.sin(Math.toRadians(section1Angle));
    double section1EndX = Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER * Constants.ArmConstants.SECTION_1_LENGTH * Math.cos(Math.toRadians(section1Angle)) + 8.2;
    arm1.add(new Pose2d(new Translation2d(8.2,0), Rotation2d.fromDegrees(section1Angle)), Rotation2d.fromDegrees(section1Angle));
    arm1.add(new Pose2d(new Translation2d(section1EndX,section1EndY), Rotation2d.fromDegrees(section1Angle)), Rotation2d.fromDegrees(section1Angle));
    armDisplay.getObject("Arm1").setTrajectory(PathPlanner.generatePath(constraints,Arrays.asList(arm1.generate())));
    //System.out.println("ENDX: " + section1EndX + " ENDY: " + section1EndY + " ANGLE: " + section1Angle);

    double section2Angle = getAngleM2();
    TrajectoryGenerator arm2 = new TrajectoryGenerator(Alliance.Red);
    double section2EndY = Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER * Constants.ArmConstants.SECTION_2_LENGTH * Math.sin(Math.toRadians(section2Angle));
    double section2EndX = Constants.ArmConstants.DISPLAY_ARM_MULTIPLIER * Constants.ArmConstants.SECTION_2_LENGTH * Math.cos(Math.toRadians(section2Angle)) + 8.2;
    double section2StartY = section1EndY;
    double section2StartX = section1EndX;
    arm2.add(new Pose2d(new Translation2d( section2StartX,section2StartY), Rotation2d.fromDegrees(section2Angle)), Rotation2d.fromDegrees(section2Angle));
    arm2.add(new Pose2d(new Translation2d(section2EndX + (section1EndX - 8.2),section2EndY + section1EndY), Rotation2d.fromDegrees(section2Angle)), Rotation2d.fromDegrees(section2Angle));
    armDisplay.getObject("Arm2").setTrajectory(PathPlanner.generatePath(constraints,Arrays.asList(arm2.generate())));
    //System.out.println("ENDX: " + section2EndX +  "STRTX: " + section2StartX + " ENDY: " + section2EndY + "STRTY: " + section2StartY + " ANGLE: " + section2Angle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
    builder.addDoubleProperty("ARM ANGLE", this::getAngleM1, null);
  }
}
