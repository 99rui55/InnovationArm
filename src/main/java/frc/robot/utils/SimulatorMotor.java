// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Add your docs here. */
public class SimulatorMotor implements Sendable {
    double speed = 0;
    int position = 0;
    double timeStamp = 0;

    public void setPosition(int pos){
        position = pos;
    }

    public void setAngle(double angle){
        position = (int)angle*Constants.ArmConstants.TICK_PER_DEGREE;
    }

    public int getPosition(){
        updatePosition();
        return position;
        
    }

    public void setSpeed(double speed){
        if(Math.abs(Math.abs(speed) - Math.abs(this.speed)) > (Constants.ArmConstants.SECTION_1_MAX_ACCELERATION * Constants.ArmConstants.TICK_PER_DEGREE)/50){
            this.speed += Math.signum(speed) * (Constants.ArmConstants.SECTION_1_MAX_ACCELERATION * Constants.ArmConstants.TICK_PER_DEGREE)/50;
        }else{
            this.speed = speed;
        }
        updatePosition();
    }

    public double getSpeed(){
        return speed/Constants.ArmConstants.TICK_PER_DEGREE;
    }

    public double getDegrees(){
        return (double)position/Constants.ArmConstants.TICK_PER_DEGREE;
    }

    private void updatePosition(){
        double time = Timer.getFPGATimestamp();
        position += speed*(time-timeStamp) * Constants.ArmConstants.TICK_PER_DEGREE;
        timeStamp = time;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("speed", this::getSpeed, null);
        builder.addDoubleProperty("position", this::getDegrees, null);
    }

}
