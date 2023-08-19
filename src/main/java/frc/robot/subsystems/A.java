// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class A implements Sendable {
    double distanceTraveled = 0;
    double velocity = 0;
    boolean isMoving = false;
    boolean stopChassis = false;
    public double getVelocity(){return velocity;}
    public boolean getIsMoving(){return isMoving;}
    private boolean getStopChassis(){return stopChassis;}
    private void setStopChassis(boolean state){
        if(state){
            //stop motors 1,2,3,4
        }
        state = false;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addBooleanProperty("IsMoving", this::getIsMoving, null);
        builder.addBooleanProperty("stopChassis", this::getStopChassis, this::setStopChassis);
    }
}
