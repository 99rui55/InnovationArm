// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmUtils {
    private static double section1Length = Constants.ArmConstants.SECTION_1_LENGTH;
    private static double section2Length = Constants.ArmConstants.SECTION_2_LENGTH;
    public static InterpolatingTreeMap<Double,Double> table = new InterpolatingTreeMap<>();
    public static void initializeTable(){
        table.put(0., 0.); table.put(3., 0.); table.put(6., 0.); table.put(9., 0.); 
        table.put(12., 0.); table.put(15., 0.); table.put(18., 0.); table.put(21., 0.); 
        table.put(24., 0.); table.put(27., 0.); table.put(30., 0.); table.put(33., 0.); 
        table.put(36., 0.); table.put(39., 0.); table.put(42., 0.); table.put(45., 0.); 
        table.put(48., 0.); table.put(51., 0.); table.put(54., 0.); table.put(57., 0.); 
        table.put(60., 0.); table.put(63., 0.); table.put(66., 0.); table.put(69., 0.); 
        table.put(72., 0.); table.put(75., 0.); table.put(78., 0.); table.put(81., 0.); 
        table.put(84., 0.); table.put(87., 0.); table.put(90., 0.); table.put(93., 0.); 
        table.put(96., 0.); table.put(99., 0.); table.put(102., 0.); table.put(105., 0.); 
        table.put(108., 0.); table.put(111., 0.); table.put(114., 0.); table.put(117.,0.); 
        table.put(120., 0.); table.put(123., 0.); table.put(126., 0.); table.put(129., 0.); 
        table.put(132., 0.); table.put(135., 0.); table.put(138., 0.); table.put(141., 0.); 
        table.put(144., 0.); table.put(147., 0.); table.put(150., 0.); table.put(153., 0.); 
        table.put(156., 0.); table.put(159., 0.); table.put(162., 0.); table.put(165., 0.); 
         table.put(168., 0.); table.put(171., 0.); table.put(174., 0.); table.put(177., 0.); 

    }

    public static double getTableValue(double key){
        return table.get(key);
    }

    public static Pair<Double,Double> ArmInverseKinematics(double endRotation, double endLength,
        double currentSection1Angle, double currentSection2Angle){
        double section1AngleEquasion = (-Math.pow(section2Length, 2) + Math.pow(section1Length, 2) + Math.pow(endLength, 2))/(2*section1Length*endLength);
        double section1Angle = Math.toDegrees(Math.acos(section1AngleEquasion));
        double section2AngleEquation = (-Math.pow(endLength, 2) + Math.pow(section1Length, 2) + Math.pow(section2Length, 2))/(2*section1Length*section2Length);
        double section2Angle = Math.toDegrees(Math.acos(section2AngleEquation));
        
        //improve efficiency
        
        
        
        double resAngle1 = section1Angle + endRotation;
        double resAngle2 = -((180-section2Angle) - section1Angle - endRotation);
        System.out.println("ANGLE1: " + resAngle1 + " ANGLE2: " + resAngle2);
        return new Pair<>(resAngle2, resAngle1);
      }

}
