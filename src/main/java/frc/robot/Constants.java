// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int LeftFrontMotorPort = 1;
        public static final int LeftBackMotorPort = 2;
        public static final int RightFrontMotorPort = 3;
        public static final int RightBackMotorPort = 4;
    }

    public static final class LimelightConstants {
        public static final double LimelightAngle = 60; 
    }

    public static final class Joystick{
        public static final int JoystickPort = 1;
        public static final int ForwardPort = 2;
        public static final int TurnPort = 3;   
    }

}
