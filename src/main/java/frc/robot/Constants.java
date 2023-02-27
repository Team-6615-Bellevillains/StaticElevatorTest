package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final int kCimcoderPulsesPerRevolution = 20;

    public static final class ElevatorConstants {
        /*
         * We are unsure about the integer values needed here right now. Temp values are
         * in place
         */

        public static final int verticalMotorAPort = 17;
        public static final int verticalMotorBPort = 3;

        public static final double verticalGearRatio = 12.0 / 60.0;
        public static final double verticalMaxHeight = Units.inchesToMeters(30);
        public static final double verticalGearDiameter = Units.inchesToMeters(1.76);

        public static final int verticalEncoderPulsesPerRevolution = kCimcoderPulsesPerRevolution;
        public static final double verticalRotationsToDistance = verticalGearDiameter * Math.PI * verticalGearRatio;

        public static final double verticalLowHeight = 0;
        public static final double verticalMidHeight = Units.inchesToMeters(15);
        public static final double verticalHighHeight = verticalMaxHeight - Units.inchesToMeters(1);

        public static final double kPVerticalElevator = 0;
        public static final double kIVerticalElevator = 0;
        public static final double kDVerticalElevator = 0;

        public static final double kSVerticalElevator = 0;
        public static final double kGVerticalElevator = 0;
        public static final double kVVerticalElevator = 0;
        public static final double kAVerticalElevator = 0;
    }


}