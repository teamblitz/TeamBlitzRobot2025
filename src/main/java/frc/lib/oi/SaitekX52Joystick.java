package frc.lib.oi;

import edu.wpi.first.wpilibj.GenericHID;

public class SaitekX52Joystick extends GenericHID {

    private final int kX = 0;
    private final int kY = 1;
    private final int kTwist = 5;

    public enum Axis {
        kXAxis(0),
        kYAxis(1),
        /** Throtle */
        kThrotle(2),
        /** Small Dial */
        kXRot(3),
        /** Large Dial */
        kYRot(4),
        /** Twist */
        kZRot(5),
        kSlider(6);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum Button {
        kT1(9),
        kT2(10),
        kT3(11),
        kT4(12),
        kT5(13),
        kT6(14),
        /** Upper trigger pulled atleast 1/2 way */
        kUpperTrigger1(1),
        /** Upper trigger pulled all the way */
        kUpperTrigger2(15),
        kLowerTrigger(6),
        kFire(2),
        kA(3),
        kB(4),
        kC(5),
        kD(7),
        kE(8),
        kMouseButton(16),
        kScrollDown(17),
        kScrollUp(18),
        // 19 unknown
        /** Hat to the right of fire button */
        kHatUp(20),
        /** Hat to the right of fire button */
        kHatRight(21),
        /** Hat to the right of fire button */
        kHatDown(22),
        /** Hat to the right of fire button */
        kHatLeft(23),
        kThrotleKnobUp(24),
        kThrotleKnobRight(25),
        kThrotleKnobDown(26),
        kThrotleKnobLeft(27),
        kModeRed(28),
        kModePink(29),
        kModeBlue(30),
        kI(31);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public SaitekX52Joystick(int port) {
        super(port);
    }

    public double getX() {
        return getRawAxis(kX);
    }

    public double getY() {
        return getRawAxis(kY);
    }

    public double getTwist() {
        return getRawAxis(kTwist);
    }

    // public double getButton() {
    //     return getRawButton(button);
    // }
}
