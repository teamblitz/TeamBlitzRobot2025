package frc.lib.oi;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBox extends GenericHID {

    public ButtonBox(int port) {
        super(port);
    }

    @Override
    public boolean getRawButton(int button) {
        if (button == 1000) {
            return getRawAxis(2) > .75;
        } else if (button == 1001) {
            return getRawAxis(3) < -.75;
        } else {
            return super.getRawButton(button);
        }
    }

    public enum Button {
        kL1(5),
        kL2(1000),
        kX(3),
        kA(1),
        kY(4),
        kB(2),
        kR1(6),
        kR2(1001),
        kL3(9),
        kR3(10);

        public final int value;

        private Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        kY(1),
        kX(0);

        public final int value;

        private Axis(int value) {
            this.value = value;
        }
    }
}
