package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;

public class Leds extends SubsystemBase {
    private static Leds instance;

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds();
        }
        return instance;
    }

    // Robot state tracking
    public int loopCycleCount = 0;
    public boolean requestCoral = false;

    public boolean stationIntake = false;
    public boolean hasCoral = false;
    public boolean hasAlgae = false; // I wish :sob:

    public double lastPickupTime = 0.0;
    public boolean indexing = false;
    public boolean autoPickupReady = false;
    public boolean autoPickupActive = false;
    public boolean autoShoot = false;
    public boolean autoDrive = false;
    public boolean climbing = false;
    public boolean endgameAlert = false;
    public boolean sameBattery = false;
    public boolean superstructureCoast = false;
    public boolean superStructureManual = false;

    public boolean autoFinished = false;
    public double autoFinishedTime = 0.0;
    public boolean lowBatteryAlert = false;
    public boolean demoMode = false;

    private Optional<Alliance> alliance = Optional.empty();
    private Color allianceColor = Color.kGreen;
    private Color secondaryDisabledColor = Color.kYellow;
    private boolean lastEnabledAuto = false;
    private double lastEnabledTime = 0.0;
    private boolean estopped = false;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final Notifier loadingNotifier;

    // Constants
    private static final boolean prideLeds = false;
    private static final int minLoopCycleCount = 10;
    private static final int length = 80;
    private static final int staticSectionLength = 3;
    private static final double strobeFastDuration = 0.1;
    private static final double strobeSlowDuration = 0.3;
    private static final double breathDuration = 1.0;
    private static final double rainbowCycleLength = 25.0;
    private static final double rainbowDuration = 0.25;
    private static final double waveExponent = 0.4;
    private static final double waveFastCycleLength = 25.0;
    private static final double waveFastDuration = 0.25;
    private static final double waveSlowCycleLength = 25.0;
    private static final double waveSlowDuration = 3.0;
    private static final double waveAllianceCycleLength = 15.0;
    private static final double waveAllianceDuration = 2.0;
    private static final double autoFadeTime = 2.5; // 3s nominal
    private static final double autoFadeMaxTime = 5.0; // Return to normal
    public static final double pickupAlertDuration = .5;

    private Leds() {
        leds = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
        loadingNotifier =
                new Notifier(
                        () -> {
                            synchronized (this) {
                                breath(
                                        Color.kWhite,
                                        Color.kBlack,
                                        System.currentTimeMillis() / 1000.0);
                                leds.setData(buffer);
                            }
                        });
        loadingNotifier.startPeriodic(0.02);
    }

    public synchronized void periodic() {
        // Update alliance color
        if (DriverStation.isFMSAttached()) {
            alliance = DriverStation.getAlliance();
            allianceColor =
                    alliance.map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
                            .orElse(Color.kGreen);
            secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
        }

        // Update auto state
        if (DriverStation.isDisabled()) {
            autoFinished = false;
        } else {
            lastEnabledAuto = DriverStation.isAutonomous();
            lastEnabledTime = Timer.getFPGATimestamp();
        }

        // Update estop state
        if (DriverStation.isEStopped()) {
            estopped = true;
        }

        // Exit during initial cycles
        loopCycleCount += 1;
        if (loopCycleCount < minLoopCycleCount) {
            return;
        }

        // Stop loading notifier if running
        loadingNotifier.stop();

        // Select LED mode
        solid(Color.kBlack); // Default to off
        if (estopped) {
            solid(Color.kRed);
        } else if (DriverStation.isDisabled()) {
            if (superstructureCoast) {
                // Arm coast alert
                solid(Color.kWhite);
            } else if (lastEnabledAuto
                    && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
                // Auto fade
                solid(
                        1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime),
                        Color.kGreen);
            } else if (lowBatteryAlert) {
                // Low battery
                solid(Color.kOrangeRed);
            } else if (prideLeds) {
                // Pride stripes
                stripes(
                        List.of(
                                Color.kBlack,
                                Color.kRed,
                                Color.kOrangeRed,
                                Color.kYellow,
                                Color.kGreen,
                                Color.kBlue,
                                Color.kPurple,
                                Color.kBlack,
                                new Color(0.15, 0.3, 1.0),
                                Color.kDeepPink,
                                Color.kWhite,
                                Color.kDeepPink,
                                new Color(0.15, 0.3, 1.0)),
                        3,
                        5.0);
                buffer.setLED(staticSectionLength, allianceColor);
            } else {
                // Default pattern
                wave(
                        allianceColor,
                        secondaryDisabledColor,
                        waveAllianceCycleLength,
                        waveAllianceDuration);
            }

            // Same battery alert
            if (sameBattery) {
                breath(Color.kRed, Color.kBlack);
            }
        } else if (DriverStation.isAutonomous()) {
            wave(Color.kGreen, Color.kYellow, waveFastCycleLength, waveFastDuration);
            if (autoFinished) {
                double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
                solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
            }
        } else { // Enabled
            if (hasCoral && lastPickupTime == -1.0) {
                lastPickupTime = Timer.getFPGATimestamp();
            } else if (!hasCoral) {
                lastPickupTime = -1.0;
            }

            if (climbing) {
                rainbow(rainbowCycleLength, rainbowDuration);
            } else if (Timer.getFPGATimestamp() - lastPickupTime <= pickupAlertDuration) {
                strobe(Color.kBlue, strobeFastDuration);
            } else if (indexing) {
                strobe(Color.kGreen, strobeFastDuration);
            } else if (autoPickupActive) {
                strobe(Color.kOrangeRed, strobeFastDuration);
            } else if (autoPickupReady) {
                solid(Color.kOrangeRed);
            } else if (hasCoral) {
                strobe(Color.kGreen, Color.kYellow, strobeSlowDuration);
            } else {
                solid(Color.kGreen);
            }

            if (endgameAlert) {
                strobe(Color.kRed, Color.kGold, strobeFastDuration);
            }
        }

        // Arm estop alert
        if (superStructureManual) {
            solid(Color.kRed);
        }

        // Deal with dumb andymark leds not following rgb conventions
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, toGRB(buffer.getLED(i)));
        }

        // Update LEDs
        leds.setData(buffer);
    }

    private void solid(Color color) {
        if (color != null) {
            for (int i = 0; i < length; i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private void solid(double percent, Color color) {
        for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
            buffer.setLED(i, color);
        }
    }

    private void strobe(Color c1, Color c2, double duration) {
        boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(c1On ? c1 : c2);
    }

    private void strobe(Color color, double duration) {
        strobe(color, Color.kBlack, duration);
    }

    private void breath(Color c1, Color c2) {
        breath(c1, c2, Timer.getFPGATimestamp());
    }

    private void breath(Color c1, Color c2, double timestamp) {
        double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        solid(new Color(red, green, blue));
    }

    private void rainbow(double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = 0; i < length; i++) {
            x += xDiffPerLed;
            x %= 180.0;
            buffer.setHSV(i, (int) x, 255, 255);
        }
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < length; i++) {
            x += xDiffPerLed;
            double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
                ratio = 0.5;
            }
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            buffer.setLED(i, new Color(red, green, blue));
        }
    }

    private void stripes(List<Color> colors, int length, double duration) {
        int offset =
                (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
        for (int i = 0; i < length; i++) {
            int colorIndex =
                    (int) (Math.floor((double) (i - offset) / length) + colors.size())
                            % colors.size();
            colorIndex = colors.size() - 1 - colorIndex;
            buffer.setLED(i, colors.get(colorIndex));
        }
    }

    public Color toGRB(Color color) {
        return new Color(color.green, color.red, color.blue);
    }
}
