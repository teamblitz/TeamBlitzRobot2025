package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


public class VisionConstants {

    // Larger numbers mean less trust.
    public static final Matrix<N3, N1> WHEEL_ODOMETRY_TRUST = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final Matrix<N3, N1> SINGLE_TAG_TRUST =  VecBuilder.fill(0.025, 0.025, .2);
    public static final Matrix<N3, N1> MULTI_TAG_TRUST =  VecBuilder.fill(.010, .010, 0.025);

    public static final Matrix<N3, N1> NO_TRUST = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    /** Std Dev Calculation Constants */
    public static final double ORDER = 2;
    public static final double PROPORTION = 3.25;


    /** Filtering Constants */
    public static final double MAX_AMBIGUITY = 0.40;
    public static final double MAX_DISTANCE_SINGLE_TAG = 3.75;
    public static final double MAX_DISTANCE_MULTI_TAG = 6.0;
}
