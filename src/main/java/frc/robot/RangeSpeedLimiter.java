package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.function.Supplier;

@Logged
public class RangeSpeedLimiter {

    private final double upperEnd;
    private final double lowerEnd;
    private final double speedLimitDistance;
    private final boolean isMotorReversed;
    private final boolean testMode;

    private final MotorController motorController;
    private final Supplier<Double> positionSupplier;

    // Constructor with testMode as optional
    public RangeSpeedLimiter(
        double upperEnd, double lowerEnd, double speedLimitDistance,
        boolean isMotorReversed,
        MotorController motorController,
        Supplier<Double> positionSupplier
    ) {
        this(upperEnd, lowerEnd, speedLimitDistance, isMotorReversed, motorController, positionSupplier, false);
    }

    // Full constructor with testMode explicitly provided
    public RangeSpeedLimiter(
        double upperEnd, double lowerEnd, double speedLimitDistance,
        boolean isMotorReversed,
        MotorController motorController,
        Supplier<Double> positionSupplier,
        boolean testMode
    ) {
        this.upperEnd = upperEnd;
        this.lowerEnd = lowerEnd;
        this.speedLimitDistance = speedLimitDistance;
        this.isMotorReversed = isMotorReversed;
        this.motorController = motorController;
        this.positionSupplier = positionSupplier;
        this.testMode = testMode;
    }

    public void profileEndMotion() {
        double limit = speedLimitAtCurrentPosition();
        double motorSpeed = motorController.get(); // Retrieve the motor speed
        /*if (isMotorReversed) {
            motorSpeed = -motorSpeed; // Reverse direction if necessary
        }*/
        if (limit < Math.abs(motorSpeed)) {
            double clampedSpeed = MathUtil.clamp(motorSpeed, -limit, limit);
            if (testMode) {
                // Log the calculations instead of setting the motor speed
                System.out.println("Test Mode Enabled: Calculated clamped speed = " + clampedSpeed);
            } else {
                motorController.set(clampedSpeed); // Set the motor speed only if testMode is false
            }
        }
    }

    public double speedLimitAtCurrentPosition() {
        double position = positionSupplier.get(); // Use the supplier to get the position
        double motorDirection = motorController.get();
        if (isMotorReversed) {
            motorDirection = -motorDirection; // Reverse direction if necessary
        }

        if (upperEnd > position && position > (upperEnd - speedLimitDistance) && motorDirection > 0) {
            return upperRangePercentage(position);
        } else if ((lowerEnd + speedLimitDistance) > position && position > lowerEnd && motorDirection < 0) {
            return lowerRangePercentage(position);
        }
        return 1; // No speed limit at the current position
    }

    public double upperRangePercentage(double position) {
        return calculateMotionProfilePercentage(upperEnd, upperEnd - speedLimitDistance, position);
    }

    public double lowerRangePercentage(double position) {
        return calculateMotionProfilePercentage(lowerEnd, lowerEnd + speedLimitDistance, position);
    }

    public static double calculateMotionProfilePercentage(double start, double end, double current) {
        double total = Math.abs(end - start);
        double progress = Math.abs(current - start);
        double percentage = progress / total;
        return Math.min(1, Math.max(0, percentage));
    }
}
