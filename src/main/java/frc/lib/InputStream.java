package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

// (from team 1155)

/**
 * A functional interface to aid in modifying double suppliers, such as from a joystick.
 */
@FunctionalInterface
public interface InputStream extends DoubleSupplier {

  /**
   * Creates an input stream from another.
   *
   * @param base The base stream.
   * @return A new input stream.
   */
  public static InputStream of(DoubleSupplier base) {
    return base::getAsDouble;
  }

  public static InputStream hypot(InputStream x, InputStream y) {
    return () -> Math.hypot(x.get(), y.get());
  }

  public static InputStream atan(InputStream x, InputStream y) {
    return () -> Math.atan2(x.get(), y.get());
  }

  /**
   * Shorthand to return a double value.
   *
   * @return The value from {@link #getAsDouble()}.
   */
  public default double get() {
    return getAsDouble();
  }

  /**
   * Maps the stream outputs by an operator.
   *
   * @param operator A function that takes in a double input and returns a double output.
   * @return A mapped stream.
   */
  public default InputStream map(DoubleUnaryOperator operator) {
    return () -> operator.applyAsDouble(getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A supplier of scaling factors.
   * @return A scaled stream.
   */
  public default InputStream scale(DoubleSupplier factor) {
    return map(x -> x * factor.getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A scaling factor.
   * @return A scaled stream.
   */
  public default InputStream scale(double factor) {
    return scale(() -> factor);
  }

  /**
   * Negates the stream outputs.
   *
   * @return A stream scaled by -1.
   */
  public default InputStream negate() {
    return scale(-1);
  }

  /**
   * Offsets the stream by a factor.
   *
   * @param factor A supplier of offset values.
   * @return An offset stream.
   */
  public default InputStream add(DoubleSupplier offset) {
    return map(x -> x + offset.getAsDouble());
  }

  /**
   * Offsets the stream by a factor.
   *
   * @param factor An offset.
   * @return An offset stream.
   */
  public default InputStream add(double factor) {
    return add(() -> factor);
  }

  /**
   * Raises the stream outputs to an exponent.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default InputStream pow(double exponent) {
    return map(x -> Math.pow(x, exponent));
  }

  /**
   * Raises the stream outputs to an exponent and keeps their original sign.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default InputStream signedPow(double exponent) {
    return map(x -> Math.copySign(Math.pow(x, exponent), x));
  }

  /**
   * Filters the stream's outputs by the provided {@link LinearFilter}.
   *
   * @param filter The linear filter to use.
   * @return A filtered stream.
   */
  public default InputStream filter(LinearFilter filter) {
    return map(filter::calculate);
  }

  /**
   * Deadbands the stream outputs by a minimum bound and scales them from 0 to a maximum bound.
   *
   * @param bound The lower bound to deadband with.
   * @param max The maximum value to scale with.
   * @return A deadbanded stream.
   */
  public default InputStream deadband(double deadband, double max) {
    return map(x -> MathUtil.applyDeadband(x, deadband, max));
  }

  /**
   * Clamps the stream outputs by a maximum bound.
   *
   * @param magnitude The upper bound to clamp with.
   * @return A clamped stream.
   */
  public default InputStream clamp(double magnitude) {
    return map(x -> MathUtil.clamp(x, -magnitude, magnitude));
  }

  /**
   * Rate limits the stream outputs by a specified rate.
   *
   * @param rate The rate in units / s.
   * @return A rate limited stream.
   */
  public default InputStream rateLimit(double rate) {
    var limiter = new SlewRateLimiter(rate);
    return map(x -> limiter.calculate(x));
  }

  /**
   * Logs the output of this stream to networktables every time it is polled.
   *
   * <p>A new stream is returned that is identical to this stream, but publishes its output to
   * networktables every time it is polled.
   *
   * @param key The NetworkTables key to publish to.
   * @return A stream with the same output as this one.
   */
  public default InputStream log(String key) {
    DoublePublisher pub = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
    return () -> {
      double val = this.get();
      pub.set(val);
      return val;
    };
  }
}