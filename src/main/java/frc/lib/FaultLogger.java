package frc.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

// (from team 1155 but slightly modified)

/**
 * FaultLogger allows for faults and errors to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(spark); // registers a spark, periodically checking for hardware faults
 * spark.set(0.5);
 * FaultLogger.check(spark); // checks that the previous set call did not encounter an error.
 * </pre>
 */
public final class FaultLogger {
  @FunctionalInterface
  public static interface FaultReporter {
    void report();
  }

  // DATA
  private static final List<FaultReporter> faultReporters = new ArrayList<>();
  private static final Set<Fault> newFaults = new HashSet<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("Faults");
  private static final FaultsTable activeAlerts = new FaultsTable(base, "Active Faults");
  private static final FaultsTable totalAlerts = new FaultsTable(base, "Total Faults");

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    activeFaults.clear();

    faultReporters.forEach(f -> f.report());
    activeFaults.addAll(newFaults);
    newFaults.clear();

    totalFaults.addAll(activeFaults);

    activeAlerts.set(activeFaults);
    totalAlerts.set(totalFaults);
  }

  /** Clears total faults. */
  public static void clear() {
    totalFaults.clear();
    activeFaults.clear();
    newFaults.clear();
  }

  /** Clears fault suppliers. */
  public static void unregisterAll() {
    faultReporters.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    newFaults.add(fault);

    switch (fault.type()) {
      case ERROR -> DriverStation.reportError(fault.toString(), false);
      case WARNING -> DriverStation.reportWarning(fault.toString(), false);
      case INFO -> System.out.println(fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String description, FaultType type) {
    report(new Fault(description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultReporters.add(() -> supplier.get().ifPresent(FaultLogger::report));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   */
  public static void register(BooleanSupplier condition, String description, FaultType type) {
    faultReporters.add(() -> {
      if (condition.getAsBoolean()) {
        report(description, type);
      }
    });
  }

  /**
   * Registers a new TalonFX.
   * 
   * @param talonFX The TalonFX.
   */
  public static void register(TalonFX talonFX) {
    // TODO: add all necessary faults to check here
  }
}