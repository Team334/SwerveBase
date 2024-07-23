package frc.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Alert.AlertType;


/**
 * Devices are added to this FaultLogger to have their faults alerted on elastic. (Based off teams 1155/353 FaultLogger)
 */
public final class FaultLogger {
  private static final HashMap<Integer, ArrayList<FaultReporter>> _faultReporters = new HashMap<Integer, ArrayList<FaultReporter>>();

  private static final Set<Fault> _activeFaults = new HashSet<Fault>();
  private static final Set<Fault> _totalFaults = new HashSet<Fault>();

  private static final FaultsTable _activeFaultsTable = new FaultsTable("Active Faults");
  private static final FaultsTable _totalFaultsTable = new FaultsTable("Total Faults");

  private static int _ID_COUNTER = 0;

  /** A lambda that returns an Optional with a Fault. */
  @FunctionalInterface
  public static interface FaultReporter {
    public Optional<Fault> report();
  }

  // A NetworkTable that contains faults. (can't use the Alert class since these aren't persistent)
  private static class FaultsTable {
    private final StringArrayPublisher errors;
    private final StringArrayPublisher warnings;
    private final StringArrayPublisher infos;

    public FaultsTable(String name) {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
      table.getStringTopic(".type").publish().set("Alerts"); // set to alerts widget type
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    /** Sets the faults to show on this table and DriverStation. */
    public void set(Set<Fault> faults) {
      errors.set(filteredStrings(faults, FaultType.ERROR));
      warnings.set(filteredStrings(faults, FaultType.WARNING));
      infos.set(filteredStrings(faults, FaultType.INFO));
    }

    // filters a list of faults into strings to display on the widget
    private String [] filteredStrings(Set<Fault> faults, FaultType typeFilter) {
      return faults.stream()
      .filter(f -> f.type == typeFilter)
      .map(Fault::toString)
      .toArray(String[]::new);
    }
  }
  
  /** Represents the classification of the fault. */
  public static enum FaultType {
    ERROR,
    WARNING,
    INFO
  }

  /** Represents a fault from a device. */
  public static record Fault(String deviceName, String description, FaultType type) {
    @Override
    public String toString() {
      return deviceName + ": " + description;
    }

    /** Returns the Alert type of this fault for the Alerts widget. */
    public AlertType alertType() {
      switch (type) {
        case ERROR:
          return AlertType.ERROR;

        case WARNING:
          return AlertType.WARNING;

        case INFO:
          return AlertType.INFO;
        
        default:
          return null;
      }
    }
  }

  /** Updates the FaultLogger by checking for and displaying faults. */
  public static void update() {
    _activeFaults.clear();

    // call each reporter and report faults they returned
    ArrayList<FaultReporter> allReporters = new ArrayList<>();
    _faultReporters.values().forEach(allReporters::addAll);
    allReporters.forEach(r -> r.report().ifPresent(FaultLogger::report));
    
    _totalFaults.addAll(_activeFaults); // adds all active faults to total faults (if there are new ones)

    // update the faults network table/driver station with all the faults
    _activeFaultsTable.set(_activeFaults);
    _totalFaultsTable.set(_totalFaults);
  }

  /** Reports a new Fault to the FaultLogger. */
  public static void report(Fault fault) {
    _activeFaults.add(fault);

    switch (fault.type) {
      case ERROR:
        DriverStation.reportError(fault.toString(), false);
        break;
    
      case WARNING:
        DriverStation.reportWarning(fault.toString(), false);
        break;

      case INFO:
        System.out.println(fault.toString());
        break;
      
      default:
        break;
    }
  }

  /** Reports a new Fault to the FaultLogger. */
  public static void report(String deviceName, String description, FaultType faultType) {
    report(new Fault(deviceName, description, faultType));
  }

  // adds a new fault reporter for a device
  private static void addReporter(int deviceId, FaultReporter reporter) {
    _faultReporters.putIfAbsent(deviceId, new ArrayList<FaultReporter>());
    _faultReporters.get(deviceId).add(reporter);
  }

  // returns a device's fault reporters
  private static ArrayList<FaultReporter> getReporters(int deviceId) {
    return _faultReporters.getOrDefault(deviceId, new ArrayList<FaultReporter>());
  }

  /** Returns the active faults for a device based on its fault reporters. */
  public static ArrayList<Fault> getFaults(int deviceId) {
    ArrayList<Fault> faults = new ArrayList<Fault>();

    getReporters(deviceId).forEach(r -> r.report().ifPresent(faults::add));

    return faults;
  }

  /** 
   * Registers a new TalonFX to the FaultLogger.
   * 
   * @returns The id generated for this device.
   */
  public static int register(String deviceName, TalonFX talonFX) {
    int id = _ID_COUNTER ++;

    addReporter(id, () -> {
      if (talonFX.getFault_BootDuringEnable().getValue()) {
        return Optional.of(new Fault(deviceName, "Boot during enable.", FaultType.ERROR));
      } return Optional.empty();
    });

    return id;
  }
}
