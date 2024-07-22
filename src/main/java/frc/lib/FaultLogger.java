package frc.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * Devices are added to this FaultLogger to have their faults alerted on elastic. (Based of teams 1155/353 FaultLogger)
 */
public final class FaultLogger {
  private static final HashMap<Integer, ArrayList<Supplier<Fault>>> _faultReporters = new HashMap<Integer, ArrayList<Supplier<Fault>>>();

  private static final ArrayList<Fault> _activeFaults = new ArrayList<Fault>();

  private static final FaultsTable _faultsTable = new FaultsTable();

  // The NetworkTable that contains this logger's active faults. (can't use Alert class since these aren't persistent)
  private static class FaultsTable {
    private final StringArrayPublisher errors;
    private final StringArrayPublisher warnings;
    private final StringArrayPublisher infos;

    public FaultsTable() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("Faults");
      table.getStringTopic(".type").publish().set("Alerts"); // set to alerts widget type
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    /** Sets the faults to show on this table and DriverStation. */
    public void set(ArrayList<Fault> faults) {
      errors.set(filteredStrings(faults, FaultType.ERROR));
      warnings.set(filteredStrings(faults, FaultType.WARNING));
      infos.set(filteredStrings(faults, FaultType.INFO));
    }

    // filters a list of faults into strings to display on the widget and shows those strings in driver station as well
    private String [] filteredStrings(ArrayList<Fault> faults, FaultType typeFilter) {
      List<String> filteredFaults = faults.stream()
      .filter(f -> f.type == typeFilter)
      .map(Fault::toString)
      .toList();

      filteredFaults.forEach((f) -> {
        switch (typeFilter) {
          case ERROR:
            DriverStation.reportError(f, false);
            break;
        
          case WARNING:
            DriverStation.reportWarning(f, false);
            break;

          case INFO:
            System.out.println(f);
            break;
          
          default:
            break;
        }
      });

      return filteredFaults.toArray(String[]::new);
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
  }

  /** Updates the FaultLogger by checking for and displaying faults. */
  public static void update() {
    // refresh active faults
    _activeFaults.clear();
    _faultReporters.values().forEach(reporters -> reporters.forEach(r -> {
      if (r.get() != null) _activeFaults.add(r.get());
    }));

    // update the faults network table/driver station with all the current active faults
    _faultsTable.set(_activeFaults);
  }

  // adds a new fault reporter to a device
  private static void addReporter(int deviceId, Supplier<Fault> reporter) {

  }

  /** Returns the fault reporters belonging to a device. */
  public static ArrayList<Supplier<Fault>> getReporters(int deviceId) {
    if (!_faultReporters.containsKey(deviceId)) return null;

    return _faultReporters.get(deviceId);
  }

  /** Registers a new TalonFX to the FaultLogger. */
  public static void register(TalonFX talonFX) {

  }
}
