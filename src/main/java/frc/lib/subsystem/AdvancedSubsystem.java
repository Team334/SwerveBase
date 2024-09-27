package frc.lib.subsystem;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.FaultsTable;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import monologue.Logged;

public abstract class AdvancedSubsystem extends SubsystemBase implements Logged, SelfChecked {
  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable = new FaultsTable(NetworkTableInstance.getDefault().getTable("Self Check"), getName() + " Faults");

  private boolean _hasErrors = false;

  public AdvancedSubsystem() {
    RobotModeTriggers.test().onFalse(runOnce(this::clearFaults));
  }

  /** Clears this subsystem's faults. */
  protected final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasErrors = false;
  }

  /** Adds a new fault under this subsystem. */
  protected final void addFault(String message, FaultType faultType) {
    Fault fault = new Fault(getName() + " Fault", message, faultType);

    _faults.add(fault);
    _faultsTable.set(_faults);

    _hasErrors = faultType == FaultType.ERROR;
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasErrors() {
    return _hasErrors;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {    
    Command selfCheck = Commands.sequence(
      Commands.runOnce(this::clearFaults), // clear all faults and hasError (also adds this subsystem as a requirement)
      selfCheck(this::addFault, this::hasErrors) // self check this subsystem
    ).withName(getName() + " Self Check");

    selfCheck.addRequirements(this);

    return selfCheck;
  }

  @Override
  public void periodic() {
    String currentCommandName = "None";

    if (getCurrentCommand() != null) {
      currentCommandName = getCurrentCommand().getName();
    }

    log("Current Command", currentCommandName);
  }
}
