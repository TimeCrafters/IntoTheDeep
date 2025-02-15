package dev.cyberarm.engine.V2;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Action;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * CyberarmEngine Version 3.0 | December 31st 2022
 * After a few years of use, it's safe to say this implementation is stable and reasonably feature complete.
 * * Added support for background tasks that run unqueued for the duration of the op mode unless stopped.
 * * Added thread-safe 'blackboard' for storing bits that need to be easily shared between states/tasks.
 *
 * CyberarmEngine Version 2.0 | October 26th 2018
 * AN Experimental reimplementation of GoldfishPi's original Engine system.
 * Designed to be easily maintainable, extensible, and easy to understand.
 */
public abstract class CyberarmEngine extends OpMode {

  public static CyberarmEngine instance;
  //Array To Hold States
  final private CopyOnWriteArrayList<CyberarmState> initStates = new CopyOnWriteArrayList<>();
  final private CopyOnWriteArrayList<CyberarmState> mainStates = new CopyOnWriteArrayList<>();
  // Array to Hold Tasks
  final private CopyOnWriteArrayList<CyberarmState> backgroundTasks = new CopyOnWriteArrayList<>();
  // HashMap to store data for States and Tasks
  final private ConcurrentHashMap<String, Object> blackboard = new ConcurrentHashMap<>();
  private int activeStateIndex = 0; // Index of currently running state
  private boolean isRunning; // Whether engine is running or not

  private final static String TAG = "PROGRAM.ENGINE";
  public boolean showStateChildrenListInTelemetry = false;

  private GamepadChecker gamepadCheckerGamepad1, gamepadCheckerGamepad2;
  private boolean useThreads = true;

  private long startTime;
  public enum STAGE {
    IDLE,
    INIT,
    INIT_LOOP,
    START,
    MAIN_LOOP,
    STOP
  }
  private STAGE stage = STAGE.IDLE;

  /**
   * Called when INIT button on Driver Station is pushed
   * ENSURE to call super.init() if you override this method
   */
  public void init() {
    stage = STAGE.INIT;

    CyberarmEngine.instance = this;
    isRunning = false;
    gamepadCheckerGamepad1 = new GamepadChecker(this, gamepad1);
    gamepadCheckerGamepad2 = new GamepadChecker(this, gamepad2);

    setup();

    isRunning = true;

    for (CyberarmState state: initStates) {
      initState(state);
    }

    for (CyberarmState state: mainStates) {
      initState(state);
    }

    // Background tasks
    for (CyberarmState task : backgroundTasks) {
      initState(task);
    }

    // Start up init_loop's first state
    activeStateIndex = 0;
    if (!initStates.isEmpty()) {
      runState(initStates.get(0));
    }
  }

  /**
   * Setup states for engine to use
   * For example:
   * <pre>
   * {@code
   *   public void setup() {
   *     addState(new TestState());
   *     addState(new AnotherState(100, 500));
   *   }
   * }
   * </pre>
   */
  public abstract void setup();

  /**
   * Called when START button on Driver Station is pushed
   * ENSURE to call super.start() if you override this method
   */
  public void start() {
    // Stop and cleanup init_loop() states
    for (CyberarmState state: initStates) {
      stopState(state);
    }
    initStates.clear();

    stage = STAGE.START;

    // Reset for main loop mainStates array
    activeStateIndex = 0;
    startTime = System.currentTimeMillis();

    if (!mainStates.isEmpty()) {
      runState(mainStates.get(0));
    }

    // Background tasks
    for (CyberarmState task : backgroundTasks) {
      runState(task);
    }
  }

  /**
   * Engine init loop
   * ENSURE to call super.init_loop() if you override this method
   */
  public void init_loop() {
    stage = STAGE.INIT_LOOP;

    execLoopFor(initStates);
  }

  /**
   * Engine main loop
   * ENSURE to call super.loop() if you override this method
   */
  public void loop() {
    stage = STAGE.MAIN_LOOP;

    execLoopFor(mainStates);
  }

  private void execLoopFor(CopyOnWriteArrayList<CyberarmState> states) {
    CyberarmState state;

    // Try to set state to the current state, if it fails assume that there are no states to run
    try {
       state = states.get(activeStateIndex);
    } catch(IndexOutOfBoundsException e) {
      // The engine is now out of states.

      telemetry.addLine("" + this.getClass().getSimpleName() + " is out of states to run!");
      telemetry.addLine();
      return;
    }

    if (!useThreads) {
      threadlessExecState(state);

      for (CyberarmState task : backgroundTasks) {
        threadlessExecState(task);
      }
    }

    // Add telemetry to show currently running state
    telemetry.addLine(
            "Running state: " +state.getClass().getSimpleName() + ". State: " +
                    (activeStateIndex + 1) + " of " + (states.size()) +
                    " (" + activeStateIndex + "/" + (states.size() - 1) + ")");

    if (showStateChildrenListInTelemetry && state.hasChildren()) {
      for(CyberarmState child: state.children) {
        telemetry.addLine("    Child: " + child.getClass().getSimpleName() + " [" + child.children.size() + "] grandchildren");
      }
    }
    telemetry.addLine();

    if (state.getHasFinished() && state.childrenHaveFinished()) {
      activeStateIndex++;

      try {
        state = states.get(activeStateIndex);
        runState(state);
      } catch(IndexOutOfBoundsException e) { /* loop will handle this in a few milliseconds */ }

    } else {
      stateTelemetry(state);

      // Background tasks
      for (CyberarmState task : backgroundTasks) {
        stateTelemetry(task);
      }
    }

    gamepadCheckerGamepad1.update();
    gamepadCheckerGamepad2.update();
  }

  /**
   * Stops every known state
   */
  @Override
  public void stop() {
    stage = STAGE.STOP;

    // Init loop states
    for (CyberarmState state: initStates) {
      stopState(state);
    }

    // Main loop states
    for (CyberarmState state: mainStates) {
      stopState(state);
    }

    // Background tasks
    for (CyberarmState task : backgroundTasks) {
      stopState(task);
    }
  }

  /**
   * Recursively calls telemetry() on states
   * @param state State to get telemetry
   */
  private void stateTelemetry(CyberarmState state) {
    if (!state.getHasFinished()) {
      state.telemetry();
    }

    for(CyberarmState childState : state.children) {
      if (!childState.getHasFinished()) {
        stateTelemetry(childState);
      }
    }
  }

  /**
   * Called when INIT button on Driver Station is pressed
   * Recursively initiates states
   * @param state State to initiate
   */
  private void initState(CyberarmState state) {
    state.init();

    for(CyberarmState childState : state.children) {
      initState(childState);
    }
  }

  /**
   * Called when programs ends or STOP button on Driver Station is pressed
   * Recursively stop states
   * @param state State to stop
   */
  public void stopState(CyberarmState state) {
    state.setHasFinished(true);
    state.stop();

    for(CyberarmState childState : state.children) {
      stopState(childState);
    }
  }

  /**
   * Recursively start up states
   * @param state State to run
   */
  protected void runState(CyberarmState state) {
    final CyberarmState finalState = state;
//    if (state.isRunning()) { return; } // Assume that we have already started running this state

    if (useThreads) {
      new Thread(() -> {
        finalState.prestart();
        finalState.start();
        finalState.startTime = System.currentTimeMillis();
        finalState.run();
      }).start();
    } else {
      finalState.prestart();
      finalState.start();
      finalState.startTime = System.currentTimeMillis();
    }

    for (CyberarmState kid : state.children) {
      runState(kid);
    }
  }

  /**
   * Recursively exec states
   * @param state State to exec
   */
  private void threadlessExecState(final CyberarmState state) {
    state.exec();

    for (CyberarmState kid : state.children) {
      threadlessExecState(kid);
    }
  }

  /**
   * Add state to init queue, will call init() on state if engine is running
   * @param state State to add to queue
   */
  public CyberarmState addInitState(CyberarmState state) {
    Log.i(TAG, "INIT Queue: Adding cyberarmState "+ state.getClass());
    initStates.add(state);

    if (isRunning()) { initState(state); }

    return state;
  }

  /**
   * Add state to main queue, will call init() on state if engine is running
   * @param state State to add to queue
   */
  public CyberarmState addState(CyberarmState state) {
    Log.i(TAG, "MAIN Queue: Adding cyberarmState "+ state.getClass());
    mainStates.add(state);

    if (isRunning()) { initState(state); }

    return state;
  }

  /**
   * Inserts state in main queue after the query state plus an offset to ensure logical insertion
   * @param query State to add state after
   * @param state State to be inserted
   * @return CyberarmState
   */
  public CyberarmState insertState(CyberarmState query, CyberarmState state) {
    int index = mainStates.indexOf(query) + query.insertOffset;
    Log.i(TAG, "MAIN Queue: Adding cyberarmState "+ state.getClass());

    mainStates.add(index, state);
    query.insertOffset++;

    if (isRunning()) { initState(state); }

    return state;
  }

  /**
   * Adds state to the main queue for the most recently added top level state as a parallel state
   * @param state State to add to last top level state
   * @return CyberarmState
   */
  public CyberarmState addParallelStateToLastState(CyberarmState state) {
    CyberarmState parentState = mainStates.get(mainStates.size() - 1);

    Log.i(TAG, "Adding parallel cyberarmState "+ state.getClass() + " to parent state " + parentState.getClass());

    parentState.addParallelState((state));

    return state;
  }

  /**
   * Adds state as a background task that is run until the opmode stops
   * background tasks are not queued, they are all started at once.
   * @param state State to add to list
   * @return CyberarmState
   */
  public CyberarmState addTask(CyberarmState state) {
    Log.i(TAG, "Adding task cyberarmState "+ state.getClass());

    backgroundTasks.add(state);

    if (isRunning()) {
      initState(state);
      runState(state);
    }

    return state;
  }

  /**
   * Retrieve value from blackboard
   * @param key String to use to look up value
   * @return Returns T of stored Object
   */
  public <T> T blackboardGet(String key) {
    return (T) blackboard.get(key);
  }

  public String blackboardGetString(String key) {
    return (String) blackboard.get(key);
  }

  public int blackboardGetInt(String key) {
    return (int) blackboard.get(key);
  }

  public long blackboardGetLong(String key) {
    return (long) blackboard.get(key);
  }

  public float blackboardGetFloat(String key) {
    return (float) blackboard.get(key);
  }

  public double blackboardGetDouble(String key) {
    return (double) blackboard.get(key);
  }

  public boolean blackboardGetBoolean(String key) {
    return (boolean) blackboard.get(key);
  }

  /**
   * Set value of key to value
   * @param key String
   * @param value Object
   * @return Returns T
   */

  public <T> T blackboardSet(String key, T value) {
    blackboard.put(key, value);

    return (T) value;
  }

  /**
   * Remove value from blackboard
   * @param key String
   * @param value Object
   * @return Returns T
   */
  public <T> T blackboardRemove(String key, T value) {
    blackboard.remove(key);

    return (T) value;
  }

  private void buttonDownForStates(CyberarmState state, Gamepad gamepad, String button) {
    state.buttonDown(gamepad, button);

    for (CyberarmState child : state.children) {
      child.buttonDown(gamepad, button);
    }
  }

  private void buttonUpForStates(CyberarmState state, Gamepad gamepad, String button) {
    state.buttonUp(gamepad, button);

    for (CyberarmState child : state.children) {
      child.buttonUp(gamepad, button);
    }
  }

  /**
   * Called by GamepadChecker when it detects that a gamepad button has been pressed
   * @param gamepad Gamepad
   * @param button String
   */
  protected void buttonDown(Gamepad gamepad, String button) {
    try {
      if (stage == STAGE.INIT_LOOP) {
        buttonDownForStates(initStates.get(activeStateIndex), gamepad, button);
      } else {
        buttonDownForStates(mainStates.get(activeStateIndex), gamepad, button);
      }
    } catch(IndexOutOfBoundsException e){
      /* loop will handle this in a few milliseconds */
    }
  }

  /**
   * Called by GamepadChecker when it detects that a gamepad button has been released
   * @param gamepad Gamepad
   * @param button String
   */
  protected void buttonUp(Gamepad gamepad, String button) {
    try {
      if (stage == STAGE.INIT_LOOP) {
        buttonUpForStates(initStates.get(activeStateIndex), gamepad, button);
      } else {
        buttonUpForStates(mainStates.get(activeStateIndex), gamepad, button);
      }
    } catch(IndexOutOfBoundsException e){
      /* loop will handle this in a few milliseconds */
    }
  }

  /**
   * This will return false while Engine.setup() is executing, and be true after.
   * @return Whether the engine main loop is running
   */
  public boolean isRunning() {
    return isRunning;
  }

  /**
   *
   * @return The index used to lookup the current state from cyberarmStates
   */
  public int getActiveStateIndex() {
    return activeStateIndex;
  }

  /**
   * Automatically populates states from a TimeCraftersConfiguration Group actions
   * requires action comments to start with an @ character followed by the class name
   * state must have a construction that takes 3 arguments: object, groupName, and actionName
   * @param configuration TimeCraftersConfiguration
   * @param packageName Package name where states are defined
   * @param object Object to pass as first argument to states constructor
   * @param objectClass Class to cast object to
   * @param groupName Group name
   */
  public void setupFromConfig(TimeCraftersConfiguration configuration, String packageName, Object object, Class<?> objectClass, String groupName) {
    setupFromConfig(configuration, packageName, object, objectClass, groupName, null);
  }

  /**
   * Automatically populates states from a TimeCraftersConfiguration Group actions
   * requires action comments to start with an @ character followed by the class name
   * state must have a construction that takes 3 arguments: object, groupName, and actionName
   * @param configuration TimeCraftersConfiguration
   * @param packageName Package name where states are defined
   * @param object Object to pass as first argument to states constructor
   * @param objectClass Class to cast object to
   * @param groupName Group name
   * @param insertAfterState state to insert states AFTER
   */
  public void setupFromConfig(TimeCraftersConfiguration configuration, String packageName, Object object, Class<?> objectClass, String groupName, CyberarmState insertAfterState) {
    CyberarmState lastState = null;
    String lastActionName = null;
    String[] lastActionNameSplit = new String[0];

    for (Action action : configuration.group(groupName).getActions()) {
      if (!action.enabled) {
        continue;
      }

      String className = null;

      if (action.comment.startsWith("@")) {
        String[] split = action.comment.split("@");
        className =  split[1].split("[ \\-]")[0];
      } else {
        throw(new RuntimeException("setupFromConfig: Action \"" + action.name + "\" in group \"" + groupName + "\" is missing magic @ in comment."));
      }

      Class<?> klass = null;
      try {
        klass = Class.forName("" + packageName + "." + className);
        if (klass !=  null) {
          String[] actionNameSplit = action.name.split("-");
          Constructor<?> constructor = klass.getConstructor(objectClass, String.class, String.class);
          CyberarmState state = (CyberarmState) constructor.newInstance(objectClass.cast(object), groupName, action.name);

          if (lastState != null && lastActionNameSplit.length == 2 && actionNameSplit.length == 2 && actionNameSplit[0].equals(lastActionNameSplit[0]))
          {
            lastState.addParallelState(state);
          } else {
            if (insertAfterState != null) {
              insertAfterState.addState(state);
            } else {
              addState(state);
            }
            lastState = state;
          }

          lastActionName = action.name;
          lastActionNameSplit = lastActionName.split("-");
        }
      } catch (ClassNotFoundException | NoSuchMethodException | IllegalAccessException e) {
        e.printStackTrace();

        RuntimeException exception = new RuntimeException(e.getMessage(), e.getCause());
        exception.setStackTrace(e.getStackTrace());

        throw(exception);
      } catch (InvocationTargetException | InstantiationException e) {
        Throwable cause = e.getCause();

        if (cause != null) {
          cause.printStackTrace();

          RuntimeException exception = new RuntimeException(cause.getMessage(), cause.getCause());
          exception.setStackTrace(cause.getStackTrace());

          throw(exception);
        } else {
          e.printStackTrace();

          RuntimeException exception = new RuntimeException(e.getMessage(), e.getCause());
          exception.setStackTrace(e.getStackTrace());

          throw(exception);
        }
      }
    }
  }

  /**
   * Disable running states in their own threads which might help with random crashes.
   */
  public void threadless() {
    useThreads = false;
  }

  /**
   * Time in milliseconds since START button was pushed
   * @return runTime
   */
  public double runTime() {
    return (System.currentTimeMillis() - startTime);
  }

  /**
   * The current stage of the engine. See {@link STAGE}
   * @return stage
   */
  public STAGE getStage() {
    return stage;
  }
}
