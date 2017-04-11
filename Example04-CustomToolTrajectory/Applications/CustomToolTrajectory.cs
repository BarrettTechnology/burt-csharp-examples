using System;
using System.Diagnostics;
using System.Threading;
using Barrett.CoAP;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using Barrett.Util.ClassExtensions;
/// <summary>
/// Uses a PID controller to follow a custom tool trajectory. In this example, the
/// custom trajectory is a circle in the horizontal plane.
/// </summary>
public class CustomToolTrajectory
{
	private RobotClient robot;
	private Barrett.KeyboardManager keyboardManager;

	const int kDof = 3;                  // number of degrees of freedom
	const int kNumDim = 3;               // number of Cartesian dimensions
	public static readonly int controlRate = 5;  // Control Rate in ms

	private Vector<float> toolPos;      // current position
	private Vector<float> toolCommand;  // commanded position
	private Vector<float> toolForce;    // commanded force

	// Characteristics of the circle. Start position is chosen to work for both
	// right- and left-handed workspaces.
	//TODO(ab) check the x coordinate
	readonly float[] startPos = new float[] { 0.2f, 0.0f, 0.225f };
	const float amplitude = 0.1f;  // meters
	const float frequency = 1.0f;  // rad/s

	private Barrett.Control.PidVector toolPid;
	private float kpTool = 200.0f;
	private float kiTool = 0.0f;
	private float kdTool = 10.0f;
	private const float filterFreq = 30.0f;
	private bool active = false;

	// A linear trajectory for moving to the start point of the circle
	private Barrett.Control.LinearTrajectoryVector startTraj;

	private Stopwatch dtTimer = new Stopwatch ();
	private Stopwatch intervalTimer = new Stopwatch ();
	private Stopwatch circleTimer = new Stopwatch ();

	public CustomToolTrajectory ()
	{
		// Initialize vectors
		toolPos = Vector<float>.Build.Dense (kNumDim);
		toolForce = Vector<float>.Build.Dense (kNumDim);
		toolCommand = Vector<float>.Build.Dense (kNumDim);

		// Set up communication with the robot and initialize force/torque to zero
		robot = new RobotClient ();
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);
		robot.SubscribeToRobotStatus (OnReceiveRobotStatus);
		robot.SendCartesianForces(Vector3.zero);

		// Set up keyboard callbacks
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true);  // print key pressed
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		keyboardManager.AddKeyPressCallback ("s", StartStop);
		keyboardManager.AddKeyPressCallback ("q", Close);
		PrintUsage ();

		// Set up PID controller
		toolPid = new Barrett.Control.PidVector (kpTool, kiTool, kdTool, kNumDim, filterFreq);

		// Set up trajectory generator
		startTraj = new Barrett.Control.LinearTrajectoryVector (kNumDim);

		// Start the dtTimer
		dtTimer.Reset ();
		dtTimer.Start ();

		// Loop: calculate forces/torques at every timestep based on current
		// state feedback from the robot.
		bool running = true;
		intervalTimer.Reset ();
		while (running) {
			running = ReadKeyPress ();

			float dt = (float) dtTimer.ElapsedTicks / (float) Stopwatch.Frequency;
			dtTimer.Restart ();

			if (active) {
				if (!startTraj.DoneMoving) {
					// Follow the linear trajectory to the start position until it is done.
					startTraj.Update ();
					startTraj.Position.CopyTo (toolCommand);
				} else {
					// Start the timer for the circle, if it's not already started.
					if (!circleTimer.IsRunning) {
						circleTimer.Start ();
					}

					// Calculate the new tool position command. Constant in the z axis and
					// circular movement in the xy plane.
					float time = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
					toolCommand [0] = amplitude * (Mathf.Cos (frequency * time) - 1.0f) + startPos [0];
					toolCommand [1] = amplitude * Mathf.Sin (frequency * time) + startPos [1];
					toolCommand [2] = startPos [2];
				}
				toolForce = toolPid.Update (toolCommand, toolPos, dt);
			} else {
				toolForce.Clear ();
			}

			robot.SendCartesianForces (toolForce.ToVector3 ())
					.Catch (e => Barrett.Logger.Debug(Barrett.Logger.CRITICAL, "Exception {0}", e))
					.Done ();

			// Calculate how long to wait until next control cycle
			Thread.Sleep (Math.Max (0, controlRate - (int)intervalTimer.ElapsedMilliseconds));
			intervalTimer.Restart ();
		}
	}

	/// <summary>
	/// Prints the usage instructions.
	/// </summary>
	public void PrintUsage ()
	{
		Barrett.Logger.Debug (Barrett.Logger.INFO, "USAGE:");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\th: Send Request to home the robot");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\te: Send Enable Request to the robot");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\td: Send Disable Request to the robot");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\ts: Start/stop movement");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Saves state information received from the robot.
	/// </summary>
	private void OnReceiveServerUpdate (Barrett.CoAP.MsgTypes.ServerUpdate update)
	{
		toolPos.FromVector3 (update.position);
	}

	/// <summary>
	/// Prints status information received from the robot.
	/// </summary>
	private void OnReceiveRobotStatus (Barrett.CoAP.MsgTypes.RobotStatus status)
	{
		Barrett.Logger.Debug(Barrett.Logger.INFO, "Handedness: {0}", status.handedness);
		Barrett.Logger.Debug(Barrett.Logger.INFO, "Outerlink: {0}", status.outerlink);
		Barrett.Logger.Debug(Barrett.Logger.INFO, "IsPatientConnected?: {0}", status.patient);
	}

	/// <summary>
	/// Unsubscribes from updates and disables the robot.
	/// </summary>
	public void Close ()
	{
		robot.UnsubscribeFromServerUpdate ();
		robot.UnsubscribeFromRobotStatus ();

		OnDisable ();
	}

	/// <summary>
	/// Sends a request to home the robot and then sleeps while waiting for that
	/// request to take effect.
	/// </summary>
	public void OnHome ()
	{
		Barrett.Logger.Debug (Barrett.Logger.WARNING, "Make sure robot is disabled and placed in the wing position." +
			" Then press enter to continue.");
		Console.ReadLine ();
		robot.SendIsHomed (false);
		Thread.Sleep (50);
		robot.SendIsHomed (true);
		Thread.Sleep (50);
		Barrett.Logger.Debug (Barrett.Logger.INFO, "Homing complete.");
	}

	/// <summary>
	/// Sends an enable request to the robot.
	/// </summary>
	public void OnEnable ()
	{
		active = false;
		robot.SendIsEnabled (true);
	}

	/// <summary>
	/// Sends a disable request to the robot.
	/// </summary>
	public void OnDisable ()
	{
		if (active) {
			Idle ();
		}
		robot.SendIsEnabled (false);
	}

	/// <summary>
	/// Begins the movement. First the robot will move to the start position, then it will move in a circle
	/// until stopped.
	/// </summary>
	public void StartStop ()
	{
		if (active) {
			Idle ();
		} else {
			active = true;
			Console.WriteLine ("Moving to tool position (" + startPos [0].ToString ("f3") + ", " +
				startPos [1].ToString ("f3") + ", " + startPos [2].ToString ("f3") + ")");
			toolForce.Clear ();
			toolPid.ResetAll ();

			// Since startPos was not declared as a Vector<float>, one is created here for
			// the input to BeginMove().
			startTraj.BeginMove (toolPos, Vector<float>.Build.DenseOfArray (startPos), 0.2f, 0.2f);
		}
	}

	/// <summary>
	/// Stops an active movement.
	/// </summary>
	public void Idle ()
	{
		startTraj.EndMove ();
		toolForce.Clear ();
		toolPid.ResetAll ();
		active = false;
		circleTimer.Reset ();
	}

	/// <summary>
	/// Reads the key press.
	/// </summary>
	public bool ReadKeyPress ()
	{
		if (Console.KeyAvailable) {
			string keyPressed = Console.ReadKey (false).KeyChar.ToString ();
			keyboardManager.HandleKeyPress (keyPressed);
		}
		return true;
	}
}

