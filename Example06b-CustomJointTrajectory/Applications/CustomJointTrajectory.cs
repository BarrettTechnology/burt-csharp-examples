using System;
using System.Diagnostics;
using System.Threading;
using Barrett.CoAP;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using Barrett.Util.ClassExtensions;

/// <summary>
/// Uses a PID controller to follow a custom joint trajectory. In this example, the
/// custom trajectory is a circle in joints 2 and 3.
/// </summary>
public class CustomJointTrajectory
{
	private RobotClient robot;
	private Barrett.KeyboardManager keyboardManager;

	const int kDof = 3;                  // number of degrees of freedom

	private Vector<float> jointPos;      // current position
	private Vector<float> jointCommand;  // commanded position
	private Vector<float> jointTorque;   // commanded torque

	// Characteristics of the circle. Start position is chosen to work for both
	// right- and left-handed workspaces.
	readonly float[] startPos = new float[] { 0.6f, 0.0f, 0.225f };
	const float radius = 0.12f;    // meters
	const float frequency = 0.4f;  // Hz

	private Barrett.Control.PidVector jointPid;
	public static readonly float[] kpJointDefault = { 45, 100,  9 };  // N-m/rad
	public static readonly float[] kiJointDefault = {  0,   0,  0 };  // N-m/rad-s
	public static readonly float[] kdJointDefault = { 12,  15,  2 };  // N-m-s/rad
	private Vector<float> kpJoint;
	private Vector<float> kiJoint;
	private Vector<float> kdJoint;
	private const float lowpassFilterFreq = 30.0f;  // rad/s
	private bool motionActive = false;

	// A linear trajectory for moving to the start point of the circle
	private Barrett.Control.LinearTrajectoryVector startTraj;

	// See Example02-HoldPosition for an explanation of the timers used in this example.
	// The circleTimer is added for generating the circular movement. This timer keeps
	// track of how long the circular trajectory has been running.
	public static readonly int controlLoopTime = 10;  // in ms
	private Stopwatch dtTimer = new Stopwatch ();
	private Stopwatch intervalTimer = new Stopwatch ();
	private Stopwatch circleTimer = new Stopwatch ();

	public CustomJointTrajectory ()
	{
		// Initialize vectors
		jointPos = Vector<float>.Build.Dense (kDof);
		jointTorque = Vector<float>.Build.Dense (kDof);
		jointCommand = Vector<float>.Build.Dense (kDof);
		kpJoint = Vector<float>.Build.DenseOfArray (kpJointDefault);
		kiJoint = Vector<float>.Build.DenseOfArray (kiJointDefault);
		kdJoint = Vector<float>.Build.DenseOfArray (kdJointDefault);

		// Set up communication with the robot and initialize force/torque to zero
		robot = new RobotClient ();
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);
		robot.SubscribeToRobotStatus (OnReceiveRobotStatus);
		robot.SendCartesianForces (Vector3.zero);

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
		jointPid = new Barrett.Control.PidVector (kpJoint, kiJoint, kdJoint, kDof, lowpassFilterFreq);

		// Set up trajectory generator
		startTraj = new Barrett.Control.LinearTrajectoryVector (kDof);

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

			if (motionActive) {
				if (!startTraj.DoneMoving) {
					// Follow the linear trajectory to the start position until it is done.
					startTraj.Update ();
					startTraj.Position.CopyTo (jointCommand);
				} else {
					// Start the timer for the circle, if it's not already started.
					if (!circleTimer.IsRunning) {
						circleTimer.Start ();
					}

					// Calculate the new joint position command. Constant in joint 0 and
					// cyclic movement in joints 1 and 2.
					float time = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
					jointCommand [0] = startPos [0];
					jointCommand [1] = radius * (Mathf.Cos (2f * Mathf.PI * frequency * time) - 1.0f) + startPos [1];
					jointCommand [2] = radius * Mathf.Sin (2f * Mathf.PI * frequency * time) + startPos [2];
				}
				jointTorque = jointPid.Update (jointCommand, jointPos, dt);
			} else {
				jointTorque.Clear ();
			}

			robot.SendCartesianForcesAndJointTorques (Vector3.zero, jointTorque.ToVector3 ())
					.Catch (e => Barrett.Logger.Debug(Barrett.Logger.CRITICAL, "Exception {0}", e))
					.Done ();

			// Calculate how long to wait until next control cycle
			Thread.Sleep (Math.Max (0, controlLoopTime - (int)intervalTimer.ElapsedMilliseconds));
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
		jointPos.FromVector3 (update.position);
	}

	/// <summary>
	/// Prints status information received from the robot.
	/// </summary>
	private void OnReceiveRobotStatus (Barrett.CoAP.MsgTypes.RobotStatus status)
	{
		Barrett.Logger.Debug (Barrett.Logger.INFO, "Handedness: {0}", status.handedness);
		Barrett.Logger.Debug (Barrett.Logger.INFO, "Outerlink: {0}", status.outerlink);
		Barrett.Logger.Debug (Barrett.Logger.INFO, "IsPatientConnected?: {0}", status.patient);
	}

	/// <summary>
	/// Unsubscribes from updates, sends a request to disable the robot, and terminates the process.
	/// </summary>
	public void Close ()
	{
		robot.UnsubscribeFromServerUpdate ();
		robot.UnsubscribeFromRobotStatus ();

		OnDisable ();
		Environment.Exit (0);
	}

	/// <summary>
	/// Sends a request to home the robot and then sleeps while waiting for that
	/// request to take effect.
	/// </summary>
	public void OnHome ()
	{
		Barrett.Logger.Debug (Barrett.Logger.WARNING, "Make sure robot is disabled and placed in the wing position." +
			"\nThen press <Enter> to continue.");
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
		motionActive = false;
		robot.SendIsEnabled (true);
	}

	/// <summary>
	/// Sends a disable request to the robot.
	/// </summary>
	public void OnDisable ()
	{
		if (motionActive) {
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
		if (motionActive) {
			Idle ();
		} else {
			motionActive = true;
			Console.WriteLine ("Moving to joint position (" + startPos [0].ToString ("f3") + ", " +
				startPos [1].ToString ("f3") + ", " + startPos [2].ToString ("f3") + ")");
			jointTorque.Clear ();
			jointPid.ResetAll ();

			// Since startPos was not declared as a Vector<float>, one is created here for
			// the input to BeginMove().
			float speed = 0.2f;
			float acceleration = 0.2f;
			startTraj.BeginMove (jointPos, Vector<float>.Build.DenseOfArray (startPos), speed, acceleration);
		}
	}

	/// <summary>
	/// Stops an active movement.
	/// </summary>
	public void Idle ()
	{
		startTraj.EndMove ();
		jointTorque.Clear ();
		jointPid.ResetAll ();
		motionActive = false;
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

