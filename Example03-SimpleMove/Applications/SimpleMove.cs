using System;
using System.Diagnostics;
using System.Threading;
using Barrett.CoAP;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using Barrett.Util.ClassExtensions;
/// <summary>
/// Uses a PID controller to move to a joint position or a tool position.
/// </summary>
public class SimpleMoveExample
{
	private RobotClient robot;
	private Barrett.KeyboardManager keyboardManager;

	const int kDof = 3;                  // number of degrees of freedom
	const int kNumDim = 3;               // number of Cartesian dimensions
	public static readonly int controlRate = 10;  // Control Rate in ms
	public static readonly float[] kpJointDefault = { 45, 100,  9 };  // N-m/rad
	public static readonly float[] kiJointDefault = {  0,   0,  0 };  // N-m/rad-s
	public static readonly float[] kdJointDefault = { 12,  15,  2 };  // N-m-s/rad

	private Vector<float> jointPos;      // current joint positions
	private Vector<float> jointTorques;  // commanded joint torques
	private Vector<float> toolPos;       // current tool position
	private Vector<float> toolForce;     // commanded tool force

	private Barrett.Control.PidVector jointPid;
	private Barrett.Control.PidVector toolPid;
	private Vector<float> kpJoint;
	private Vector<float> kiJoint;
	private Vector<float> kdJoint;
	private float kpTool = 200.0f;  // N/m
	private float kiTool = 0.0f;    // N/m-s
	private float kdTool = 10.0f;   // N-s/m
	private const float lowpassFilterFreq = 30.0f;  // rad/s
	private bool jointActive = false;
	private bool toolActive = false;

	// Position commands should be saved in vectors to be used as inputs to
	// the PID controllers.
	private Vector<float> jointCommand;
	private Vector<float> toolCommand;

	// Trajectory generators create a linear trajector with a trapezoidal velocity profile.
	// See the Burt C# library API documentation for more details about the characteristics
	// of this type of trajectory and how it is generated.
	private Barrett.Control.LinearTrajectoryVector jointTraj;
	private Barrett.Control.LinearTrajectoryVector toolTraj;

	private Stopwatch dtTimer = new Stopwatch ();
	private Stopwatch intervalTimer = new Stopwatch ();

	public SimpleMoveExample ()
	{
		// Initialize vectors
		jointPos = Vector<float>.Build.Dense (kDof);
		jointTorques = Vector<float>.Build.Dense (kDof);
		toolPos = Vector<float>.Build.Dense (kNumDim);
		toolForce = Vector<float>.Build.Dense (kNumDim);
		jointCommand = Vector<float>.Build.Dense (kDof);
		toolCommand = Vector<float>.Build.Dense (kNumDim);
		kpJoint = Vector<float>.Build.DenseOfArray (kpJointDefault);
		kiJoint = Vector<float>.Build.DenseOfArray (kiJointDefault);
		kdJoint = Vector<float>.Build.DenseOfArray (kdJointDefault);

		// Set up communication with the robot and initialize force/torque to zero
		robot = new RobotClient ();
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);
		robot.SubscribeToRobotStatus (status => {
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Handedness: {0}", status.handedness);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Outerlink: {0}", status.outerlink);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "IsPatientConnected?: {0}", status.patient);
		});

		robot.SendCartesianForces(Vector3.zero);
		robot.SendJointTorques(Vector3.zero);

		// Set up keyboard callbacks
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true);  // print key pressed
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		keyboardManager.AddKeyPressCallback ("j", MoveToJoint);
		keyboardManager.AddKeyPressCallback ("t", MoveToTool);
		keyboardManager.AddKeyPressCallback ("i", OnIdle);
		keyboardManager.AddKeyPressCallback ("p", PrintInfo);
		keyboardManager.AddKeyPressCallback ("q", Close);
		PrintUsage ();

		// Set up PID controllers
		jointPid = new Barrett.Control.PidVector (kpJoint, kiJoint, kdJoint, kDof, lowpassFilterFreq);
		toolPid = new Barrett.Control.PidVector (kpTool, kiTool, kdTool, kNumDim, lowpassFilterFreq);

		// Set up trajectory generators
		jointTraj = new Barrett.Control.LinearTrajectoryVector (kDof);
		toolTraj = new Barrett.Control.LinearTrajectoryVector (kNumDim);

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

			if (jointActive) {
				jointTraj.Update ();
				jointTorques = jointPid.Update (jointTraj.Position, jointPos, dt);
				toolForce.Clear ();
			} else if (toolActive) {
				toolTraj.Update ();	
				toolForce = toolPid.Update (toolTraj.Position, toolPos, dt);
				jointTorques.Clear ();
			} else {
				jointTorques.Clear ();
				toolForce.Clear ();
			}

			robot.SendCartesianForcesAndJointTorques (toolForce.ToVector3 (), jointTorques.ToVector3 ())
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
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tj: Move to joint position");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tt: Move to tool position");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\ti: Idle the robot (stop moving/holding)");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tp: Print state info");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Prints the current joint positions and tool positions.
	/// </summary>
	public void PrintInfo ()
	{
		Console.WriteLine ("Joint positions: {0}", jointPos.ToVector3().ToString ("f3"));
		Console.WriteLine ("Tool position: {0}", toolPos.ToVector3().ToString ("f3"));
	}

	/// <summary>
	/// Saves state information received from the robot.
	/// </summary>
	private void OnReceiveServerUpdate (Barrett.CoAP.MsgTypes.ServerUpdate update)
	{
		toolPos.FromVector3 (update.position);
		jointPos.FromVector3 (update.joint_position);
	}

	/// <summary>
	/// Unsubscribes from updates and sends a request to disable the robot.
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
		jointActive = toolActive = false;
		robot.SendIsEnabled (true);
	}

	/// <summary>
	/// Sends a disable request to the robot.
	/// </summary>
	public void OnDisable ()
	{
		OnIdle ();
		robot.SendIsEnabled (false);
	}

	/// <summary>
	/// Begins a movement to the specified joint position.
	/// </summary>
	//
	// Here, the BeginMove () command is used to start the linear trajectory:
	//
	//    jointTraj.BeginMove (jointPos, jointCommand, speed, acc);
	//
	// The two parameters speed and acc are optional. Both are scalar floats. speed is the norm
	// of the velocity vector, and acc is the norm of the acceleration vector. For joint
	// trajectories, the max speed is specified in rad/s, and the max acceleration is specified
	// in rad/s^2. For tool trajectories, the max tool speed is specified in m/s, and the tool
	// acceleration is specified in m/s^2. If left unspecified, this command defaults to using
	// a value of 0.5f for both speed and acceleration.
	public void MoveToJoint ()
	{
		if (toolActive || jointActive)
		{
			Console.WriteLine ("Press i to idle the robot before executing another move.\n");
		}
		else if (ParsePositions (ref jointCommand))
		{
			jointActive = true;
			Console.WriteLine ("Moving to joint position (" + jointCommand [0] + ", " +
				jointCommand [1] + ", " + jointCommand [2] + ")");
			jointTraj.BeginMove (jointPos, jointCommand);
			jointTorques.Clear ();
			jointPid.ResetAll ();
		}
	}

	/// <summary>
	/// Begins a movement to the specified tool position.
	/// </summary>
	//
	// Here, the BeginMove () command is used to start the linear trajectory:
	//
	//    jointTraj.BeginMove (jointPos, jointCommand, speed, acc);
	//
	// The two parameters speed and acc are optional. Both are scalar floats. speed is the norm
	// of the velocity vector, and acc is the norm of the acceleration vector. For joint
	// trajectories, the max speed is specified in rad/s, and the max acceleration is specified
	// in rad/s^2. For tool trajectories, the max tool speed is specified in m/s, and the tool
	// acceleration is specified in m/s^2. If left unspecified, this command defaults to using
	// a value of 0.5f for both speed and acceleration. For tool movements, it is currently
	// recommended to keep the speed between 0.2f and 0.7f.
	public void MoveToTool ()
	{
		if (toolActive || jointActive)
		{
			Console.WriteLine ("Press i to idle the robot before executing another move.");
		}
		else if (ParsePositions (ref toolCommand))
		{
			toolActive = true;
			Console.WriteLine ("Moving to tool position (" + toolCommand.ToVector3 ().ToString ("f3") + ")");
			toolTraj.BeginMove (toolPos, toolCommand);
			toolForce.Clear ();
			toolPid.ResetAll ();
		}
	}

	/// <summary>
	/// Stops an active movement or releases an active hold.
	/// </summary>
	public void OnIdle ()
	{
		jointTraj.EndMove ();
		jointTorques.Clear ();
		jointPid.ResetAll ();
		jointActive = false;

		toolTraj.EndMove ();
		toolForce.Clear ();
		toolPid.ResetAll ();
		toolActive = false;
	}

	/// <summary>
	/// Parses command line input for setting goal positions.
	/// </summary>
	/// <returns><c>true</c>, if positions was parsed, <c>false</c> otherwise.</returns>
	/// <param name="pos">Position.</param>
	private bool ParsePositions (ref Vector<float> pos)
	{
		String s;
		float temp;

		for (int i = 0; i < pos.Count; i++) {
			Console.Write ("Enter coordinate {0}: ", i + 1);
			s = Console.ReadLine ();
			if (float.TryParse (s, out temp)) {
				pos [i] = temp;
			} else {
				Console.WriteLine ("Invalid entry. Aborting.");
				return false;
			}
		}
		return true;
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

