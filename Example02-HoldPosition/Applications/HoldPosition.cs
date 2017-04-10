using System;
using System.Diagnostics;
using System.Threading;
using Barrett.CoAP;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using RSG;
using Barrett.Util.ClassExtensions;

/// <summary>
/// Uses a PID controller to hold joint position or tool position.
/// </summary>
public class HoldPositionExample
{
	private RobotClient robot;
	private Barrett.KeyboardManager keyboardManager;

	public static readonly int kDof = 3;     // degrees of freedom
	public static readonly int kNumDim = 3;  // Cartesian dims
	public static readonly int _controlLoopTime = 10;  // in ms
	public static readonly float[] kpJointDefault = { 45, 100,  9 };
	public static readonly float[] kiJointDefault = {  0,   0,  0 };
	public static readonly float[] kdJointDefault = { 12,  15,  2 };

	private Vector<float> jointPos;      // current joint positions
	private Vector<float> jointHoldPos;  // joint hold position command
	private Vector<float> jointTorques;  // commanded joint torques
	private Vector<float> toolPos;       // current tool position
	private Vector<float> toolHoldPos;   // tool hold position command
	private Vector<float> toolForce;     // commanded tool force

	private Barrett.Control.PidVector jointPid;
	private Barrett.Control.PidVector toolPid;
	private Vector<float> kpJoint;
	private Vector<float> kiJoint;
	private Vector<float> kdJoint;
	private float kpTool = 200.0f;
	private float kiTool = 0.0f;
	private float kdTool = 10.0f;
	private const float filterFreq = 30.0f;
	private bool jointHolding = false;
	private bool toolHolding = false;
	private Stopwatch _dtTimer = new Stopwatch ();
	private Stopwatch _intervalTimer= new Stopwatch ();

	public HoldPositionExample ()
	{
		// Initialize vectors. This is required to set the length of the vectors before they
		// are used. Build.Dense initializes all the elements to 0. Build.DenseOfArray sets
		// the vector length to the length of the array and the values equal to the values
		// in the array.
		jointPos = Vector<float>.Build.Dense (kDof);
		jointHoldPos = Vector<float>.Build.Dense (kDof);
		jointTorques = Vector<float>.Build.Dense (kDof);
		toolPos = Vector<float>.Build.Dense (kNumDim);
		toolHoldPos = Vector<float>.Build.Dense (kNumDim);
		toolForce = Vector<float>.Build.Dense (kNumDim);
		kpJoint = Vector<float>.Build.DenseOfArray (kpJointDefault);
		kiJoint = Vector<float>.Build.DenseOfArray (kiJointDefault);
		kdJoint = Vector<float>.Build.DenseOfArray(kdJointDefault);

		// Set up communication with the robot.
		robot = new RobotClient ();
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);

		robot.SubscribeToRobotStatus (status => {
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Handedness: {0}", status.handedness);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Outerlink: {0}", status.outerlink);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "IsPatientConnected?: {0}", status.patient);
		});

		// Initialize force/torque inputs to zero. Here, both are initialized separately. Alternately,
		// both can be set in a single command with
		//    robot.SendCartesianForcesAndJointTorques (Vector3.zero, Vector3.zero);
		robot.SendCartesianForces (Vector3.zero);
		robot.SendJointTorques (Vector3.zero);

		// Set up keyboard callbacks
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true);  // print key pressed
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		keyboardManager.AddKeyPressCallback ("j", HoldJointPosition);
		keyboardManager.AddKeyPressCallback ("t", HoldToolPosition);
		keyboardManager.AddKeyPressCallback ("p", PrintInfo);
		keyboardManager.AddKeyPressCallback ("q", Close);
		PrintUsage ();

		// Set up PID controllers
		jointPid = new Barrett.Control.PidVector (kpJoint, kiJoint, kdJoint, kDof, filterFreq);
		toolPid = new Barrett.Control.PidVector (kpTool, kiTool, kdTool, kNumDim, filterFreq);

		// Start the _dtTimer
		_dtTimer.Reset ();
		_dtTimer.Start ();

		// Loop: calculate forces/torques at every timestep based on current
		// state feedback from the robot.
		bool running = true;
		_intervalTimer.Reset ();

		while (running) {
			running = ReadKeyPress ();

			float dt = (float)_dtTimer.ElapsedTicks / (float)Stopwatch.Frequency;
			_dtTimer.Restart ();
			if (jointHolding) {
				jointTorques = jointPid.Update (jointHoldPos, jointPos, dt);
				toolForce.Clear ();
			} else if (toolHolding) {
				toolForce = toolPid.Update (toolHoldPos, toolPos, dt);
				jointTorques.Clear ();
			} else {
				jointTorques.Clear ();
				toolForce.Clear ();
			}

			robot.SendCartesianForcesAndJointTorques (toolForce.ToVector3 (), jointTorques.ToVector3 ())
				.Catch (e => Barrett.Logger.Debug(Barrett.Logger.CRITICAL, "Exception {0}", e))
				.Done ();

			Thread.Sleep (Math.Max (0, _controlLoopTime - (int)_intervalTimer.ElapsedMilliseconds));
			_intervalTimer.Restart ();
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
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tj: Toggle hold joint position");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tt: Toggle hold tool position");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tp: Print state info");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Prints the current joint positions and tool positions.
	/// </summary>
	public void PrintInfo ()
	{
		Console.WriteLine ("Joint positions: {0}", jointPos.ToVector3());
		Console.WriteLine ("Tool position: {0}", toolPos.ToVector3());
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
		jointHolding = toolHolding = false;
		robot.SendIsEnabled (true);
	}

	/// <summary>
	/// Sends a disable request to the robot.
	/// </summary>
	public void OnDisable ()
	{
		robot.SendIsEnabled (false);
	}

	/// <summary>
	/// Holds the current joint position or releases and active hold.
	/// </summary>
	public void HoldJointPosition ()
	{
		if (toolHolding) {
			Console.WriteLine ("Press t to release tool hold before activating joint hold.\n");
			return;
		}
		if (!jointHolding) {
			jointPos.CopyTo (jointHoldPos);
			Console.WriteLine ("Holding joint position at {0}", jointHoldPos.ToVector3());
			jointTorques.Clear ();
			jointPid.ResetAll ();
		} else {
			Console.WriteLine ("Releasing joint hold position.");
		}
		jointHolding = !jointHolding;
	}

	/// <summary>
	/// Holds the current tool position or releases and active hold.
	/// </summary>
	public void HoldToolPosition ()
	{
		if (jointHolding) {
			Console.WriteLine ("Press j to release joint hold before activating tool hold.\n");
			return;
		}

		if (!toolHolding) {
			toolPos.CopyTo (toolHoldPos);
			Console.WriteLine ("Holding tool position at {0}", toolHoldPos.ToVector3());
			toolForce.Clear ();
			toolPid.ResetAll ();
		} else {
			Console.WriteLine ("Releasing tool hold position.");
		}
		toolHolding = !toolHolding;
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
