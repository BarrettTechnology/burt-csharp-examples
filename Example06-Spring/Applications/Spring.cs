using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Barrett.CoAP;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using RSG;
using Barrett.Util.ClassExtensions;

/// <summary>
/// Creates virtual springs in joint space and Cartesian space by taking keypress user input.
/// Each spring's location is determined by the current location of the joint/tool when the spring is created.
/// The springs can be moved by disabling and re-enabling in a different location.
/// </summary>
public class SpringExample
{
	private RobotClient robot;
	private Barrett.KeyboardManager keyboardManager;

	public static readonly int kDof = 3;     // degrees of freedom
	public static readonly int kNumDim = 3;  // Cartesian dims
	public static readonly float[] kpJointDefault = { 20, 50,  5 };  // N-m/rad

	private Vector<float> jointPos;        // current joint positions
	private Vector<float> jointSpringPos;  // joint position when joint spring is/was enabled for each joint
										   // this stores positions for each joint spring in one kDof length array
										   // by only storing the relevant joint's position when a spring is activated
										   // e.g. when a spring is created on joint 1,
										   // just jointSpringPos[0] is updated
	private Vector<float> jointTorques;    // commanded joint torques
	private Vector<float> toolPos;         // current tool position
	private Vector<float> cartSpringPos;   // tool position when cartesian spring is/was enabled for each axis
										   // this uses the same logic as jointSpringPos; when a spring is created
										   // normal to the x axis, only cartSpringPos[0] is updated
	private Vector<float> toolForce;       // commanded tool force
	private Vector<float> kpJoint;  // Gain for joint spring
	private float kpTool = 200.0f;  // Gain for Cartesian spring in N/m
	private const float lowpassFilterFreq = 30.0f;  // rad/s
	private bool [] jointSpringEnabled = new bool[kDof];
	private bool [] cartesianSpringEnabled = new bool[kNumDim];

	// Timers used for control. controlLoopTime specifies the desired time for each control cycle.
	//
	// The intervalTimer measures how long the control cycle took to execute so the program knows
	// how long to sleep before starting the next one. It needs to be reset at the end of each
	// cycle to count properly. This example uses millisecond precision in the Sleep call at the
	// end of the control loop.
	public static readonly int controlLoopTime = 10;  // in ms
	private Stopwatch intervalTimer = new Stopwatch ();

	public SpringExample ()
	{
		// Initialize vectors. This is required to set the length of the vectors before they
		// are used. Build.Dense initializes all the elements to 0. Build.DenseOfArray sets
		// the vector length to the length of the array and the values equal to the values
		// in the array.
		jointPos = Vector<float>.Build.Dense (kDof);
		jointSpringPos = Vector<float>.Build.Dense (kDof);
		jointTorques = Vector<float>.Build.Dense (kDof);
		toolPos = Vector<float>.Build.Dense (kNumDim);
		cartSpringPos = Vector<float>.Build.Dense (kNumDim);
		toolForce = Vector<float>.Build.Dense (kNumDim);
		kpJoint = Vector<float>.Build.DenseOfArray (kpJointDefault);

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
		Console.WriteLine("Spring Example Program");
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true);  // print key pressed
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		keyboardManager.AddKeyPressCallback ("1", JointSpring);
		keyboardManager.AddKeyPressCallback ("2", JointSpring);
		keyboardManager.AddKeyPressCallback ("3", JointSpring);
		keyboardManager.AddKeyPressCallback ("x", CartesianSpring);
		keyboardManager.AddKeyPressCallback ("y", CartesianSpring);
		keyboardManager.AddKeyPressCallback ("z", CartesianSpring);
		keyboardManager.AddKeyPressCallback ("r", RemoveAllSprings);
		keyboardManager.AddKeyPressCallback ("p", PrintInfo);
		keyboardManager.AddKeyPressCallback ("q", Close);
		PrintUsage ();

		// Loop: calculate forces/torques at every timestep based on current
		// state feedback from the robot.
		bool running = true;
		intervalTimer.Reset ();

		while (running) {
			running = ReadKeyPress ();
			jointTorques.Clear ();
			toolForce.Clear ();
			jointTorques = CalcJointSpringTorques ();
			toolForce = CalcCartesianSpringForces ();

			robot.SendCartesianForcesAndJointTorques (toolForce.ToVector3 (), jointTorques.ToVector3 ())
				.Catch (e => Barrett.Logger.Debug(Barrett.Logger.CRITICAL, "Exception {0}", e))
				.Done ();

			Thread.Sleep (Math.Max (0, controlLoopTime - (int)intervalTimer.ElapsedMilliseconds));
			intervalTimer.Restart ();
		}
	}

	/// <summary>
	/// Disables all joint and cartesian springs.
	/// </summary>
	public void RemoveAllSprings()
	{ 
		for (int i = 0; i < kDof; i++) { jointSpringEnabled [i] = false; };
		for (int j = 0; j < kNumDim; j++) { cartesianSpringEnabled[j] = false; };
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
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\t1: Toggle joint 1 spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\t2: Toggle joint 2 spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\t3: Toggle joint 3 spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tx: Toggle x-axis spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\ty: Toggle y-axis spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tz: Toggle z-axis spring");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tr: Remove all springs");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tp: Print state info");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Prints the current joint positions, tool positions, and enabled springs.
	/// </summary>
	public void PrintInfo ()
	{
		Console.WriteLine ("Joint positions: {0}", jointPos.ToVector3().ToString ("f3"));
		Console.WriteLine ("Tool position: {0}", toolPos.ToVector3().ToString ("f3"));
		// Print out which spring(s) are enabled, or that no springs are enabled.
		bool flag = false;
		for (int i = 0; i < kDof; i++) {
			if (jointSpringEnabled [i] == true) {
				Console.WriteLine ("Joint {0} spring enabled.", i);
				flag = true;
			}
		}
		for (int i = 0; i < kNumDim; i++) {
			if (cartesianSpringEnabled [i] == true) {
				Console.WriteLine ("{0}-axis spring enabled.", ConvertAxisNumberToLetter(i));
				flag = true;
			}
		}
		if (flag == false) {
			Console.WriteLine("No springs are enabled.");
		}
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
		RemoveAllSprings ();
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
	/// Creates a virtual spring on the specified joint. The spring will start in the current location
	/// of that joint and go to the end of the robot's range of motion.
	/// 
	/// When KeyboardManager calls this method, the key press that triggered it is passed as an argument.
	/// </summary>
	public void JointSpring (string joint = "1")
	{
		int jointNum = Convert.ToInt32 (joint) - 1;
		if (jointNum < 0 || jointNum > kDof) { jointNum = 0; }  // set to joint 1 if out of range
		if (AnyCartesianSpringEnabled()) {
			Console.WriteLine ("Press r to remove all Cartesian springs before activating joint spring.");
			return;
		}
		if (!jointSpringEnabled [jointNum]) {
			jointPos.CopyTo (jointSpringPos);
			Console.WriteLine ("Enabling joint spring on joint {0}", joint);
			jointTorques.Clear ();
		} else {
			Console.WriteLine ("Disabling joint {0} spring.", joint);
		}
		jointSpringEnabled [jointNum] = !jointSpringEnabled [jointNum];
	}

	/// <summary>
	/// Creates a virtual spring on the plane normal to the specified axis, that starts at the current tool position.
	/// </summary>
	public void CartesianSpring (string axis = "z")
	{
		if (AnyJointSpringEnabled()) {
			Console.WriteLine ("Press r to remove all joint springs before activating a Cartesian spring.\n");
			return;
		}
		int axisNum = ConvertAxisLetterToNumber (axis);
		if (!cartesianSpringEnabled [axisNum]) {
			cartSpringPos[axisNum] = toolPos[axisNum];
			Console.WriteLine ("Enabling Cartesian spring along {0}-axis.", axis);
			toolForce.Clear ();
		} else {
			Console.WriteLine ("Disabling Cartesian spring along {0}-axis.", axis);
		}
		cartesianSpringEnabled[axisNum] = !cartesianSpringEnabled[axisNum];
	}

	/// <summary>
	/// Check if there is a spring enabled on any of the joints
	/// </summary>
	/// <returns><c>true</c>, if there are any joint springs enabled, <c>false</c> otherwise.</returns>
	public bool AnyJointSpringEnabled()
	{
		bool anyJointSpringEnabled = false;
		for (int i = 0; i < kDof; i++) {
			if (jointSpringEnabled [i] == true) {
				anyJointSpringEnabled = true;
			}
		}
		return anyJointSpringEnabled;
	}

	/// <summary>
	/// Check if there are any Cartesian springs enabled
	/// </summary>
	/// <returns><c>true</c>, if there are any Cartesian springs enabled, <c>false</c> otherwise.</returns>
	public bool AnyCartesianSpringEnabled()
	{
		bool anyCartesianSpringEnabled = false;
		for (int i = 0; i < kNumDim; i++) {
			if (cartesianSpringEnabled [i] == true) {
				anyCartesianSpringEnabled = true;
			}
		}
		return anyCartesianSpringEnabled;
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

	/// <summary>
	/// Calculates the joint spring torques based on the current joint positions and joint spring locations.
	/// </summary>
	/// <returns>The joint spring torques.</returns>
	private Vector<float> CalcJointSpringTorques ()
	{
		Vector<float> jointSpringTorques = Vector<float>.Build.Dense (kDof);
		for (int j = 0; j < kDof; j++) {
			if (jointSpringEnabled [j]) {
				float dist = jointPos [j] - jointSpringPos [j];
				if (dist > 0) {  // If the joint is within the spring range
					jointSpringTorques [j] = -(dist) * kpJoint [j];
				}
			}
		}
		return jointSpringTorques;
	}

	/// <summary>
	/// Calculates the Cartesian spring forces based on the current tool position and Cartesian spring locations.
	/// </summary>
	/// <returns>The cartesian spring forces.</returns>
	private Vector<float> CalcCartesianSpringForces()
	{
		Vector<float> normal = Vector<float>.Build.Dense (kNumDim);
		Vector<float> cartSpringForces = Vector<float>.Build.Dense (kNumDim);
		for (int i = 0; i < kNumDim; i++)
		{
			if (cartesianSpringEnabled [i]) {
				normal [i] = 1.0f;
				float penetration_depth = normal.DotProduct (cartSpringPos.Subtract(toolPos));
				if (penetration_depth > 0) {
					cartSpringForces [i] = kpTool * penetration_depth;   
				}
				normal[i] = 0.0f;
			}
		}
		return cartSpringForces;
	}

	private string ConvertAxisNumberToLetter(int axisNumber)
	{
		string axisLetter = "";
		switch (axisNumber)
		{
		case 0:
			axisLetter = "x";
			break;
		case 1:
			axisLetter = "y";
			break;
		case 2:
			axisLetter = "z";
			break;
		default:
			Console.WriteLine ("Invalid axis number.");
			break;
		}
		return axisLetter;
	}

	private int ConvertAxisLetterToNumber(string axisLetter)
	{
		int axisNumber = 0;
		switch (axisLetter)
		{
		case "x":
			axisNumber = 0;
			break;
		case "y":
			axisNumber = 1;
			break;
		case "z":
			axisNumber = 2;
			break;
		default:
			Console.WriteLine ("Invalid axis.");
			break;
		}
		return axisNumber;
	}

	/// <summary>
	/// Saves state information received from the robot.
	/// </summary>
	private void OnReceiveServerUpdate (Barrett.CoAP.MsgTypes.ServerUpdate update)
	{
		toolPos.FromVector3 (update.position);
		jointPos.FromVector3 (update.joint_position);
	}
}
