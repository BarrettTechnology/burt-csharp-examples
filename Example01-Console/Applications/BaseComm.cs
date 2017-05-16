using System;
using System.Text;
using System.Collections;
using System.Threading;
using MsgPack;
using Barrett.CoAP;
using Barrett.CoAP.MsgTypes;
using UnityEngine;

/// <summary>
/// This is a basic robot communication example that introduces how to run the burt from C#.
/// It covers how to use key presses to enable and disable the robot, home the robot, and print state information.
///
/// In order to accomplish this, the code uses a KeyboardManager to call a function based on a keypress and
/// a RobotClient to send and receive data from the robot such as current position.
/// 
/// When you run this program, you will first want to home the robot by pressing 'h'. You can then try enabling and
/// disabling the robot with 'e' and 'd'. When the arm is enabled, you will hear the motors turn on and you can move
/// the robot freely. The robot will compensate for it's own weight, and will stay in a position that you move it to.
/// When you disable the robot, it will fall slowly unless it is in a stable position.
/// </summary>
public class BaseCommExample
{	
	public RobotClient robot;
	public Barrett.KeyboardManager keyboardManager;

	/// <summary>
	/// Initializes a new instance of the <see cref="BaseComm"/> class.
	/// </summary>
	public BaseCommExample ()
	{
		// Set up communication with the robot and initialize force to zero
		robot = new RobotClient ();
		var version = robot.GetVersion ();
		Barrett.Logger.Debug (Barrett.Logger.INFO, version.ToString ());
		robot.SendCartesianForces (Vector3.zero);
	
		// Set up keyboard callbacks. PrintUsage needs to be updated when these are changed.
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true); // print key pressed
		keyboardManager.AddKeyPressCallback ("q", Close);  // this will call the Close method when 'q' is pressed
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		keyboardManager.AddKeyPressCallback ("t", SubscribeToUpdate);
		PrintUsage ();

		// Loop: send zero force at every timestep.
		bool running = true;
		while (running) {
			running = ReadKeyPress ();
			Thread.Sleep (50);
			robot.SendCartesianForces (Vector3.zero);
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
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tt: Subscribe to robot state updates");
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Handles state information received from the robot. Prints tool position
	/// and velocity.
	/// </summary>
	public void OnReceiveServerUpdate (ServerUpdate update)
	{
		Console.WriteLine ("Position {0}", update.position);
		Console.WriteLine ("Velocity {0}", update.velocity);
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
	/// 
	/// Homing the robot allows it to calibrate itself while in a known position.
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
	/// Subscribes to updates with robot state information (tool position
	/// and velocity).
	/// </summary>
	public void SubscribeToUpdate ()
	{
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);
		robot.SubscribeToRobotStatus (status => {
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Handedness: {0}", status.handedness);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "Outerlink: {0}", status.outerlink);
			Barrett.Logger.Debug(Barrett.Logger.INFO, "IsPatientConnected?: {0}", status.patient);
		});

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
