using System;
using System.Text;
using System.Collections;
using System.Threading;
using MsgPack;
using Barrett.CoAP;
using Barrett.CoAP.MsgTypes;
using UnityEngine;

/// <summary>
/// Streams basic state information to the console.
/// </summary>
public class DisplayBasicInfo
{
	public RobotClient robot;
	public Barrett.KeyboardManager keyboardManager;

	Vector3 tool_position = Vector3.zero;
	Vector3 tool_velocity = Vector3.zero;
	Vector3 joint_position = Vector3.zero;
	RobotHandednessEnum handedness = RobotHandednessEnum.Error;
	OuterlinkStatusEnum outerlinkStatus = OuterlinkStatusEnum.NotConnected;

	/// <summary>
	/// Initializes a new instance of the <see cref="DisplayBasicInfo"/> class.
	/// </summary>
	public DisplayBasicInfo ()
	{
		// Set up communication with the robot and initialize force to zero
		robot = new RobotClient ();
		SubscribeToUpdates ();
		robot.SendCartesianForces (Vector3.zero);

		// Set up keyboard callbacks
		keyboardManager = new Barrett.KeyboardManager ();
		keyboardManager.SetDebug (true); // print key pressed
		keyboardManager.AddKeyPressCallback ("q", Close);
		keyboardManager.AddKeyPressCallback ("h", OnHome);
		keyboardManager.AddKeyPressCallback ("e", OnEnable);
		keyboardManager.AddKeyPressCallback ("d", OnDisable);
		PrintUsage ();

		// Add some blank lines for readability, then get the cursor position.
		Console.WriteLine ();
		Console.WriteLine ();
		int top = Console.CursorTop;

		// Print labels, which do not change. Add an offset on the left side for readability.
		int line = top;
		int left = 10;
		Console.SetCursorPosition (left, line);
		Console.WriteLine ("Joint positions:");
		line++;
		Console.SetCursorPosition (left, line);
		Console.WriteLine ("Tool position:");
		line++;
		Console.SetCursorPosition (left, line);
		Console.WriteLine ("Tool Velocity:");
		line++;
		Console.SetCursorPosition (left, line);
		Console.WriteLine ("Handedness:");
		line++;
		Console.SetCursorPosition (left, line);
		Console.WriteLine ("Outer link status:");

		// Move the cursor to a place that is nice to display user input
		line += 3;
		Console.SetCursorPosition (0, line);
		Console.Write(new string(' ', Console.WindowWidth));
		line++;
		Console.SetCursorPosition (0, line + 1);
		Console.Write(new string(' ', Console.WindowWidth));
		Console.SetCursorPosition (0, line);

		// Set the start position on each line for the data. Make sure there is enough space
		// for your labels.
		left = 30;

		// Loop: send zero force and display most recent state info
		bool running = true;
		while (running) {
			line = top;
			Console.SetCursorPosition(left, line);
			Console.Write (joint_position.ToString ("F4") + "       ");
			line++;
			Console.SetCursorPosition(left, line);
			Console.Write (tool_position.ToString ("F4") + "       ");
			line++;
			Console.SetCursorPosition(left, line);
			Console.Write (tool_velocity.ToString ("F4") + "       ");
			line++;
			Console.SetCursorPosition(left, line);
			Console.Write (handedness + "       ");
			line++;
			Console.SetCursorPosition(left, line);
			Console.Write (outerlinkStatus + "       ");

			line += 3;
			Console.SetCursorPosition (0, line);
			Console.Write(new string(' ', Console.WindowWidth));
			Console.SetCursorPosition (0, line + 1);
			Console.Write(new string(' ', Console.WindowWidth));
			Console.SetCursorPosition (0, line);
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
		Barrett.Logger.Debug (Barrett.Logger.INFO, "\tq: Quit");
	}

	/// <summary>
	/// Handles state information received from the robot. Prints tool position
	/// and velocity.
	/// </summary>
	public void OnReceiveServerUpdate (ServerUpdate update)
	{
		tool_position = update.position;
		tool_velocity = update.velocity;
		joint_position = update.joint_position;
	}

	/// <summary>
	/// Prints status information received from the robot.
	/// </summary>
	private void OnReceiveRobotStatus (Barrett.CoAP.MsgTypes.RobotStatus status)
	{
		handedness = status.handedness;
		outerlinkStatus = status.outerlink;
	}

	/// <summary>
	/// Subscribes to updates with robot state information (tool position
	/// and velocity).
	/// </summary>
	public void SubscribeToUpdates ()
	{
		robot.SubscribeToServerUpdate (OnReceiveServerUpdate);
		robot.SubscribeToRobotStatus (OnReceiveRobotStatus);
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
