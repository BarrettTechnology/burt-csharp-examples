using System;
using System.Text;
using System.Threading;
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

	// Initialize all the state information to default values. These will be printed
	// to the console until the first state update is received.
	// The default values are all zeroes for Vector3 types and error states for the
	// Enum types.
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
		// Set up communication with the robot and initialize force to zero.
		robot = new RobotClient ();
		SubscribeToUpdates ();
		robot.SendCartesianForces (Vector3.zero);

		// Set up keyboard callbacks.
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

		// Print labels, which do not change. Add an offset on the left side for readability
		// and specify the length of the strings to clear any text that already exists in
		// that space. Make sure that the length is longer than your strings to leave enough
		// space.
		int line = top;
		int left = 10;
		int length = 20;
		PrintAtPosition (left, line++, "Joint positions:", length);
		PrintAtPosition (left, line++, "Tool position:", length);
		PrintAtPosition (left, line++, "Tool Velocity:", length);
		PrintAtPosition (left, line++, "Handedness:", length);
		PrintAtPosition (left, line++, "Outer link status:", length);

		// Set the start position on each line for the data.
		left += length;
		length = 30;

		// Loop: Display most recent state info, check for new key presses, and send zero force.
		bool running = true;
		while (running) {
			line = top;
			PrintAtPosition (left, line++, joint_position.ToString ("F4"), length);
			PrintAtPosition (left, line++, tool_position.ToString ("F4"), length);
			PrintAtPosition (left, line++, tool_velocity.ToString ("F4"), length);
			PrintAtPosition (left, line++, handedness.ToString (), length);
			PrintAtPosition (left, line++, outerlinkStatus.ToString (), length);

			// Move the cursor to a nice place to display user input.
			line += 2;
			Console.SetCursorPosition (0, line);
			running = ReadKeyPress ();

			Thread.Sleep (50);
			robot.SendCartesianForces (Vector3.zero);
		}

		Console.Write ("Quitting.");
		Environment.Exit (0);
	}

	/// <summary>
	/// Prints the specified text at the desired cursor position. If a length is specified as
	/// the fourth argument, the string will be either truncated to that length or padded
	/// with whitespace to that length. Padding with whitespace has the effect of clearing
	/// text previously written to that position.
	/// </summary>
	/// <param name="left">Position of the string from the left side of the window.</param>
	/// <param name="top">Position of the string from the top of the window.</param>
	/// <param name="str">String to be printed.</param>
	/// <param name="length">(optional) Length of the string.</param>
	public void PrintAtPosition (int left, int top, string str, int length = -1)
	{
		// Pad the string with zeroes if needed, or shorten the string if needed.
		if (length > str.Length) {
			str += new string (' ', length - str.Length);
		} else if ((length >= 0) && (length < str.Length)) {
			str = str.Substring (0, length);
		}

		Console.SetCursorPosition(left, top);
		Console.Write (str);
	}

	/// <summary>
	/// Clears the specified line(s) of text.
	/// </summary>
	/// <param name="line">The position of the line from the top of the window.</param>
	/// <param name="num">(optional) The number of lines to clear, default 1.</param>
	public void ClearLine (int line, uint num = 1)
	{
		for (uint i = 0; i < num; i++) {
			PrintAtPosition (0, line++, "", Console.WindowWidth);
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
	/// Handles state information received from the robot.
	/// </summary>
	public void OnReceiveServerUpdate (ServerUpdate update)
	{
		tool_position = update.position;
		tool_velocity = update.velocity;
		joint_position = update.joint_position;
	}

	/// <summary>
	/// Handles status information received from the robot.
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
	/// Reads the key press.
	/// </summary>
	public bool ReadKeyPress ()
	{
		if (Console.KeyAvailable) {
			// Clear the previous command. Since the amount of text printed by the previous
			// command is unknown, clear the everything below the current cursor position.
			// Note that the cursor is assumed to be already positioned within the designated
			// area for user input.
			int line = Console.CursorTop;
			ClearLine (line, (uint)(Console.WindowHeight - line));
			Console.SetCursorPosition (0, line);

			// Handle the new command
			string keyPressed = Console.ReadKey (false).KeyChar.ToString ();
			keyboardManager.HandleKeyPress (keyPressed);
			if (keyPressed.Equals ("q") || keyPressed.Equals ("Q")) {
				return false;
			}
		}
		return true;
	}
}
