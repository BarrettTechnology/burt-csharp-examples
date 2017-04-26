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
	/// <summary>
	/// Initializes a new instance of the <see cref="DisplayBasicInfo"/> class.
	/// </summary>
	public DisplayBasicInfo ()
	{
		bool running = true;
		int count = 0;
		while (running) {
			Console.SetCursorPosition(0, 0);
			Console.WriteLine("Seconds since start: " + count++.ToString ());
			Thread.Sleep (1000);
		}
	}
}
