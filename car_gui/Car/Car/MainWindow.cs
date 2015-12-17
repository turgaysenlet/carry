using System;
using System.Collections;
using System.Collections.Generic;
using Gtk;
using System.Diagnostics;
using Pango;

public partial class MainWindow: Gtk.Window
{	
	Process roscore = new Process();
	Process usb = new Process();
	Process image = new Process();
	Process uvc = new Process();
	Process record = new Process();
	Process gps = new Process();
	List<Process> processes = new List<Process> ();
	List<ToggleButton> buttons = new List<ToggleButton> ();
	FontDescription font = new FontDescription(); 
	List<int> processIds = new List<int>();
	public MainWindow (): base (Gtk.WindowType.Toplevel)
	{
		Build ();
		GLib.Timeout.Add (1000, CheckWorkingTimeoutHandler);	
		processes.AddRange (new Process[] { roscore, usb, image, record, gps });
		buttons.AddRange (new ToggleButton[] { buttonRoscore, buttonUsb, buttonImage, buttonRecord, buttonGps });
		SetButtonColors ();
	}
	protected bool CheckWorkingTimeoutHandler() {
		RefreshButtons ();
		return true;
	}

	void SetButtonColors ()
	{
		foreach (Button button in buttons) {
			SetButtonColors (button);
		}
		SetButtonColors (buttonUvc);
		SetButtonColors (buttonCloseAll);
		SetButtonColors (buttonExit);
	}

	void SetButtonColors (Button button) {
		button.ModifyBg (StateType.Normal, new Gdk.Color (220, 220, 220));
		button.ModifyBg (StateType.Prelight, new Gdk.Color (160, 160, 180));
		button.ModifyBg (StateType.Active, new Gdk.Color (160, 160, 160));
	}

	void SetSelectedButtonColors (Button button) {
		button.ModifyBg (StateType.Active, new Gdk.Color (220, 220, 220));
		button.ModifyBg (StateType.Prelight, new Gdk.Color (160, 160, 180));
		button.ModifyBg (StateType.Normal, new Gdk.Color (160, 160, 160));
	}

	void CloseAllProcesses ()
	{
		Process kill = new Process ();
		kill.EnableRaisingEvents = true; 
		kill.StartInfo.FileName = "/usr/bin/bash_kill";
		kill.Start();
		try {
			kill.WaitForExit (2000);
		} catch {
		}

		foreach (ToggleButton button in buttons) {
			button.Active = false;
		}

		foreach (Process proc in processes) {
			if (ProcessHasStarted (proc) && !proc.HasExited) {
				try {
					proc.Kill ();
				} catch {
				}
			}
		}
		Process[] p = Process.GetProcesses ();
		foreach (Process proc in p) {
			try {
				string name = proc.ProcessName;
				if (name.Contains ("bash_roscore") ||
					name.Contains ("bash_usb") ||
					name.Contains ("bash_uvc") ||
					name.Contains ("bash_left") ||
					name.Contains ("bash_record") ||
					name.Contains ("bash_gps")) {
					proc.Kill ();
				}
			} catch {
			}
			try {
				if (processIds.Contains(proc.Id)) {
					proc.Kill ();
				}
			} catch {
			}
		}
	}

	void RefreshButtons ()
	{
		int i = 0;
		foreach (Process proc in processes) {
			if (ProcessHasStarted (proc) && !proc.HasExited && buttons [i] != null) {
				SetSelectedButtonColors (buttons [i]);
				//buttons [i].Active = true;
			} else {
				SetButtonColors (buttons [i]);
				//buttons [i].Active = false;
			}
			i++;
		}
	}

	void AddProcessId (Process proc)
	{
		try {
			processIds.Add(proc.Id);
		} catch {
		}
	}

	protected bool ProcessHasStarted(Process proc) {
		if (proc == null) {
			return false;
		}
		try {
			bool h = proc.HasExited;
		} catch (InvalidOperationException ex) {
			if (ex.Message.Contains ("Process has not been started")) {
				return false;
			}
		}
		return true;
	}

	protected void OnDeleteEvent (object sender, DeleteEventArgs a)
	{
		CloseAllProcesses ();
		Application.Quit ();
		a.RetVal = true;
	}

	protected void buttonExitClicked (object sender, EventArgs e)
	{
		CloseAllProcesses ();
		Application.Quit ();
	}
	

	protected void buttonCloseAllClicked (object sender, EventArgs e)
	{
		CloseAllProcesses ();
		RefreshButtons ();
	}

	protected void buttonRoscoreClicked (object sender, EventArgs e)
	{
		if (buttonRoscore.Active) {
			roscore.EnableRaisingEvents = true; 
			roscore.StartInfo.FileName = "/usr/bin/bash_roscore";
			roscore.Start();
			AddProcessId (roscore);
			//roscore.StartInfo.RedirectStandardOutput = true;
			//roscore.StartInfo.Arguments = "-l | grep NTFS";
			//roscore.WaitForExit();
			//string data = roscore.StandardOutput.ReadToEnd();
			//Console.WriteLine( data + " was returned" );
		}
	}

	protected void buttonUsbClicked (object sender, EventArgs e)
	{
		if (buttonUsb.Active) {
			usb.EnableRaisingEvents = true; 
			usb.StartInfo.FileName = "/usr/bin/bash_usb";
			usb.Start ();
			AddProcessId (usb);
		}
	}

	protected void buttonImageClicked (object sender, EventArgs e)
	{
		if (buttonImage.Active) {
			image.EnableRaisingEvents = true; 
			image.StartInfo.FileName = "/usr/bin/bash_left";
			image.Start ();
			AddProcessId (image);
		}
	}

	protected void buttonUvcClicked (object sender, EventArgs e)
	{
		uvc.EnableRaisingEvents = true; 
		uvc.StartInfo.FileName = "/usr/bin/bash_uvc";
		uvc.Start ();
		AddProcessId (uvc);
	}

	protected void buttonRecordClicked (object sender, EventArgs e)
	{
		if (buttonRecord.Active) {
			record.EnableRaisingEvents = true; 
			record.StartInfo.FileName = "/usr/bin/bash_record";
			record.Start ();
			AddProcessId (record);
		}
	}

	protected void buttonGpsClicked (object sender, EventArgs e)
	{
		if (buttonGps.Active) {
			gps.EnableRaisingEvents = true; 
			gps.StartInfo.FileName = "/usr/bin/bash_gps";
			gps.Start ();
			AddProcessId (gps);
		}
	}
}
