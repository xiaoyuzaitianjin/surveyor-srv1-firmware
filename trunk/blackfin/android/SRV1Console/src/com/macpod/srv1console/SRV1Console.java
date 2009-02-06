package com.macpod.srv1console;

/*
 *  SRV1Console.java - Android interface to the SRV-1 Blackfin robot.
 *    Copyright (C) 2008-2009  Surveyor Corporation and Jeffrey Nelson
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 */

/* g1 has a screen size of 480 x 320
 * 
 * can transmit 1 = 80x64, 3 = 160x120, 5 = 320x240,
 */
 
import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;

public class SRV1Console extends Activity {
	private SRV1Client engine;
	protected SRV1Console ref = this;

	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);

		engine = new SRV1Client();

		Button stateButton = (Button) findViewById(R.id.state_button);
		stateButton.setOnClickListener(stateListener);

		disconnectedInterface();
	}

	protected void onStop() {
		engine.disconnect();
		super.onStop();
	}

	private OnClickListener stateListener = new OnClickListener() {
		public void onClick(View v) {
			try {
				if (engine.connected()) {
					// Disconnect
					engine.disconnect();
				} else {
					// Connect
					String server = ((EditText) findViewById(R.id.server_field))
							.getText().toString();

					if (!engine.connect(server, interface_handler)) {
						// Report a vague error for users to ponder over if we
						// can't connect.
						Dialogue("Could not connect to given server.");
					}
				}
			} catch (Exception e) {
				Dialogue("An error occured.");

			}
		}

	};

	public static final int CONNECTED_INTERFACE = 0;
	public static final int DISCONNECTED_INTERFACE = 1;
	public static final int UPDATE_IMAGE = 2;

	private Handler interface_handler = new Handler() {
		public void handleMessage(Message msg) {
			switch (msg.what) {
			case CONNECTED_INTERFACE:
				connectedInterface();
				break;
			case DISCONNECTED_INTERFACE:
				disconnectedInterface();
				break;
			case UPDATE_IMAGE:
				updateImage();
				break;
			}
		}
	};

	public void updateImage() {
		try {
			ImageView iv = (ImageView) findViewById(R.id.picview);
			iv.setImageBitmap(engine.getBitmap());
		} catch (Exception e) {
			// Ignore. Could be thrown because of a null bitmap or because the socket is dead. 
		}
	}

	public void connectedInterface() {
		try {
			TextView tv = (TextView) findViewById(R.id.status_label);
			tv.setText(R.string.status_running);
			Button stateButton = (Button) findViewById(R.id.state_button);
			stateButton.setText(R.string.state_stop);
		} catch (Exception e) {
		}

	}

	public void disconnectedInterface() {
		try {
			TextView tv = (TextView) findViewById(R.id.status_label);
			tv.setText(R.string.status_stopped);
			Button stateButton = (Button) findViewById(R.id.state_button);
			stateButton.setText(R.string.state_start);
		} catch (Exception e) {
		}
	}

	private void Dialogue(CharSequence message) {
		new AlertDialog.Builder(this).setMessage(message).setNeutralButton(
				"OK", new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int whichButton) {
					}
				}).show();
	}

}