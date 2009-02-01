package com.macpod.srv1console;

/*
 *  SRV1Client.java - Android interface to the SRV-1 Blackfin robot.
 *    Copyright (C) 2008-2009  Surveyor Corporation and Macpod Software
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
 
import java.io.DataInputStream;
import java.io.DataOutputStream;
//import java.io.InputStream;
import java.net.InetAddress;
import java.net.Socket;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class SRV1Client {
	private RoboClientThreader threader = null;

	public boolean connect(String host, Handler new_handler) {
		if (threader != null && threader.connected()) {
			return false;
		}
		try {
			threader = new RoboClientThreader();
			return threader.connect(host, new_handler);
		} catch (Exception e) {
			return false;
		}
	}

	public void disconnect() {
		try {
			if (threader == null)
				return;
			threader.disconnect();
			threader = null;
		} catch (Exception e) {
		}
	}

	public boolean connected() {
		if (threader == null)
			return false;
		return threader.connected();
	}

	public Bitmap getBitmap() {
		return threader.getBitmap();
	}

	private class RoboClientThreader extends Thread {
		public static final int SRV1_PORT = 10001;
		private Bitmap bm = null;
		private Socket connection = null;
		private Handler interface_handler;

		public Bitmap getBitmap() {
			return bm;
		}

		private long ignoreGarbage(DataInputStream in) throws Exception {
			int avail_count, skip_count;
			long total_skip_count = 0;

			do {
				avail_count = in.available();
				total_skip_count += in.skip(avail_count);
				//sleep(250);
			} while (avail_count > 0);

			return total_skip_count;
		}


		public void run() {

			// Update the client's interface to show we are running.
			try {
				Message m = new Message();
				m.what = SRV1Console.CONNECTED_INTERFACE;
				interface_handler.sendMessage(m);
			} catch (Exception e) {
			}

			try {
				DataOutputStream out = new DataOutputStream(connection
						.getOutputStream());
				DataInputStream in = new DataInputStream(connection
						.getInputStream());
				Log.d("SRV1", "Connection is up!");

				// Loop until we know we set it correctly.
				for (;;) {
					Log.d("SRV1", "done reading garbage" + ignoreGarbage(in));

					// Write to set resolution to 320x240
					out.writeByte('b');

					// Verify resolution was set.
					char verif1 = (char) in.readByte();
					Log.d("SRV1", "Recieved:" + verif1);
					char verif2 = (char) in.readByte();
					Log.d("SRV1", "Recieved:" + verif2);

					if (verif1 == '#' && verif2 == 'b') {
						break;
					}
					Log.d("SRV1", "Couldn't verify resolution was set.");
				}
				Log.d("SRV1", "Successfully set image resolution!");

				while (true) { // Read and display images as fast as possible.
					Log
							.d("SRV1",
									"===========================================");
					Log
							.d("SRV1", "done reading garbage 1:"
									+ ignoreGarbage(in));
					long garbage = ignoreGarbage(in);
					if (garbage > 0) {
						Log.w("SRV1", "read more garbage:" + garbage);
					}

					while (in.available() == 0) {
						out.writeByte('I');
						//sleep(250);
					}

					// Read in header
					if ((char) in.readByte() != '#'
							|| (char) in.readByte() != '#'
							|| (char) in.readByte() != 'I'
							|| (char) in.readByte() != 'M'
							|| (char) in.readByte() != 'J'
							|| (char) in.readByte() != '5') {
						Log.d("SRV1", "Bad header");
						continue;
					}

					int s0 = in.readUnsignedByte();
					int s1 = in.readUnsignedByte();
					int s2 = in.readUnsignedByte();
					int s3 = in.readUnsignedByte();

					int val = 0 | s0 | s1 << 8 | s2 << 16 | s3 << 24;
					Log.d("SRV1", "Image size integer: " +  val);
					// Get data
					//=============================================================
					
					
					int bytesRead=0;
					int bytesToRead=val;
					byte[] data = new byte[bytesToRead];
					while (bytesRead < bytesToRead) {
					  int result = in.read(data, bytesRead, bytesToRead - bytesRead);
					  if (result == -1) break;
					  bytesRead += result;
					}
					
			
					bm = BitmapFactory.decodeByteArray(data, 0, val);
					

					//==============================================================
					Log.d("SRV1", "Displaying the new image");

					// Display the image
					try {
						Message m = new Message();
						m.what = SRV1Console.UPDATE_IMAGE;
						interface_handler.sendMessage(m);
					} catch (Exception e) {
					}

					Log.d("SRV1", "Done displaying the new image");

				}
			} catch (Exception e) {
				try {
					connection.shutdownInput();
				} catch (Exception ee) {
				}
				try {
					connection.shutdownOutput();
				} catch (Exception ee) {
				}
				try {
					connection.close();
				} catch (Exception ee) {
				}
			} finally {
				connection = null;
				try {
					Message m = new Message();
					m.what = SRV1Console.DISCONNECTED_INTERFACE;
					interface_handler.sendMessage(m);
				} catch (Exception e) {
				}
			}
		}

		public boolean connected() {
			return (connection != null);
		}

		public boolean connect(String host, Handler new_handler) {
			boolean err = false;
			if (connected()) {
				return false;
			}

			try {
				InetAddress addr = InetAddress.getByName(host);
				connection = new Socket(addr, SRV1_PORT);
				interface_handler = new_handler;
				start();
			} catch (Exception e) {
				connection = null;
				err = true;
			}

			return !err;
		}

		public void disconnect() {
			if (connection == null)
				return;

			try {
				connection.shutdownInput();
			} catch (Exception e) {
			}
			try {
				connection.shutdownOutput();
			} catch (Exception e) {
			}
			try {
				connection.close();
			} catch (Exception e) {
			}
			try {
				join();
			} catch (Exception e) {
			}
		}
	}
}
