package org.pointclouds.PCLAndroidSample;


import android.app.Activity;
import android.widget.TextView;
import android.os.Bundle;

 
public class HelloPCL extends Activity {
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		
		TextView  tv = new TextView(this);
		int       x  = 1000;
		int       y  = 42;
		
		// here, we dynamically load the library at runtime
		// before calling the native method.
		//
		System.loadLibrary("helloPCLWorld");
	
		int z = 26;
		tv.setText( "The sum of " + x + " and " + y + " is " + z + " and boost: " + new HelloPCL().boostMkDir());
		
		System.out.println();

		setContentView(tv);
	}	
	
	public native String boostMkDir ();
	public native String smoothPointCloud ();
 }
