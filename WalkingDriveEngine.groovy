
import java.time.Duration;

import java.util.ArrayList;

import javafx.application.Platform;

import org.reactfx.util.FxTimer;

import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.util.ThreadUtil;
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine;


double stepOverHeight=2;
long stepOverTime=200;
Double zLock=-70;
Closure calcHome = { DHParameterKinematics leg -> 
		TransformNR h=leg.calcHome() 

		TransformNR tr = leg.forwardOffset(new TransformNR())
		tr.setZ(zLock)
		
		return h;

}

ArrayList<Object> args = new ArrayList<Object>();

args.add(stepOverHeight)
args.add(stepOverTime)
args.add(zLock)
args.add(calcHome)


return ScriptingEngine.inlineGistScriptRun("bcb4760a449190206170", "GenericWalkingEngine.groovy" , args);
	