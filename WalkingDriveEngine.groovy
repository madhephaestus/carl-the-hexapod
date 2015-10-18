
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


double stepOverHeight=5;
long stepOverTime=100;
Double zLock=-65;
Closure calcHome = { DHParameterKinematics leg -> leg.calcHome() }

ArrayList<Object> args = new ArrayList<Object>();

args.add(stepOverHeight)
args.add(stepOverTime)
args.add(zLock)
args.add(calcHome)


return ScriptingEngineWidget.inlineGistScriptRun("bcb4760a449190206170", "GenericWalkingEngine.groovy" , args);
	