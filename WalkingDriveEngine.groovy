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

IDriveEngine engine =  new IDriveEngine (){
	double stepOverHeight=25;
	boolean takingStep = false;
	private Double zLock=-75;
	TransformNR previousGLobalState;
	TransformNR target;

	private void interopolate(double iteration,double end,double time,MobileBase source){
		FxTimer.runLater(
				Duration.ofMillis((int)(time*1000.0)/end) ,new Runnable() {
					@Override
					public void run() {

						previousGLobalState.translateX(target.getX()/end);
						previousGLobalState.translateY(target.getY()/end);
						previousGLobalState.translateZ(target.getZ()/end);
						double rotz = -target.getRotation().getRotationZ()/end +previousGLobalState.getRotation().getRotationZ() ;
						double rotx = target.getRotation().getRotationX() ;
						double roty = target.getRotation().getRotationY();
						RotationNR neRot = new RotationNR(	Math.toDegrees(rotx),
															Math.toDegrees(roty),
															Math.toDegrees(rotz));//RotationNR.getRotationZ(Math.toDegrees(rotz));
		
						previousGLobalState.setRotation(neRot );
						// New target calculated appliaed to global offset
						source.setGlobalToFiducialTransform(previousGLobalState);
						if(iteration<end)
							interopolate(iteration+1, end, time,source);
					}
				});
	}
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		
		if(takingStep)
			return;