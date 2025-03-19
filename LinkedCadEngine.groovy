import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.creature.CreatureLab
import org.apache.commons.io.IOUtils
import com.neuronrobotics.bowlerstudio.vitamins.*
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase

import eu.mihosoft.vrl.v3d.CSG
//First we load teh default cad generator script 
ICadGenerator defaultCadGen=(ICadGenerator) ScriptingEngine
	                    .gitScriptRun(
                                "https://github.com/madhephaestus/carl-the-hexapod.git", // git location of the library
	                              "ThreeDPrintCad.groovy" , // file to load
	                              null
                        )
return new ICadGenerator(){
	@Override 
	public ArrayList generateCad(DHParameterKinematics d, int linkIndex) {
		ArrayList allCad=defaultCadGen.generateCad(d,linkIndex)
		//If you want you can add things here
		//allCad.add(myCSG);
		return allCad
	}
	@Override 
	public ArrayList generateBody(MobileBase b ) {
		ArrayList allCad=defaultCadGen.generateBody(b)
		//If you want you can add things here
		//allCad.add(myCSG);
		return allCad
	}
}