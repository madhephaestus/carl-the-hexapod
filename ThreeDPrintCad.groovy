import eu.mihosoft.vrl.v3d.Extrude;

import eu.mihosoft.vrl.v3d.parametrics.*;
import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import javafx.scene.paint.Color;
import javax.vecmath.Matrix4d;
import Jama.Matrix;
import com.neuronrobotics.bowlerstudio.BowlerStudioController;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.imageprovider.NativeResource;
import com.neuronrobotics.nrconsole.plugin.BowlerCam.RGBSlider.ColorBox;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins;

import eu.mihosoft.vrl.v3d.CSG;
import eu.mihosoft.vrl.v3d.Cube;
import eu.mihosoft.vrl.v3d.FileUtil;
import eu.mihosoft.vrl.v3d.PrepForManufacturing
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.STL;
import eu.mihosoft.vrl.v3d.Sphere;
import eu.mihosoft.vrl.v3d.Transform;
import eu.mihosoft.vrl.v3d.Cylinder;
import eu.mihosoft.vrl.v3d.Vector3d;
import javafx.scene.paint.Color;
import com.neuronrobotics.bowlerstudio.physics.*;

return new ICadGenerator(){
	//CSG servoReference= new MicroServo().toCSG();
	HashMap<String , HashMap<String,ArrayList<CSG>>> map =  new HashMap<>();
	HashMap<String,ArrayList<CSG>> bodyMap =  new HashMap<>();
	CSG dyioReference=   (CSG)(ScriptingEngine.inlineGistScriptRun(
		"fb4cf429372deeb36f52", 
		"dyioCad.groovy" ,
		null))
	
//	.transformed(new Transform().translateZ(12.0))
//	.transformed(new Transform().translateX(5.4));
	
	//CSG horn=  STL.file(NativeResource.inJarLoad(IVitamin.class,"smallmotorhorn.stl").toPath())

	CSG mountScrewKeepaway = (CSG)(ScriptingEngine.inlineGistScriptRun(
		"488e0ee249a5c16ae4d8",
		"moutScrewKeepaway.groovy" ,
		null))
	private double attachmentRodWidth=10;
	private double attachmentBaseWidth=15;
	//private double printerTollerence =0.5;
	private double mountScrewKeepawaySize= 7.5;
	private double mountScrewHoleKeepawaySize= 4.1;
	private double mountScrewSeperationDistance=attachmentRodWidth/2+mountScrewHoleKeepawaySize/2+0.5;
	double cylandarRadius = 13.5;
	private double bearingPinRadius=3;
	LengthParameter printerOffset = new LengthParameter("printerOffset",0.5,[2,0.001])
	
	private CSG toZMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateZ(-target.getBounds().getMin().z));
	}
	private CSG toZMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateZ(-target.getBounds().getMax().z));
	}
	private CSG toXMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateX(-target.getBounds().getMin().x));
	}
	private CSG toXMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateX(-target.getBounds().getMax().x));
	}
	private CSG toYMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateY(-target.getBounds().getMin().y));
	}
	private CSG toYMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateY(-target.getBounds().getMax().y));
	}
	private CSG toZMin(CSG incoming){
		return toZMin(incoming,incoming);
	}
	private CSG toZMax(CSG incoming){
		return toZMax(incoming,incoming);
	}
	private CSG toXMin(CSG incoming){
		return toXMin(incoming,incoming);
	}
	private CSG toXMax(CSG incoming){
		return toXMax(incoming,incoming);
	}
	private CSG toYMin(CSG incoming){
		return toYMin(incoming,incoming);
	}
	private CSG toYMax(CSG incoming){
		return toYMax(incoming,incoming);
	}
	private CSG getAppendageMount(LinkConfiguration conf){
		double cylindarKeepawayHeight = 80;
		CSG attachmentbase =new Cylinder(// The first part is the hole to put the screw in
					computeKeepayayRadius(conf)*2,
					cylindarKeepawayHeight,
					 (int)20).toCSG()
					 .toXMin()
			.transformed(new Transform().translateX(-computeKeepayayRadius(conf)))
			.transformed(new Transform().translateZ(-cylindarKeepawayHeight/2))
		
		return attachmentbase;
	}
	
	
	private CSG getMountScrewKeepaway(){
		return mountScrewKeepaway.clone();
	}
	
	private CSG getAttachment(LinkConfiguration conf){
		CSG h  =new Cube(1,1,1).toCSG()
		if(conf !=null){
			h  =Vitamins.get(conf.getShaftType(),conf.getShaftSize())		
		}
		return (CSG)(ScriptingEngine.gitScriptRun(
		"https://github.com/madhephaestus/carl-the-hexapod.git",
		"servoAttachment.groovy" ,
		[h,conf]))
	}
	
	private CSG getFoot(LinkConfiguration conf){
		CSG attach = getAttachment(conf).toXMax();
		CSG foot = new Sphere(attachmentRodWidth).toCSG();
		return attach.union(foot);
	}
	/*
	private CSG reverseDHValues(CSG incoming,DHLink dh ){
		println "Reversing "+dh
		TransformNR step = new TransformNR(dh.DhStep(0))
		Transform move = TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}
	
	private CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = TransformFactory.nrToCSG(step)
		println dh
		return incoming.transformed(move)
		
	}
	*/
	private CSG reverseDHValues(CSG incoming,DHLink dh ){
		println "Reversing "+dh
		return incoming
			.rotx(Math.toDegrees(-dh.getAlpha()))
			.movex(dh.getR()/2)		
	}
	
	private CSG moveDHValues(CSG incoming,DHLink dh ){
		return incoming.transformed(new Transform().translateZ(-dh.getD()))
		.transformed(new Transform().rotZ(-Math.toDegrees(dh.getTheta())))
		.transformed(new Transform().rotZ((90+Math.toDegrees(dh.getTheta()))))
		.transformed(new Transform().translateX(-dh.getR()))
		.transformed(new Transform().rotX(Math.toDegrees(dh.getAlpha())));
		
	}

		/**
	 * Gets the all dh chains.
	 *
	 * @return the all dh chains
	 */
	public ArrayList<DHParameterKinematics> getLimbDHChains(MobileBase base) {
		ArrayList<DHParameterKinematics> copy = new ArrayList<DHParameterKinematics>();
		for(DHParameterKinematics l:base.getLegs()){
			copy.add(l);	
		}
		for(DHParameterKinematics l:base.getAppendages() ){
			copy.add(l);	
		}
		return copy;
	}

	ArrayList<CSG> generateBody(MobileBase base){

		ArrayList<CSG> cutouts=new ArrayList<>();
		ArrayList<CSG> attach=new ArrayList<>();
		String legStr =""
		for(DHParameterKinematics l:getLimbDHChains(base)){
			legStr+=l.getRobotToFiducialTransform(). getXml();
		}
		if(bodyMap.get(legStr)!=null){
			println "Body cached"
			for(CSG csg:bodyMap.get(legStr))
				csg.setManipulator(base.getRootListener());
			return bodyMap.get(legStr)
		}
		println "Generating body"
		CSG attachUnion=null;
		for(DHParameterKinematics l:getLimbDHChains(base)){
			TransformNR position = l.getRobotToFiducialTransform();
			Transform csgTrans = TransformFactory.nrToCSG(position)
			cutouts.add(getAppendageMount(l.getLinkConfiguration(0))
				.transformed(csgTrans)
				);
			CSG attachment = getAttachment(l.getLinkConfiguration(0))// get attachment for root
				.transformed(csgTrans)
			attach.add(attachment);
			if(attachUnion==null){
				attachUnion=attachment;
			}else{
				attachUnion = 	attachUnion.union(attachment)
			}
			
		}
		if(attachUnion==null)
			return []
		CSG upperBody = attachUnion.hull()
		
		
		CSG dyioBottomPlate =dyioReference
						.rotx(180)
						.movez(dyioReference.getMaxZ())
						.movez(upperBody.getMinZ())	
	
		CSG myDyIO=dyioReference
				.movez(upperBody.getMaxZ()+22.0)				
				
		upperBody=upperBody
		.union(myDyIO.union(dyioBottomPlate)
					.movex(	upperBody.getMaxX()-
						Math.abs(upperBody.getMinX()))
					.movey(	upperBody.getMaxY()-
						Math.abs(upperBody.getMinY()))
						
		).hull()
		.difference(myDyIO.movex(	upperBody.getMaxX()-
						Math.abs(upperBody.getMinX()))
					.movey(	upperBody.getMaxY()-
						Math.abs(upperBody.getMinY())))
		CSG batteryBox = new Cube(71,66,23)
					.toCSG()
					.movex(	upperBody.getMaxX()-
						Math.abs(upperBody.getMinX()))
					.movey(	upperBody.getMaxY()-
						Math.abs(upperBody.getMinY()))
					.toZMax()
					.movez(upperBody.getMaxZ())	
		cutouts.add(batteryBox)				
		upperBody= upperBody.difference(cutouts);
		
		upperBody= upperBody.union(attach);
		
	
					
		upperBody.setColor(Color.GREEN);
		upperBody.setManipulator(base.getRootListener());
		upperBody.setManufactuing(new PrepForManufacturing() {
					public CSG prep(CSG arg0) {
						return arg0.toZMin();
					}
				});
		
		def bodyParts = [upperBody]as ArrayList<CSG>
		bodyMap.put(legStr,bodyParts)
		return bodyParts;
	}

	private double computeKeepayayRadius(LinkConfiguration conf){
		HashMap<String, Object> shaftmap = Vitamins.getConfiguration(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
		double totalFlangLen = (shaftmap.flangeLongDimention-shaftmap.servoThickDimentionThickness)/2
		double shaftToShortSideFlandgeEdge = shaftmap.shaftToShortSideDistance+totalFlangLen
		double x = shaftToShortSideFlandgeEdge
		double y = shaftmap.servoThinDimentionThickness
		return Math.sqrt((x*x)+(y*y))
	}
	
	
	public ArrayList<CSG> generateCad(DHParameterKinematics sourceLimb, int linkIndex){
		String legStr = sourceLimb.getXml()
		LinkConfiguration conf = sourceLimb.getLinkConfiguration(linkIndex);
		LinkConfiguration lastConf = null;
		int numLinks = sourceLimb.getChain().getLinks().size() 
		if(linkIndex< numLinks-1){
			lastConf =sourceLimb.getLinkConfiguration(linkIndex+1);
		}
		String linkStr =conf.getXml()
		ArrayList<CSG> csg = null;
		HashMap<String,ArrayList<CSG>> legmap=null;
		if(map.get(legStr)==null){
			map.put(legStr, new HashMap<String,ArrayList<CSG>>())	
			// now load the cad and return it. 
		}
		legmap=map.get(legStr)
		if(legmap.get(linkStr) == null ){
			legmap.put(linkStr,new ArrayList<CSG>())
		}
		csg = legmap.get(linkStr)
		if(csg.size()>linkIndex){
			// this link is cached
			println "This link is cached"
			return csg;
		}
		
		//printBed=true;
		ArrayList<DHLink> dhLinks=sourceLimb.getChain().getLinks();
		
		
		
		HashMap<String, Object> shaftmap = Vitamins.getConfiguration(conf.getShaftType(),conf.getShaftSize())
		double hornOffset = 	shaftmap.get("hornThickness")	
		
		CSG servoReference=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
		.transformed(new Transform().rotZ(-90))
		
		CSG servoKeepaway = toXMin(toZMax(	new Cube(Math.abs(servoReference.getBounds().getMin().x) +
			Math.abs(servoReference.getBounds().getMax().x),
			Math.abs(servoReference.getBounds().getMin().y) +
			Math.abs(servoReference.getBounds().getMax().y),
			Math.abs(servoReference.getBounds().getMax().z)).toCSG(),
		
		)
		)
		servoKeepaway = servoKeepaway
						.movex(-Math.abs(servoReference.getBounds().getMin().x))
			
		dh = dhLinks.get(linkIndex);
		CSG nextAttachment;
		if(linkIndex>0)
			nextAttachment=getAttachment(lastConf);
		else
			nextAttachment=getAttachment(conf);
		
		
		CSG servo=servoReference
			.movez(-servoReference.getMaxZ() )
			.union(servoKeepaway)
			.movez(attachmentRodWidth/4-hornOffset/2)
			.transformed(new Transform().rotX(180))// allign to the horn
			.transformed(new Transform().rotZ(-90))// allign to the horn
			
			;
			servo= servo.makeKeepaway(printerOffset.getMM())
		double totalServoExtention =  Math.abs(servo.getMinY())
		
		boolean addNub=false;
		
		//println "Servo Keepaway Radius = "+cylandarRadius
		
		double rOffsetForNextLinkStart=dh.getR()+mountScrewKeepawaySize+0.75;
		double rOffsetForNextLink=rOffsetForNextLinkStart;
		CSG foot = null;
		if(linkIndex==	dhLinks.size()-1){
			foot=getFoot(conf);	
			//test for last link
			foot.setManufactuing(new PrepForManufacturing() {
				public CSG prep(CSG arg0) {
					return 	arg0.transformed(new Transform().rotY(90))
							.toZMin()
							.toXMin()
				}
			});

		
			foot.setManipulator(dhLinks.get(dhLinks.size()-1).getListener());
				
			
			foot.setColor(Color.GOLD);
			csg.add(foot);//This is the root that attaches to the base
			BowlerStudioController.addCsg(foot);
		}
		
		if(linkIndex==dhLinks.size()-1){
				 rOffsetForNextLink = rOffsetForNextLink-
				Math.abs(foot.toXMax().getBounds().getMin().x) +attachmentRodWidth
			
		}else{
			rOffsetForNextLink = rOffsetForNextLink-
				Math.abs(nextAttachment.toXMax().getBounds().getMin().x)
			
		}
		//println "Link # "+linkIndex+" offset = "+(rOffsetForNextLink)+" rad "+dh.getR() +" offset = "
		
		double linkThickness = dh.getD();
		if(linkThickness<attachmentBaseWidth/2)
			linkThickness=attachmentBaseWidth/2
		linkThickness +=3;
		

		servo= moveDHValues(servo,dh);
		
		double yScrewOffset = 2.5
		double ServoKeepawayRad = Math.sqrt((servoReference.getMinX()*servoReference.getMinX())+
							(servoReference.getMaxY()*servoReference.getMaxY()))	+1
		CSG upperLink = toZMin(new Cylinder(ServoKeepawayRad,linkThickness,(int)8).toCSG())
		
		double screwsCloseY = 20;
		double screwsFarY=rOffsetForNextLink+mountScrewKeepawaySize/2
		double screwsZ=upperLink.getBounds().getMax().z - linkThickness
		
		double mountCylindarRad = (mountScrewKeepawaySize+1)/2
		CSG mountHoleAttachment = new Cylinder(mountCylindarRad, // Radius at the top
		  				mountCylindarRad, // Radius at the bottom
		  				linkThickness, // Height
		  			         (int)8//resolution
		  			         ).toCSG()
		  			         .toZMin()
										
		mountHoleAttachment = mountHoleAttachment.movex(mountScrewSeperationDistance)
		mountHoleAttachment =mountHoleAttachment.union(mountHoleAttachment.movex(-2*mountScrewSeperationDistance))
		CSG mountHoleAttachmentGroup = mountHoleAttachment
									.movey(screwsFarY)
									
		CSG upperScrews = getMountScrewKeepaway()
			.movey(screwsFarY)
			.movez(screwsZ)
		if(dh.getR()>80){
			upperScrews =upperScrews.union( getMountScrewKeepaway()
				
				.movey(screwsCloseY)
				)
				mountHoleAttachmentGroup=mountHoleAttachmentGroup
				.union(
					mountHoleAttachment
									.movey(screwsCloseY)
					)
		}
		double magicNumOffset = 5;
		// adding the radius rod
		CSG rod = toYMin(
							toZMin(
								new Cube( 
									attachmentBaseWidth+magicNumOffset,
									rOffsetForNextLink,
									upperLink.getBounds().getMax().z
									).toCSG()
								)
							)
		CSG clip = toYMin(
			toZMax(
				new Cube(
					attachmentBaseWidth+12,
					9+12,
					attachmentBaseWidth+magicNumOffset
					).toCSG()
				)
			)
			.toYMax()
			.movey(rOffsetForNextLink+8)// allign to the NEXT ATTACHMENT
			.movez(linkThickness)// allign to the NEXT ATTACHMENT
		
		double upperLinkZOffset = Math.abs(servoReference.getBounds().getMax().z-magicNumOffset)
		//Build the upper link
		upperLink=upperLink.union(mountHoleAttachmentGroup,rod)
		.hull()//smooth out the shape
		CSG projection = upperLink.scalez(10)
						.intersect(clip)
		
		upperLink=upperLink.union(projection.toZMax())// add the clip in
		
		upperLink= upperLink.difference(upperScrews);
		upperLink=upperLink.movez(upperLinkZOffset)
		
		upperLink= moveDHValues(upperLink,dh).difference(servo);


			// debugging	
		projection=moveDHValues(	projection,dh)		
		projection.setManipulator(dh.getListener());
		//csg.add(projection);// view the clip
		if(linkIndex== dhLinks.size()-1)
			upperLink= upperLink.difference(foot.makeKeepaway(printerOffset.getMM()*2));
		else
			upperLink= upperLink.difference(getAttachment(null).makeKeepaway(printerOffset.getMM()*2));
		
		double LowerLinkThickness = attachmentRodWidth/2-2
		CSG lowerLink = new Cylinder(
			ServoKeepawayRad,
			 LowerLinkThickness +linkThickness+(magicNumOffset*2),
			 (int)8)
			 .toCSG()
			 .toZMin()
			 .movez(-LowerLinkThickness)
		CSG linkSweepCutout= new Cylinder(
			ServoKeepawayRad+1,
			 LowerLinkThickness +linkThickness+(magicNumOffset*2),
			 (int)8)
			 .toCSG()
			 .toZMin()
			 .movez(-attachmentRodWidth/2)
		CSG pinBlank = new Cylinder( bearingPinRadius-(printerOffset.getMM()/2),
									 LowerLinkThickness,
									 (int)20)
						.toCSG()
						.toZMin()
						.movez(-attachmentRodWidth/2)
		linkSweepCutout=	linkSweepCutout.difference(	pinBlank)	
						
		//lowerLink=lowerLink.union(pinBlank)
						
		
		lowerLink=lowerLink.transformed(new Transform().translateZ(-attachmentRodWidth/2))
		CSG lowerClip =
				
				new Cube(
					attachmentBaseWidth+(magicNumOffset*1),
					rOffsetForNextLink-2,
					LowerLinkThickness +linkThickness+(magicNumOffset*2)
					).toCSG().toZMin().toYMin()
			
			
			.transformed(new Transform().translateY(8))// allign to the NEXT ATTACHMENT
			
			.transformed(new Transform().translateZ(-attachmentRodWidth/2 -LowerLinkThickness ))
		
		lowerLink=lowerLink.union(
			lowerClip,
			mountHoleAttachmentGroup.movez(lowerLink.getMinZ())
			
			).hull();
		//Remove the divit or the bearing
		lowerLink= lowerLink.difference(
				getAttachment(null)
					.makeKeepaway((double)-0.2)
					.movez(-0.15),
				upperScrews
				.movez(6),
				linkSweepCutout
				)// allign to the NEXT ATTACHMENT);
		
		lowerLink= moveDHValues(lowerLink,dh);
		//remove the next links connector and the upper link for mating surface
		lowerLink= lowerLink.difference(upperLink,servo);
		if(linkIndex== dhLinks.size()-1)
			lowerLink= lowerLink.difference(foot.makeKeepaway(printerOffset.getMM()*2));
		else
			lowerLink= lowerLink.difference(getAttachment(conf).makeKeepaway(printerOffset.getMM()*2));
		
		
		
		upperLink.setManufactuing(new PrepForManufacturing() {
			public CSG prep(CSG arg0) {
				return reverseDHValues(arg0,dhLinks.get(linkIndex))
						.roty(180)
						.toZMin()
						.toXMin()
			}
		});
		
		
		lowerLink.setManufactuing(new PrepForManufacturing() {
			public CSG prep(CSG arg0) {
				return 	reverseDHValues(arg0,dhLinks.get(linkIndex))
						.toZMin()
						.toXMin()
			}
		});
		
		nextAttachment.setManufactuing(new PrepForManufacturing() {
			public CSG prep(CSG arg0) {
				return 	arg0
						//.transformed(new Transform().rotY(90))
						.toZMin()
						.toXMin()
			}
		});
			
		
		
		nextAttachment.setManipulator(dh.getListener());
		nextAttachment.setColor(Color.SILVER);
		servo.setManipulator(dh.getListener());
		upperLink.setColor(Color.PURPLE);
		upperLink.setManipulator(dh.getListener());
		
		
		lowerLink.setColor(Color.RED);
		lowerLink.setManipulator(dh.getListener());
		
		//csg.add(servo);// view the servo
		//BowlerStudioController.addCsg(servo);
		
		upperScrews= moveDHValues(upperScrews.movez(upperLinkZOffset),dh)
					
		upperScrews.setManipulator(dh.getListener());
		//csg.add(upperScrews);//view the screws
		
		
		if(linkIndex<dhLinks.size()-1){
			csg.add(nextAttachment);//This is the root that attaches to the base
			BowlerStudioController.addCsg(nextAttachment);
		}
		
		csg.add(upperLink);//This is the root that attaches to the base
		BowlerStudioController.addCsg(upperLink);
		
		csg.add(lowerLink);//White link forming the lower link
		BowlerStudioController.addCsg(lowerLink);
			
		
		
		return csg;
	}
};