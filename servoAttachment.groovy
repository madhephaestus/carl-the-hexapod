import eu.mihosoft.vrl.v3d.*;

import com.neuronrobotics.bowlerstudio.vitamins.Vitamins;
import eu.mihosoft.vrl.v3d.parametrics.*;
CSG makeAttach(){
	double attachmentRodWidth=10;
	double attachmentBaseWidth=15;
	double rodLength =  36.5;
	double pinMax = 3;
	double pinMin =3;
	LengthParameter printerOffset = new LengthParameter("printerOffset",0.4,[2,0.001])
	//ScriptingEngine.setAutoupdate(true)
	String type = "hobbyServoHorn"
	String id = "hv6214mg_1"
	StringParameter size = new StringParameter(	type+" Default",
						id,
						Vitamins.listVitaminSizes(type));
	size.setStrValue(id);
	CSG horn = Vitamins.get(type,size.getStrValue())
	horn.setParameter(size)
	
	if(args != null){
		horn=args.get(0);
		
	}
	attachmentRodWidth = horn.getMaxX()*2+2
	attachmentBaseWidth = attachmentRodWidth+5
	
	
	CSG attachmentbase = new RoundedCube(attachmentBaseWidth,attachmentBaseWidth,4)
						.cornerRadius(attachmentBaseWidth/10)
						.noCenter()
						.toCSG()
						.movex(-attachmentBaseWidth/2)
						.movey(-(attachmentBaseWidth/2))
	CSG cutOffBottomOfAttachment 	=new Cube(	attachmentBaseWidth,
						10,
						rodLength)
								.toCSG()
						.toYMax()	
						.movey(-5)
	attachmentbase=attachmentbase.difference(cutOffBottomOfAttachment)
	
	CSG post = new Cube(	attachmentRodWidth,
						10,
						rodLength
						+attachmentRodWidth/2)
						.toCSG()
						.toZMin();
						
	attachmentbase = attachmentbase
					.union(post)
					.toZMax()
					.movez( attachmentRodWidth/2);
	
	CSG hornAttach =horn
					.rotx(90)
	CSG hornKeepaway = hornAttach.union(hornAttach.movey(Math.abs(hornAttach.getMinY())*1.5)
					.intersect(new Cube(attachmentbase.getMaxX()*2,
					attachmentbase.getMaxY()*2,
					13).toCSG()))
					.makeKeepaway(printerOffset.getMM())
	attachmentbase =attachmentbase
					.difference(hornKeepaway);
	
	
	
	CSG bearingPin = new Cylinder(pinMax,pinMin, (int)5 ,(int)50).toCSG()
		.transformed(new Transform().rotX(90)).toYMax(post);
	attachmentbase =attachmentbase.difference(bearingPin);
	attachmentbase=attachmentbase
				//.union(hornAttach)
	
	
	return attachmentbase
		.transformed(new Transform().rot(-90, -90, 0))
		.setParameter(printerOffset)// add any parameters that are not used to create a primitive
		.setRegenerate({ makeAttach()})// add a regeneration function to the CSG being returrned to lonk a change event to a re-render

}
//CSGDatabase.clear()
return makeAttach()