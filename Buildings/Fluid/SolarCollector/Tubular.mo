within Buildings.Fluid.SolarCollector;
model Tubular
  extends Buildings.Fluid.SolarCollector.BaseClasses.PartialSolarCollector;
    Buildings.Fluid.SolarCollector.Data.GlazedFlatPlate.GenericGlazedFlatPlate per
    "Performance data"  annotation (choicesAllMatching=true);
  parameter Modelica.SIunits.Temperature TIn_nominal
    "Inlet temperature at nominal condition";
  BaseClasses.ASHRAESolarGain solHeaGaiNom(
    nSeg=nSeg,
    y_intercept=per.y_intercept,
    B0=per.B0,
    B1=per.B1,
    shaCoe=0.1,
    A_c=per.A,
    til=0.69813170079773)
    annotation (Placement(transformation(extent={{-10,60},{10,80}})));
  BaseClasses.ASHRAEHeatLoss heaLos(
    A_c=per.A,
    nSeg=nSeg,
    y_intercept=per.y_intercept,
    slope=per.slope,
    I_nominal=I_nominal,
    TIn_nominal=TIn_nominal,
    TEnv_nominal=TEnv_nominal,
    Cp=Cp,
    m_flow_nominal=rho*per.VperA_flow_nominal*per.A)
    annotation (Placement(transformation(extent={{-12,20},{8,40}})));
equation
  connect(temSen.T, heaLos.TFlu) annotation (Line(
      points={{-4,-16},{-20,-16},{-20,24},{-14,24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(weaBus.TDryBul, heaLos.TEnv) annotation (Line(
      points={{-100,78},{-88,78},{-88,36},{-14,36}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(HDirTil.inc, solHeaGaiNom.incAng) annotation (Line(
      points={{-59,52},{-50,52},{-50,66},{-12,66}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(HDirTil.H, solHeaGaiNom.HDirTil) annotation (Line(
      points={{-59,56},{-52,56},{-52,72},{-12,72}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(HDifTilIso.HGroDifTil, solHeaGaiNom.HGroDifTil) annotation (Line(
      points={{-59,76},{-52,76},{-52,74.8},{-12,74.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(HDifTilIso.HSkyDifTil, solHeaGaiNom.HSkyDifTil) annotation (Line(
      points={{-59,88},{-52,88},{-52,78},{-12,78}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(heaLos.QLos, QLos.Q_flow) annotation (Line(
      points={{9,30},{38,30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(solHeaGaiNom.QSol_flow, heaGai.Q_flow) annotation (Line(
      points={{11,70},{24,70},{24,70},{38,70}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                      graphics),
       defaultComponentName="solCol",
     Documentation(info="<html>
 <h4>Overview</h4>
 <p>
 This component models the tubular solar thermal collector. By default this model uses ASHRAE 93 ratings data and references the tubular data library.
 </p>
 <h4>Notice</h4>
 <p>
 1. As metioned in the reference, the SRCC incident angle modifier equation coefficients are only valid for incident angles of 60 degrees or less. 
 Because these curves can be valid yet behave poorly for angles greater than 60 degrees, the model cuts off collectors' gains of both direct and diffuse solar radiation for incident angles greater than 60 degrees. 
 <br>
 2. By default, the esitimated heat capacity of the collector without fluid is calculated based on the dry mass and the specific heat capacity of copper.
 <h4>References</h4>
 <p>
 <a href=\"http://www.energyplus.gov\">EnergyPlus 7.0.0 Engineering Reference</a>, October 13, 2011.
 <br>
 J.A. Duffie and W.A. Beckman 2006, Solar Engineering of Thermal Processes (3rd Edition), John Wiley & Sons, Inc.  
 </p>
 </html>", revisions="<html>
 <ul>
 <li>
 January 4, 2013, by Peter Grant:<br>
 First implementation.
 </li>
 </ul>
 </html>"));
end Tubular;
