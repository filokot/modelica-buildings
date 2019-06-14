within Buildings.DistrictEnergySystem.Loads;
package BaseClasses "BaseClasses - Package with base classes for Loads"
  extends Modelica.Icons.BasesPackage;



  model HeatFlowUA
    "Model of exponential heat transfer with thermal conductance as an input"
    extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;

    parameter Real n(min=0.5, max=2) = 1.3
     "Exponent for heat transfer: Q_flow = UA * DeltaT^n";

    Buildings.Controls.OBC.CDL.Interfaces.RealInput UA(
      quantity="ThermalConductance",
      unit="W/K",
      displayUnit="W/K",
      min=0)
      "Thermal conductance"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-120,70}),
                           iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-110,70})));
  equation
   Q_flow = UA * abs(dT)^n * sign(dT);
  //  Q_flow = UA * Buildings.Utilities.Math.Functions.powerLinearized(
  //     x=dT, n=n, x0=1E-2);
  annotation (
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
              100,100}}), graphics={
          Rectangle(
            extent={{-90,70},{90,-70}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={192,192,192},
            fillPattern=FillPattern.Forward),
          Line(
            points={{-90,70},{-90,-70}},
            thickness=0.5),
          Line(
            points={{90,70},{90,-70}},
            thickness=0.5),
          Text(
            extent={{-150,115},{150,75}},
            textString="%name",
            lineColor={0,0,255})}),
          defaultComponentName="heaFloUA");
  end HeatFlowUA;
end BaseClasses;
