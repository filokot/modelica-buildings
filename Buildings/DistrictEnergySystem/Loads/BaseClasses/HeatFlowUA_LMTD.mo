within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowUA_LMTD
  "Model for computing the heat flow rate (from liquid to the load)"
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
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TLiqSup(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Liquid supply temperature" annotation (Placement(
        transformation(extent={{-140,-90},{-100,-50}}), iconTransformation(
          extent={{-130,-90},{-90,-50}})));
   Modelica.SIunits.TemperatureDifference dT_log;
protected
  parameter Real dTdT_small = 0.5;
equation
  dT_log = Buildings.DistrictEnergySystem.Loads.BaseClasses.logMeanTempDif(
    T1_a=TLiqSup, T1_b=port_a.T, T2_a=port_b.T, T2_b=port_b.T);
  Q_flow = UA * abs(dT_log)^n * sign(dT_log);
//   Q_flow = UA * Buildings.Utilities.Math.Functions.powerLinearized(
//      dT_log, n, 0.1);
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
end HeatFlowUA_LMTD;
